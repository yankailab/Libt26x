// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Kai
// Optional integration test: this intentionally stops and resets a real camera.
#include <t26x/t26x.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
using Clock = std::chrono::steady_clock;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void check(t26x_status status, const t26x_error& error, const char* operation) {
    if (status != T26X_OK)
        throw std::runtime_error(std::string(operation) + ": " +
                                 (error.message[0] ? error.message : t26x_status_string(status)));
}

// Bounds standalone runs too, including a regression that deadlocks in stop().
struct watchdog {
    std::mutex mutex;
    std::condition_variable condition;
    bool done = false;
    std::thread thread;
    watchdog() : thread([this] {
        std::unique_lock<std::mutex> lock(mutex);
        if (!condition.wait_for(lock, std::chrono::seconds(20), [this] { return done; })) {
            std::fputs("Hardware lifecycle test exceeded 20 seconds\n", stderr);
            std::_Exit(EXIT_FAILURE);
        }
    }) {}
    ~watchdog() {
        { std::lock_guard<std::mutex> lock(mutex); done = true; }
        condition.notify_all();
        thread.join();
    }
};

struct owned_device {
    t26x_device* value = nullptr;
    ~owned_device() { if (value) t26x_close(value, nullptr); }
    void close() {
        t26x_device* closing = value;
        value = nullptr;
        t26x_error error{};
        check(t26x_close(closing, &error), error, "Close");
    }
};

struct callbacks {
    t26x_device* device = nullptr;
    std::atomic<unsigned> count{0};
    std::atomic<bool> attempted{false}, finished{false};
    std::atomic<t26x_status> stop_result{T26X_INTERNAL}, stream_error{T26X_OK};
};

void on_pose(const t26x_pose*, void* user) {
    auto& state = *static_cast<callbacks*>(user);
    ++state.count;
    if (!state.attempted.exchange(true)) {
        // The API must reject callback lifecycle calls without joining itself.
        t26x_error error{};
        state.stop_result = t26x_stop(state.device, &error);
        state.finished = true;
    }
}

void on_error(const t26x_error* error, void* user) {
    static_cast<callbacks*>(user)->stream_error = error->code;
}

template<size_t N> void finite(const float (&values)[N]) {
    for (float value : values) require(std::isfinite(value), "Pose contains non-finite data");
}

void validate(const t26x_pose& pose, const t26x_pose& previous) {
    require(pose.timestamp_ns > previous.timestamp_ns, "Pose timestamp did not advance");
    require(pose.frame_number > previous.frame_number, "Pose frame number did not advance");
    require(pose.tracker_confidence <= 3 && pose.mapper_confidence <= 3, "Invalid pose confidence");
    finite(pose.translation); finite(pose.rotation); finite(pose.velocity);
    finite(pose.acceleration); finite(pose.angular_velocity); finite(pose.angular_acceleration);
}

void open_wait(owned_device& camera, t26x_open_options options) {
    const auto deadline = Clock::now() + std::chrono::milliseconds(options.boot_timeout_ms);
    t26x_error error{};
    for (;;) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
        require(remaining > 0, "Camera did not appear within the requested boot timeout");
        options.boot_timeout_ms = static_cast<uint32_t>(remaining);
        const auto status = t26x_open(&options, &camera.value, &error);
        if (status == T26X_OK) return;
        // Immediately after reset the camera may not yet appear in enumeration.
        if (status != T26X_NOT_FOUND) check(status, error, "Open");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void cycle(t26x_device* camera, unsigned index) {
    callbacks state;
    state.device = camera;
    t26x_stream_config config;
    t26x_stream_config_init(&config);
    config.on_pose = on_pose;
    config.on_error = on_error;
    config.user = &state;
    t26x_error error{};
    check(t26x_start(camera, &config, &error), error, "Start");
    // Keep callback storage alive if a later check throws during streaming.
    struct stop_guard {
        t26x_device* camera;
        ~stop_guard() { t26x_stop(camera, nullptr); }
    } stop{camera};
    require(t26x_is_streaming(camera) != 0, "Start did not enter streaming state");
    t26x_pose previous{};
    for (unsigned i = 0; i < 20; ++i) {
        t26x_pose pose{};
        check(t26x_wait_for_pose(camera, &pose, i == 0 ? 3000 : 1000, &error), error, "Wait for pose");
        validate(pose, previous);
        previous = pose;
    }
    require(state.finished && state.stop_result == T26X_INVALID_STATE,
            "Callback stop must return T26X_INVALID_STATE");
    require(state.stream_error == T26X_OK, "Streaming error callback was invoked");
    uint32_t tracker = 0, mapper = 0;
    check(t26x_confidence(camera, &tracker, &mapper, &error), error, "Read confidence");
    require(tracker <= 3 && mapper <= 3, "Invalid cached confidence");
    check(t26x_stop(camera, &error), error, "Stop");
    check(t26x_stop(camera, &error), error, "Repeated stop");
    require(t26x_is_streaming(camera) == 0, "Stop did not clear streaming state");
    t26x_pose stopped{};
    require(t26x_wait_for_pose(camera, &stopped, 10, &error) == T26X_INVALID_STATE,
            "Waiting after stop must return T26X_INVALID_STATE");
    const auto count = state.count.load();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    require(state.count == count, "Callback ran after stop returned");
    std::printf("Cycle %u: %u callbacks, confidence %u/%u\n", index, count, tracker, mapper);
    std::fflush(stdout);
}
} // namespace

int main(int argc, char** argv) {
    try {
        t26x_open_options options;
        t26x_open_options_init(&options);
        options.boot_timeout_ms = 5000;
        for (int i = 1; i < argc; ++i) {
            if (!std::strcmp(argv[i], "--help")) {
                std::puts("Usage: t26x-test-hardware [--firmware FILE] [--serial SERIAL] [--boot-timeout-ms N]\n"
                          "Runs for at most 20 seconds; resets and reopens the selected camera.");
                return 0;
            }
            require(i + 1 < argc, "Missing option value");
            const char* option = argv[i++];
            if (!std::strcmp(option, "--firmware")) options.firmware_path = argv[i];
            else if (!std::strcmp(option, "--serial")) options.serial = argv[i];
            else if (!std::strcmp(option, "--boot-timeout-ms")) {
                size_t end = 0;
                const auto value = std::stoul(argv[i], &end);
                require(end == std::strlen(argv[i]) && value > 0 && value <= 10000,
                        "Boot timeout must be between 1 and 10000 milliseconds");
                options.boot_timeout_ms = static_cast<uint32_t>(value);
            } else throw std::runtime_error(std::string("Unknown option: ") + option);
        }
        watchdog timeout;
        owned_device camera;
        open_wait(camera, options);
        t26x_error error{};
        t26x_device_info info{};
        check(t26x_get_device_info(camera.value, &info, &error), error, "Read device information");
        std::printf("T265 %s firmware %s USB %d Mbps\n", info.serial, info.firmware_version, info.usb_speed_mbps);
        std::fflush(stdout);
        const std::string serial = info.serial;
        options.serial = serial.c_str();
        for (unsigned i = 1; i <= 3; ++i) cycle(camera.value, i);
        check(t26x_reset(camera.value, &error), error, "Reset");
        require(t26x_is_streaming(camera.value) == 0, "Reset left the camera streaming");
        camera.close();
        open_wait(camera, options);
        cycle(camera.value, 4);
        camera.close();
        std::puts("Hardware lifecycle test passed");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "Hardware lifecycle test failed: %s\n", error.what());
        return 1;
    }
}

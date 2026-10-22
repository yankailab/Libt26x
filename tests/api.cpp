// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Kai
#include <t26x/t26x.hpp>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace {
unsigned checks = 0;

void require(bool condition, const char *message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

template<class Action> void invalid_argument(Action action) {
    t26x_error detail;
    std::memset(&detail, 0xa5, sizeof detail);
    require(action(&detail) == T26X_INVALID_ARGUMENT, "invalid C arguments must return T26X_INVALID_ARGUMENT");
    require(detail.code == T26X_INVALID_ARGUMENT, "error code matches return value");
    require(detail.message[0] != '\0', "errors have a diagnostic");
    require(std::memchr(detail.message, '\0', sizeof detail.message) != nullptr, "error diagnostics are terminated");
    require(action(nullptr) == T26X_INVALID_ARGUMENT, "error output may be omitted");
}

template<class Action> void closed_cpp_call(Action action) {
    bool threw = false;
    try { action(); }
    catch (const t26x::error &error) {
        threw = true;
        require(error.code() == T26X_INVALID_ARGUMENT, "closed C++ handle preserves C error status");
        require(error.what()[0] != '\0', "C++ exception carries diagnostic");
    }
    require(threw, "operation on closed C++ device throws t26x::error");
}

void test_defaults() {
    t26x_open_options options;
    std::memset(&options, 0xa5, sizeof options);
    t26x_open_options_init(&options);
    require(options.serial == nullptr && options.firmware_path == nullptr, "default open chooses first device and bundled firmware");
    require(options.boot_timeout_ms > 0, "default boot timeout is bounded and nonzero");
    t26x_stream_config config;
    std::memset(&config, 0xa5, sizeof config);
    t26x_stream_config_init(&config);
    require(config.enable_fisheye == 0, "default stream is USB2-compatible pose only");
    require(!config.on_pose && !config.on_fisheye && !config.on_error && !config.user,
            "default callback configuration is empty");
    t26x_open_options_init(nullptr);
    t26x_stream_config_init(nullptr);
    require(t26x::default_open_options().boot_timeout_ms == options.boot_timeout_ms,
            "C++ open defaults follow C defaults");
    require(t26x::default_stream_config().enable_fisheye == 0, "C++ stream defaults follow C defaults");
}

void test_c_arguments() {
    t26x_pose pose{};
    pose.timestamp_ns = 42;
    t26x_device_info info{};
    info.usb_speed_mbps = 123;
    uint32_t tracker = 99, mapper = 100;
    // A missing output handle is rejected before any libusb/device operation.
    invalid_argument([](t26x_error *error) { return t26x_open(nullptr, nullptr, error); });
    invalid_argument([](t26x_error *error) { return t26x_start(nullptr, nullptr, error); });
    invalid_argument([](t26x_error *error) { return t26x_stop(nullptr, error); });
    invalid_argument([](t26x_error *error) { return t26x_reset(nullptr, error); });
    invalid_argument([&](t26x_error *error) { return t26x_get_device_info(nullptr, &info, error); });
    invalid_argument([](t26x_error *error) { return t26x_get_device_info(nullptr, nullptr, error); });
    invalid_argument([&](t26x_error *error) { return t26x_get_pose(nullptr, &pose, error); });
    invalid_argument([](t26x_error *error) { return t26x_get_pose(nullptr, nullptr, error); });
    invalid_argument([&](t26x_error *error) { return t26x_wait_for_pose(nullptr, &pose, 0, error); });
    invalid_argument([](t26x_error *error) { return t26x_wait_for_pose(nullptr, nullptr, 1, error); });
    invalid_argument([&](t26x_error *error) { return t26x_confidence(nullptr, &tracker, &mapper, error); });
    invalid_argument([&](t26x_error *error) { return t26x_confidence(nullptr, nullptr, &mapper, error); });
    invalid_argument([&](t26x_error *error) { return t26x_confidence(nullptr, &tracker, nullptr, error); });
    require(pose.timestamp_ns == 42 && info.usb_speed_mbps == 123, "failed reads do not modify output structures");
    require(tracker == 99 && mapper == 100, "failed confidence query does not modify outputs");
    require(t26x_is_streaming(nullptr) == 0, "null device is not streaming");
    t26x_error detail{};
    detail.code = T26X_INTERNAL;
    detail.message[0] = 'X';
    require(t26x_close(nullptr, &detail) == T26X_OK, "close null handle is safe");
    require(detail.code == T26X_OK && !detail.message[0], "successful close clears old errors");
    require(t26x_close(nullptr, nullptr) == T26X_OK, "close permits both optional pointers null");
}

void test_cpp_owner() {
    static_assert(!std::is_copy_constructible<t26x::device>::value, "device cannot be copied");
    static_assert(!std::is_copy_assignable<t26x::device>::value, "device cannot be copy assigned");
    static_assert(std::is_nothrow_move_constructible<t26x::device>::value, "device is safely movable");
    static_assert(std::is_nothrow_destructible<t26x::device>::value, "destructor cannot throw");
    t26x::device camera;
    require(!camera.is_open() && !camera.is_streaming() && !camera.native_handle(), "default C++ device is closed");
    camera.close();
    camera.close();
    closed_cpp_call([&] { camera.start(); });
    closed_cpp_call([&] { camera.stop(); });
    closed_cpp_call([&] { camera.reset(); });
    closed_cpp_call([&] { camera.info(); });
    closed_cpp_call([&] { camera.pose(); });
    closed_cpp_call([&] { camera.wait_for_pose(0); });
    closed_cpp_call([&] { camera.confidence(); });
    t26x::device moved(std::move(camera));
    require(!camera.is_open() && !moved.is_open(), "moving closed owner preserves closed state");
    camera = std::move(moved);
    require(!camera.is_open() && !moved.is_open(), "move assignment of closed owner is safe");
    t26x::device &alias = camera;
    camera = std::move(alias);
    require(!camera.is_open(), "self move preserves ownership state");
}
} // namespace

int main() {
    try {
        test_defaults();
        test_c_arguments();
        test_cpp_owner();
        require(t26x_version() && t26x_version()[0], "library reports its version");
        for (int value = T26X_OK; value <= T26X_INTERNAL; ++value) {
            const char *message = t26x_status_string(static_cast<t26x_status>(value));
            require(message && message[0], "every public status has a printable description");
        }
        require(t26x_status_string(static_cast<t26x_status>(-1)) != nullptr, "unknown status has a safe description");
        std::printf("API: %u checks passed without opening USB devices.\n", checks);
        return EXIT_SUCCESS;
    } catch (const std::exception &error) {
        std::fprintf(stderr, "API check failed: %s\n", error.what());
        return EXIT_FAILURE;
    }
}

/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kai
 */
#include <t26x/t26x.hpp>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

namespace {

volatile std::sig_atomic_t interrupted = 0;

void interrupt_handler(int) { interrupted = 1; }

struct statistics {
    std::atomic<unsigned long long> poses{0};
    std::atomic<unsigned long long> left{0};
    std::atomic<unsigned long long> right{0};
    std::atomic<unsigned long long> invalid_frames{0};
    std::atomic<unsigned long long> errors{0};
    std::atomic<uint32_t> tracker{0};
    std::atomic<uint32_t> mapper{0};
};

void on_pose(const t26x_pose *pose, void *user) {
    statistics &stats = *static_cast<statistics *>(user);
    stats.poses.fetch_add(1, std::memory_order_relaxed);
    stats.tracker.store(pose->tracker_confidence, std::memory_order_relaxed);
    stats.mapper.store(pose->mapper_confidence, std::memory_order_relaxed);
}

void on_fisheye(const t26x_fisheye_frame *frame, void *user) {
    statistics &stats = *static_cast<statistics *>(user);
    if (!frame->data || !frame->width || !frame->height ||
        frame->stride < frame->width ||
        frame->data_size / frame->height < frame->stride ||
        (frame->camera_index != 1 && frame->camera_index != 2)) {
        stats.invalid_frames.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    // Copy pixels here if needed after return; the USB reader owns frame->data.
    if (frame->camera_index == 1)
        stats.left.fetch_add(1, std::memory_order_relaxed);
    else
        stats.right.fetch_add(1, std::memory_order_relaxed);
}

void on_error(const t26x_error *error, void *user) {
    statistics &stats = *static_cast<statistics *>(user);
    stats.errors.fetch_add(1, std::memory_order_relaxed);
    std::fprintf(stderr, "Stream error: %s (%s)\n", error->message,
                 t26x_status_string(error->code));
}

void usage(const char *program) {
    std::printf("Usage: %s [--seconds N] [--fisheye] [--reset]\n"
                "       [--serial SERIAL] [--firmware PATH] [--help]\n"
                "Capture for 5 seconds by default (N: 1..3600).\n"
                "--fisheye requires USB 3 and checks both cameras.\n"
                "--reset resets the camera after capture and closes its handle.\n",
                program);
}

} // namespace

int main(int argc, char **argv) {
    long duration = 5;
    bool fisheye = false;
    bool reset = false;
    t26x_open_options options = t26x::default_open_options();
    for (int index = 1; index < argc; ++index) {
        if (std::strcmp(argv[index], "--help") == 0) {
            usage(argv[0]);
            return EXIT_SUCCESS;
        } else if (std::strcmp(argv[index], "--fisheye") == 0) {
            fisheye = true;
        } else if (std::strcmp(argv[index], "--reset") == 0) {
            reset = true;
        } else if (std::strcmp(argv[index], "--seconds") == 0 && index + 1 < argc) {
            char *end = nullptr;
            const char *value = argv[++index];
            errno = 0;
            duration = std::strtol(value, &end, 10);
            if (errno || !*value || *end || duration < 1 || duration > 3600) {
                std::fprintf(stderr, "Duration must be an integer from 1 to 3600 seconds.\n");
                return EXIT_FAILURE;
            }
        } else if (std::strcmp(argv[index], "--serial") == 0 && index + 1 < argc) {
            options.serial = argv[++index];
        } else if (std::strcmp(argv[index], "--firmware") == 0 && index + 1 < argc) {
            options.firmware_path = argv[++index];
        } else {
            std::fprintf(stderr, "Unknown option or missing value: %s\n", argv[index]);
            usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    std::signal(SIGINT, interrupt_handler);
    std::signal(SIGTERM, interrupt_handler);
    // Declare stats before the device so callbacks cannot outlive their context
    // when an exception causes the device destructor to stop streaming.
    statistics stats;
    try {
        t26x::device camera(options);
        const t26x_device_info info = camera.info();
        std::printf("Libt26x %s, camera %s, firmware %s, USB %d Mbps\n",
                    t26x_version(), info.serial, info.firmware_version, info.usb_speed_mbps);
        t26x_stream_config config = t26x::default_stream_config();
        config.enable_fisheye = fisheye ? 1 : 0;
        config.on_pose = on_pose;
        config.on_fisheye = on_fisheye;
        config.on_error = on_error;
        config.user = &stats;
        camera.start(config);
        const auto started = std::chrono::steady_clock::now();
        const auto deadline = started + std::chrono::seconds(duration);
        while (!interrupted && std::chrono::steady_clock::now() < deadline &&
               stats.errors.load(std::memory_order_relaxed) == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        camera.stop();
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - started).count();
        std::printf("Captured %.2f seconds: poses=%llu (%.1f Hz), left=%llu, right=%llu, "
                    "confidence=%u/%u, invalid_frames=%llu, errors=%llu\n",
                    elapsed, stats.poses.load(), stats.poses.load() / elapsed,
                    stats.left.load(), stats.right.load(),
                    static_cast<unsigned>(stats.tracker.load()),
                    static_cast<unsigned>(stats.mapper.load()),
                    stats.invalid_frames.load(), stats.errors.load());
        bool passed = stats.poses.load() > 0 && stats.errors.load() == 0 &&
                      stats.invalid_frames.load() == 0;
        if (fisheye && (stats.left.load() == 0 || stats.right.load() == 0)) {
            std::fprintf(stderr, "Both fisheye cameras must produce frames.\n");
            passed = false;
        }
        if (stats.poses.load() == 0) std::fprintf(stderr, "No pose samples received.\n");
        if (reset) {
            camera.reset();
            std::printf("Camera reset; reopen it to resume streaming.\n");
        } else {
            camera.close();
        }
        return passed ? EXIT_SUCCESS : EXIT_FAILURE;
    } catch (const t26x::error &error) {
        std::fprintf(stderr, "Libt26x: %s (%s)\n", error.what(), t26x_status_string(error.code()));
    } catch (const std::exception &error) {
        std::fprintf(stderr, "%s\n", error.what());
    }
    return EXIT_FAILURE;
}

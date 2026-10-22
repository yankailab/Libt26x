/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kai
 */
#define _POSIX_C_SOURCE 200809L
#include <t26x/t26x.h>

#include <errno.h>
#include <inttypes.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static volatile sig_atomic_t interrupted = 0;

static void interrupt_handler(int signal_number) {
    (void)signal_number;
    interrupted = 1;
}

static double monotonic_seconds(void) {
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) return -1.0;
    return (double)now.tv_sec + (double)now.tv_nsec * 1e-9;
}

static void report_error(const char *operation, const t26x_error *error) {
    fprintf(stderr, "%s: %s (%s)\n", operation, error->message,
            t26x_status_string(error->code));
}

int main(int argc, char **argv) {
    long duration = 5;
    t26x_device *device = NULL;
    t26x_error error = {T26X_OK, {0}};
    t26x_open_options options;
    t26x_stream_config config;
    t26x_device_info info;
    uint64_t samples = 0;
    double started;
    int result = EXIT_SUCCESS;

    if (argc == 2 && strcmp(argv[1], "--help") == 0) {
        printf("Usage: %s [seconds: 1..3600, default 5]\n", argv[0]);
        return EXIT_SUCCESS;
    }
    if (argc > 2) {
        fprintf(stderr, "Usage: %s [seconds: 1..3600, default 5]\n", argv[0]);
        return EXIT_FAILURE;
    }
    if (argc == 2) {
        char *end = NULL;
        errno = 0;
        duration = strtol(argv[1], &end, 10);
        if (errno || !argv[1][0] || *end || duration < 1 || duration > 3600) {
            fprintf(stderr, "Duration must be an integer from 1 to 3600 seconds.\n");
            return EXIT_FAILURE;
        }
    }

    signal(SIGINT, interrupt_handler);
    signal(SIGTERM, interrupt_handler);
    t26x_open_options_init(&options);
    t26x_stream_config_init(&config);
    if (t26x_open(&options, &device, &error) != T26X_OK) {
        report_error("open", &error);
        return EXIT_FAILURE;
    }
    if (t26x_get_device_info(device, &info, &error) != T26X_OK) {
        report_error("device info", &error);
        result = EXIT_FAILURE;
        goto cleanup;
    }
    printf("Libt26x %s, camera %s, firmware %s, USB %d Mbps\n",
           t26x_version(), info.serial, info.firmware_version, info.usb_speed_mbps);
    if (t26x_start(device, &config, &error) != T26X_OK) {
        report_error("start", &error);
        result = EXIT_FAILURE;
        goto cleanup;
    }
    started = monotonic_seconds();
    if (started < 0) {
        perror("clock_gettime");
        result = EXIT_FAILURE;
        goto cleanup;
    }
    while (!interrupted) {
        t26x_pose pose;
        t26x_status status;
        double now = monotonic_seconds();
        if (now < 0) {
            perror("clock_gettime");
            result = EXIT_FAILURE;
            break;
        }
        if (now - started >= (double)duration) break;
        status = t26x_wait_for_pose(device, &pose, 1000, &error);
        if (status == T26X_TIMEOUT) continue;
        if (status != T26X_OK) {
            report_error("wait for pose", &error);
            result = EXIT_FAILURE;
            break;
        }
        ++samples;
        if (samples == 1 || samples % 200 == 0) {
            printf("frame=%" PRIu64 " time=%" PRIu64 " ns position=(%.4f, %.4f, %.4f) m "
                   "quaternion=(%.4f, %.4f, %.4f, %.4f) confidence=%u/%u\n",
                   pose.frame_number, pose.timestamp_ns,
                   pose.translation[0], pose.translation[1], pose.translation[2],
                   pose.rotation[0], pose.rotation[1], pose.rotation[2], pose.rotation[3],
                   (unsigned)pose.tracker_confidence, (unsigned)pose.mapper_confidence);
        }
    }
    printf("Received %" PRIu64 " pose samples.\n", samples);
    if (!samples) {
        fprintf(stderr, "No pose samples received.\n");
        result = EXIT_FAILURE;
    }
    if (t26x_stop(device, &error) != T26X_OK) {
        report_error("stop", &error);
        result = EXIT_FAILURE;
    }

cleanup:
    if (t26x_close(device, &error) != T26X_OK) {
        report_error("close", &error);
        result = EXIT_FAILURE;
    }
    return result;
}

/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kai
 */
#ifndef T26X_T26X_H
#define T26X_T26X_H

#include <stddef.h>
#include <stdint.h>

#if defined(__GNUC__) || defined(__clang__)
#define T26X_API __attribute__((visibility("default")))
#else
#define T26X_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct t26x_device t26x_device;

typedef enum t26x_status {
    T26X_OK = 0,
    T26X_INVALID_ARGUMENT = 1,
    T26X_NOT_FOUND = 2,
    T26X_ACCESS = 3,
    T26X_BUSY = 4,
    T26X_IO = 5,
    T26X_TIMEOUT = 6,
    T26X_UNSUPPORTED = 7,
    T26X_PROTOCOL = 8,
    T26X_INVALID_STATE = 9,
    T26X_FIRMWARE = 10,
    T26X_INTERNAL = 11
} t26x_status;

typedef struct t26x_error {
    t26x_status code;
    char message[256];
} t26x_error;

typedef struct t26x_open_options {
    /* NULL selects the first available device. */
    const char *serial;
    /* NULL uses the firmware selected when Libt26x was built. */
    const char *firmware_path;
    uint32_t boot_timeout_ms;
} t26x_open_options;

typedef struct t26x_pose {
    /* Device-clock timestamp in nanoseconds; not a wall-clock timestamp. */
    uint64_t timestamp_ns;
    uint64_t frame_number;
    /* Right-handed coordinates: +X right, +Y up, +Z toward the user.
     * Linear units are metres, metres/second, and metres/second squared.
     * Angular units are radians/second and radians/second squared. */
    float translation[3];
    float velocity[3];
    float acceleration[3];
    /* Unit quaternion in X, Y, Z, W order. */
    float rotation[4];
    float angular_velocity[3];
    float angular_acceleration[3];
    /* 0: failed, 1: low, 2: medium, 3: high. */
    uint32_t tracker_confidence;
    uint32_t mapper_confidence;
} t26x_pose;

typedef struct t26x_fisheye_frame {
    /* Device-clock timestamp in nanoseconds; not a wall-clock timestamp. */
    uint64_t timestamp_ns;
    uint32_t frame_number;
    /* 1: left camera; 2: right camera. */
    uint8_t camera_index;
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    uint32_t exposure_us;
    float gain;
    /* Unrectified Y8 pixels. Storage is valid only during this callback. */
    const uint8_t *data;
    size_t data_size;
} t26x_fisheye_frame;

/* Callbacks run on internal USB reader threads. They must return promptly,
 * must not throw C++ exceptions, and must not call lifecycle methods or wait
 * for another pose (open/start/stop/reset/close/wait_for_pose).
 * Each supplied data pointer is valid only until the callback returns.
 * Callbacks on different reader threads may run concurrently. */
typedef void (*t26x_pose_callback)(const t26x_pose *pose, void *user);
typedef void (*t26x_fisheye_callback)(const t26x_fisheye_frame *frame, void *user);
typedef void (*t26x_error_callback)(const t26x_error *error, void *user);

typedef struct t26x_stream_config {
    /* Pose is always enabled. Fisheye streaming requires USB 3. */
    int enable_fisheye;
    t26x_pose_callback on_pose;
    t26x_fisheye_callback on_fisheye;
    t26x_error_callback on_error;
    void *user;
} t26x_stream_config;

typedef struct t26x_device_info {
    char serial[32];
    char firmware_version[32];
    int usb_speed_mbps;
} t26x_device_info;

/* Initialize option structures before changing individual fields. */
T26X_API void t26x_open_options_init(t26x_open_options *options);
T26X_API void t26x_stream_config_init(t26x_stream_config *config);

/* Error output pointers are optional. No function lets C++ exceptions cross
 * the C ABI. Lifecycle operations must be externally serialized. Do not close
 * a device while another thread is using its handle, including in a read call.
 * Reads may otherwise run concurrently with streaming and with each other. */
T26X_API t26x_status t26x_open(const t26x_open_options *options,
                              t26x_device **device, t26x_error *error);
T26X_API t26x_status t26x_start(t26x_device *device,
                               const t26x_stream_config *config, t26x_error *error);
/* Stops streaming and waits for in-flight callbacks to finish. */
T26X_API t26x_status t26x_stop(t26x_device *device, t26x_error *error);
/* Hardware reset invalidates the handle. Close it, then open a new handle.
 * A failed reset attempt may also invalidate it; it must still be closed. */
T26X_API t26x_status t26x_reset(t26x_device *device, t26x_error *error);
/* Stops streaming and releases the handle, including on USB stop errors.
 * Do not reuse the handle after this call. Calling from a callback returns
 * T26X_INVALID_STATE without releasing it. A NULL device is accepted. */
T26X_API t26x_status t26x_close(t26x_device *device, t26x_error *error);

T26X_API t26x_status t26x_get_device_info(t26x_device *device,
                                        t26x_device_info *info, t26x_error *error);
/* Copies the latest pose; returns T26X_INVALID_STATE before the first pose. */
T26X_API t26x_status t26x_get_pose(t26x_device *device, t26x_pose *pose,
                                 t26x_error *error);
/* Waits for a NEW pose arriving after this call, for at most timeout_ms.
 * Do not call from a callback on this device. */
T26X_API t26x_status t26x_wait_for_pose(t26x_device *device, t26x_pose *pose,
                                      uint32_t timeout_ms, t26x_error *error);
/* Returns confidence from the latest cached pose. Both outputs are required. */
T26X_API t26x_status t26x_confidence(t26x_device *device, uint32_t *tracker,
                                   uint32_t *mapper, t26x_error *error);
T26X_API int t26x_is_streaming(t26x_device *device);
T26X_API const char *t26x_version(void);
T26X_API const char *t26x_status_string(t26x_status status);

#ifdef __cplusplus
}
#endif

#endif /* T26X_T26X_H */

/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kai
 */
#ifndef T26X_T26X_HPP
#define T26X_T26X_HPP

#include <t26x/t26x.h>

#include <stdexcept>
#include <string>
#include <utility>

namespace t26x {

class error : public std::runtime_error {
public:
    error(t26x_status status, const std::string &message)
        : std::runtime_error(message), status_(status) {}

    t26x_status code() const noexcept { return status_; }

private:
    t26x_status status_;
};

inline t26x_open_options default_open_options() {
    t26x_open_options options;
    t26x_open_options_init(&options);
    return options;
}

inline t26x_stream_config default_stream_config() {
    t26x_stream_config config;
    t26x_stream_config_init(&config);
    return config;
}

struct confidence_levels {
    uint32_t tracker;
    uint32_t mapper;
};

/* Move-only owner. The C API's threading and callback rules also apply here.
 * An instance must outlive all callbacks and all concurrent read operations. */
class device {
public:
    device() noexcept : handle_(nullptr) {}

    explicit device(const t26x_open_options &options) : handle_(nullptr) {
        open(options);
    }

    ~device() noexcept {
        if (handle_) t26x_close(handle_, nullptr);
    }

    device(const device &) = delete;
    device &operator=(const device &) = delete;

    device(device &&other) noexcept : handle_(other.handle_) {
        other.handle_ = nullptr;
    }

    device &operator=(device &&other) {
        if (this != &other) {
            close();
            handle_ = other.handle_;
            other.handle_ = nullptr;
        }
        return *this;
    }

    void open(const t26x_open_options &options = default_open_options()) {
        if (handle_) throw error(T26X_INVALID_STATE, "device is already open");
        t26x_error detail = {};
        check(t26x_open(&options, &handle_, &detail), detail);
    }

    void start(const t26x_stream_config &config = default_stream_config()) {
        t26x_error detail = {};
        check(t26x_start(handle_, &config, &detail), detail);
    }

    void stop() {
        t26x_error detail = {};
        check(t26x_stop(handle_, &detail), detail);
    }

    /* Resets hardware and closes the invalidated handle. Call open() again
     * after the USB device has re-enumerated before starting a new stream. */
    void reset() {
        t26x_error detail = {};
        check(t26x_reset(handle_, &detail), detail);
        close();
    }

    void close() {
        if (!handle_) return;
        t26x_error detail = {};
        const t26x_status status = t26x_close(handle_, &detail);
        // C close consumes the handle even if stopping USB streaming failed.
        // A callback lifecycle rejection is the sole ownership exception.
        if (status != T26X_INVALID_STATE) handle_ = nullptr;
        check(status, detail);
    }

    bool is_open() const noexcept { return handle_ != nullptr; }
    bool is_streaming() const noexcept { return t26x_is_streaming(handle_) != 0; }

    t26x_device_info info() const {
        t26x_device_info result = {};
        t26x_error detail = {};
        check(t26x_get_device_info(handle_, &result, &detail), detail);
        return result;
    }

    t26x_pose pose() const {
        t26x_pose result = {};
        t26x_error detail = {};
        check(t26x_get_pose(handle_, &result, &detail), detail);
        return result;
    }

    t26x_pose wait_for_pose(uint32_t timeout_ms = 1000) const {
        t26x_pose result = {};
        t26x_error detail = {};
        check(t26x_wait_for_pose(handle_, &result, timeout_ms, &detail), detail);
        return result;
    }

    confidence_levels confidence() const {
        confidence_levels result = {};
        t26x_error detail = {};
        check(t26x_confidence(handle_, &result.tracker, &result.mapper, &detail), detail);
        return result;
    }

    /* Borrowed pointer: ownership remains with this instance. */
    t26x_device *native_handle() const noexcept { return handle_; }

private:
    static void check(t26x_status status, const t26x_error &detail) {
        if (status != T26X_OK)
            throw error(status, detail.message[0] ? detail.message : t26x_status_string(status));
    }

    t26x_device *handle_;
};

} // namespace t26x

#endif /* T26X_T26X_HPP */

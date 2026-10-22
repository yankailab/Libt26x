// SPDX-License-Identifier: Apache-2.0
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.
// Copyright (c) 2026 Kai
// Modified from librealsense v2.53.1 src/tm2/tm-device.cpp and tm-boot.h:
// standalone Linux/libusb transport, C ABI, bounded decoding and owned threads.
#include <t26x/t26x.h>
#include "protocol.hpp"
#include <libusb.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <fstream>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#if LIBT26X_HAVE_EMBEDDED_FIRMWARE
extern "C" {
extern const unsigned char t26x_firmware_start[], t26x_firmware_end[];
}
#endif

namespace {
using namespace t265;
using namespace t26x_detail;
using Clock = std::chrono::steady_clock;
constexpr unsigned command_timeout = 5000;
constexpr uint8_t command_out = 0x02, command_in = 0x82, video_in = 0x81, pose_in = 0x83;
thread_local const t26x_device* callback_device = nullptr;

struct failure : std::runtime_error {
    t26x_status code;
    failure(t26x_status c, const std::string& message) : std::runtime_error(message), code(c) {}
};

t26x_status set_error(t26x_error* error, t26x_status code, const char* message) noexcept {
    if (error) { error->code = code; std::snprintf(error->message, sizeof error->message, "%s", message); }
    return code;
}

template<class F> t26x_status protect(t26x_error* error, F&& action) noexcept {
    try { action(); return set_error(error, T26X_OK, ""); }
    catch (const failure& e) { return set_error(error, e.code, e.what()); }
    catch (const std::exception& e) { return set_error(error, T26X_INTERNAL, e.what()); }
    catch (...) { return set_error(error, T26X_INTERNAL, "Unknown C++ exception"); }
}

void require(bool condition, t26x_status code, const char* message) {
    if (!condition) throw failure(code, message);
}

void usb_check(int result, const char* operation) {
    if (result >= 0) return;
    t26x_status code = T26X_IO;
    if (result == LIBUSB_ERROR_ACCESS) code = T26X_ACCESS;
    else if (result == LIBUSB_ERROR_BUSY) code = T26X_BUSY;
    else if (result == LIBUSB_ERROR_NO_DEVICE) code = T26X_NOT_FOUND;
    else if (result == LIBUSB_ERROR_TIMEOUT) code = T26X_TIMEOUT;
    else if (result == LIBUSB_ERROR_NOT_SUPPORTED) code = T26X_UNSUPPORTED;
    throw failure(code, std::string(operation) + ": " + libusb_error_name(result));
}

struct usb_list {
    libusb_device** devices = nullptr;
    explicit usb_list(libusb_context* context) {
        const auto count = libusb_get_device_list(context, &devices);
        if (count < 0) usb_check(static_cast<int>(count), "Enumerate USB devices");
    }
    ~usb_list() { if (devices) libusb_free_device_list(devices, 1); }
};

struct usb_handle {
    libusb_device_handle* value = nullptr;
    bool claimed = false;
    ~usb_handle() { close(); }
    void close() noexcept {
        if (value) { if (claimed) libusb_release_interface(value, 0); libusb_close(value); }
        value = nullptr; claimed = false;
    }
    void open(libusb_device* device) {
        close();
        usb_check(libusb_open(device, &value), "Open T265 (check udev permissions)");
        usb_check(libusb_claim_interface(value, 0), "Claim T265 interface 0 (close other camera applications)");
        claimed = true;
    }
};

std::string location(libusb_device* device) {
    uint8_t ports[8];
    int count = libusb_get_port_numbers(device, ports, sizeof ports);
    std::string result = std::to_string(libusb_get_bus_number(device));
    if (count > 0) for (int i = 0; i < count; ++i) result += "." + std::to_string(ports[i]);
    else result += ":" + std::to_string(libusb_get_device_address(device));
    return result;
}

struct callback_scope {
    const t26x_device* previous;
    explicit callback_scope(const t26x_device* device) : previous(callback_device) { callback_device = device; }
    ~callback_scope() { callback_device = previous; }
};
} // namespace

struct t26x_device {
    libusb_context* context = nullptr;
    usb_handle handle;
    t26x_device_info info{};
    t26x_stream_config config{};
    std::vector<t265::supported_raw_stream_libtm_message> profiles;
    t265::supported_raw_stream_libtm_message cameras[2]{};
    std::atomic<bool> running{false}, stopping{false};
    bool started = false, reset_done = false;
    std::thread pose_thread, video_thread;
    std::mutex pose_mutex;
    std::condition_variable pose_ready;
    t26x_pose latest{};
    uint64_t generation = 0;
    bool have_pose = false;
    t26x_error stream_error{};

    ~t26x_device() {
        { std::lock_guard<std::mutex> lock(pose_mutex); running = false; }
        pose_ready.notify_all();
        if (pose_thread.joinable()) pose_thread.join();
        if (video_thread.joinable()) video_thread.join();
        handle.close();
        if (context) libusb_exit(context);
    }

    std::vector<uint8_t> command(const void* request, size_t length) {
        const auto h = read<bulk_message_request_header>(static_cast<const uint8_t*>(request), length);
        int sent = 0;
        usb_check(libusb_bulk_transfer(handle.value, command_out,
                  const_cast<unsigned char*>(static_cast<const unsigned char*>(request)),
                  static_cast<int>(length), &sent, command_timeout), "Send T265 command");
        require(size_t(sent) == length, T26X_PROTOCOL, "Short T265 command write");
        std::vector<uint8_t> reply(1024);
        int received = 0;
        usb_check(libusb_bulk_transfer(handle.value, command_in, reply.data(),
                  static_cast<int>(reply.size()), &received, command_timeout), "Read T265 command reply");
        require(received >= int(sizeof(bulk_message_response_header)), T26X_PROTOCOL, "Truncated T265 command reply");
        reply.resize(received);
        const auto response = read<bulk_message_response_header>(reply.data(), reply.size());
        require(response.dwLength == reply.size() && response.wMessageID == h.wMessageID,
                T26X_PROTOCOL, "T265 command reply length or ID mismatch");
        if (response.wStatus != SUCCESS) {
            auto code = response.wStatus == DEVICE_BUSY ? T26X_BUSY :
                        response.wStatus == TIMEOUT ? T26X_TIMEOUT : T26X_PROTOCOL;
            throw failure(code, "T265 command " + std::to_string(h.wMessageID) +
                          " returned firmware status " + std::to_string(response.wStatus));
        }
        return reply;
    }
    template<class Request> std::vector<uint8_t> command(const Request& request) {
        return command(&request, sizeof request);
    }
    void simple(uint16_t id) { command(bulk_message_request_header{6, id}); }

    void inspect() {
        libusb_device* device = libusb_get_device(handle.value);
        libusb_config_descriptor* descriptor = nullptr;
        usb_check(libusb_get_active_config_descriptor(device, &descriptor), "Read T265 USB configuration");
        std::unique_ptr<libusb_config_descriptor, decltype(&libusb_free_config_descriptor)>
            owned(descriptor, libusb_free_config_descriptor);
        unsigned endpoints = 0;
        for (int i = 0; i < descriptor->bNumInterfaces; ++i)
            for (int a = 0; a < descriptor->interface[i].num_altsetting; ++a) {
                const auto& iface = descriptor->interface[i].altsetting[a];
                if (iface.bInterfaceNumber != 0 || iface.bAlternateSetting != 0) continue;
                for (int e = 0; e < iface.bNumEndpoints; ++e) {
                    const auto& ep = iface.endpoint[e];
                    const auto type = ep.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK;
                    if (ep.bEndpointAddress == command_out && type == LIBUSB_TRANSFER_TYPE_BULK) endpoints |= 1;
                    if (ep.bEndpointAddress == command_in && type == LIBUSB_TRANSFER_TYPE_BULK) endpoints |= 2;
                    if (ep.bEndpointAddress == video_in && type == LIBUSB_TRANSFER_TYPE_BULK) endpoints |= 4;
                    if (ep.bEndpointAddress == pose_in && type == LIBUSB_TRANSFER_TYPE_INTERRUPT) endpoints |= 8;
                }
            }
        require(endpoints == 15, T26X_UNSUPPORTED, "Device does not have the T265 USB endpoints");
        bulk_message_request_set_low_power_mode power{};
        power.header = {sizeof power, DEV_SET_LOW_POWER_MODE};
        command(power);
        auto bytes = command(bulk_message_request_get_device_info{{6, DEV_GET_DEVICE_INFO}});
        require(bytes.size() >= sizeof(bulk_message_response_get_device_info), T26X_PROTOCOL, "Truncated T265 device information");
        const auto response = read<bulk_message_response_get_device_info>(bytes.data(), bytes.size());
        const auto& data = response.message;
        require((data.bStatus & 15) != 1 && data.dwStatusCode != FW_STATUS_CODE_NO_CALIBRATION_DATA,
                T26X_FIRMWARE, "T265 reports an error or missing factory calibration");
        // Upstream serial is the first six EEPROM bytes interpreted in display order.
        uint8_t serial[8]; std::memcpy(serial, &data.llSerialNumber, sizeof serial);
        uint64_t value = 0;
        for (int i = 0; i < 6; ++i) value = (value << 8) | serial[i];
        std::snprintf(info.serial, sizeof info.serial, "%llX", static_cast<unsigned long long>(value));
        std::snprintf(info.firmware_version, sizeof info.firmware_version, "%u.%u.%u.%u",
                      data.bFWVersionMajor, data.bFWVersionMinor, data.bFWVersionPatch, data.dwFWVersionBuild);
        const int speed = libusb_get_device_speed(device);
        info.usb_speed_mbps = speed >= LIBUSB_SPEED_SUPER ? 5000 : speed == LIBUSB_SPEED_HIGH ? 480 : 12;
        read_profiles();
    }

    void read_profiles() {
        auto bytes = command(bulk_message_request_get_supported_raw_streams{{6, DEV_GET_SUPPORTED_RAW_STREAMS}});
        require(bytes.size() >= 12, T26X_PROTOCOL, "Truncated T265 supported streams");
        const auto count = read<uint16_t>(bytes.data(), bytes.size(), 8);
        // Firmware 0.2.0.951 can pad this reply beyond the advertised entries.
        if (bytes.size() < 12 + size_t(count) * sizeof(supported_raw_stream_libtm_message))
            throw failure(T26X_PROTOCOL, "Invalid T265 supported stream count: " + std::to_string(count) +
                          " profiles, " + std::to_string(bytes.size()) + " reply bytes");
        profiles.clear();
        unsigned selected = 0;
        for (unsigned i = 0; i < count; ++i) {
            auto p = read<supported_raw_stream_libtm_message>(bytes.data(), bytes.size(), 12 + i * 12);
            const int type = p.bSensorID & 31, index = p.bSensorID >> 5;
            unsigned bit = 0;
            if (type == Fisheye && index < 2 && p.bPixelFormat == Y8 &&
                p.wWidth == 848 && p.wHeight == 800 && p.wFramesPerSecond == 30) bit = 1u << index;
            else if (type == Gyro && index == 0 && p.wFramesPerSecond == 200) bit = 4;
            else if (type == Accelerometer && index == 0 && p.wFramesPerSecond == 62) bit = 8;
            if (!bit || (selected & bit)) continue;
            selected |= bit;
            p.bOutputMode = 0;
            if (type == Fisheye) {
                require(p.wStride >= p.wWidth, T26X_PROTOCOL, "Invalid fisheye stride");
                cameras[index] = p;
            }
            profiles.push_back(p);
        }
        require(selected == 15, T26X_UNSUPPORTED, "T265 is missing the required stereo/IMU stream profiles");
    }

    void boot(libusb_device* device, const char* firmware_path) {
        std::vector<unsigned char> file;
        const unsigned char* firmware = nullptr;
        size_t size = 0;
        if (firmware_path && *firmware_path) {
            std::ifstream input(firmware_path, std::ios::binary | std::ios::ate);
            require(bool(input), T26X_FIRMWARE, "Cannot open T265 firmware file");
            const auto length = input.tellg();
            require(length > 0 && length <= 32 * 1024 * 1024, T26X_FIRMWARE, "Invalid T265 firmware file size");
            file.resize(static_cast<size_t>(length));
            input.seekg(0);
            require(bool(input.read(reinterpret_cast<char*>(file.data()), length)), T26X_FIRMWARE, "Cannot read T265 firmware file");
            firmware = file.data(); size = file.size();
        } else {
#if LIBT26X_HAVE_EMBEDDED_FIRMWARE
            firmware = t26x_firmware_start;
            size = reinterpret_cast<uintptr_t>(t26x_firmware_end) - reinterpret_cast<uintptr_t>(t26x_firmware_start);
#endif
            require(firmware && size, T26X_FIRMWARE, "No embedded firmware; provide open_options.firmware_path");
        }
        usb_handle boot_handle;
        boot_handle.open(device);
        int transferred = 0;
        usb_check(libusb_bulk_transfer(boot_handle.value, 0x01, const_cast<unsigned char*>(firmware),
                  static_cast<int>(size), &transferred, 10000), "Boot T265 firmware into RAM");
        require(size_t(transferred) == size, T26X_FIRMWARE, "Short T265 firmware upload");
    }

    void open(const t26x_open_options& options) {
        usb_check(libusb_init(&context), "Initialize libusb");
        const auto deadline = Clock::now() + std::chrono::milliseconds(options.boot_timeout_ms ? options.boot_timeout_ms : 15000);
        std::set<std::string> booted;
        bool waiting = false;
        t26x_error candidate_error{};
        do {
            usb_list list(context);
            for (size_t i = 0; list.devices[i]; ++i) {
                libusb_device_descriptor d{};
                if (libusb_get_device_descriptor(list.devices[i], &d) != 0 || d.idVendor != 0x8087 || d.idProduct != 0x0b37) continue;
                try {
                    handle.open(list.devices[i]);
                    inspect();
                    if (!options.serial || !*options.serial || std::string(options.serial) == info.serial) return;
                    handle.close();
                } catch (const failure& e) {
                    set_error(&candidate_error, e.code, e.what()); handle.close();
                }
            }
            for (size_t i = 0; list.devices[i]; ++i) {
                libusb_device_descriptor d{};
                if (libusb_get_device_descriptor(list.devices[i], &d) != 0 || d.idVendor != 0x03e7 || d.idProduct != 0x2150) continue;
                if (!booted.insert(location(list.devices[i])).second) continue;
                boot(list.devices[i], options.firmware_path);
                waiting = true;
            }
            if (!waiting) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } while (Clock::now() < deadline);
        if (candidate_error.code != T26X_OK) throw failure(candidate_error.code, candidate_error.message);
        throw failure(waiting ? T26X_TIMEOUT : T26X_NOT_FOUND,
                      waiting ? "T265 did not re-enumerate with the requested serial before the boot timeout" : "No matching T265 found");
    }

    void fail_stream(t26x_status code, const char* message) noexcept {
        t26x_error error{};
        {
            std::lock_guard<std::mutex> lock(pose_mutex);
            if (stream_error.code != T26X_OK || stopping) return;
            set_error(&stream_error, code, message); error = stream_error;
            running = false;
        }
        pose_ready.notify_all();
        if (config.on_error) {
            callback_scope scope(this);
            try { config.on_error(&error, config.user); } catch (...) {}
        }
    }

    void receive(const uint8_t* data, size_t size, bool video) {
        const auto h = header(data, size);
        if (!video && h.wMessageID == DEV_GET_POSE) {
            auto pose = decode_pose(data, size, 0);
            {
                std::lock_guard<std::mutex> lock(pose_mutex);
                pose.frame_number = ++generation; latest = pose; have_pose = true;
            }
            pose_ready.notify_all();
            if (config.on_pose) { callback_scope scope(this); config.on_pose(&pose, config.user); }
        } else if (video && h.wMessageID == DEV_SAMPLE) {
            require(size >= sizeof(bulk_message_video_stream), T26X_PROTOCOL, "Truncated T265 video packet");
            const uint8_t sensor = data[6];
            require((sensor & 31) == Fisheye && (sensor >> 5) < 2,
                    T26X_PROTOCOL, "Unexpected T265 video sensor");
            // Firmware reports dropped frames with a zero pixel payload.
            // As in the upstream driver, discard these without stopping pose.
            if (read<uint32_t>(data, size, 40) == 0) return;
            auto frame = decode_video(data, size, cameras[sensor >> 5]);
            if (config.on_fisheye) { callback_scope scope(this); config.on_fisheye(&frame, config.user); }
        } else if (h.wMessageID == DEV_ERROR || h.wMessageID == DEV_STATUS || h.wMessageID == SLAM_ERROR) {
            const auto status = read<uint16_t>(data, size, 6);
            if (h.wMessageID == SLAM_ERROR && status != SLAM_ERROR_CODE_OTHER) return;
            if (status == SUCCESS || status == TEMPERATURE_WARNING ||
                (h.wMessageID == DEV_STATUS && status == DEVICE_STOPPED)) return;
            throw failure(T26X_IO, "T265 stream status " + std::to_string(status));
        }
        // IMU output and relocalization notifications are not exposed by this SDK.
    }

    void reader(bool video) noexcept {
        try {
            // Capacities and transfer lengths are multiples of USB max-packet size.
            const size_t capacity = video ? 2 * 1024 * 1024 : 4096;
            const size_t max_message = video ? 1024 * 1024 : 1024;
            std::vector<uint8_t> buffer(capacity);
            size_t used = 0;
            while (running) {
                int received = 0;
                const int request = static_cast<int>(((capacity - used) / 1024) * 1024);
                require(request > 0, T26X_PROTOCOL, "T265 stream buffer exhausted");
                const int status = video ?
                    libusb_bulk_transfer(handle.value, video_in, buffer.data() + used, request, &received, 100) :
                    libusb_interrupt_transfer(handle.value, pose_in, buffer.data() + used, request, &received, 100);
                if (!running) break;
                if (status != LIBUSB_ERROR_TIMEOUT) usb_check(status, "Read T265 stream");
                used += static_cast<size_t>(received);
                size_t offset = 0;
                while (used - offset >= sizeof(interrupt_message_header)) {
                    const auto h = read<interrupt_message_header>(buffer.data() + offset, used - offset);
                    require(h.dwLength >= sizeof(h) && h.dwLength <= max_message,
                            T26X_PROTOCOL, "Invalid T265 stream packet length");
                    if (h.dwLength > used - offset) break;
                    if (running && !stopping) receive(buffer.data() + offset, h.dwLength, video);
                    offset += h.dwLength;
                }
                if (offset) { used -= offset; if (used) std::memmove(buffer.data(), buffer.data() + offset, used); }
            }
        } catch (const failure& e) { fail_stream(e.code, e.what()); }
        catch (const std::exception& e) { fail_stream(T26X_PROTOCOL, e.what()); }
        catch (...) { fail_stream(T26X_INTERNAL, "Exception from streaming callback"); }
    }

    void start(const t26x_stream_config& requested) {
        require(!reset_done, T26X_INVALID_STATE, "Close and reopen the camera after reset");
        require(!started && !pose_thread.joinable(), T26X_INVALID_STATE, "Stop the previous stream before starting");
        require(!requested.enable_fisheye || info.usb_speed_mbps >= 5000, T26X_UNSUPPORTED,
                "Fisheye streaming requires a USB3 connection; pose works over USB2");
        std::vector<uint8_t> request(8 + profiles.size() * sizeof(supported_raw_stream_libtm_message));
        const bulk_message_request_header h{static_cast<uint32_t>(request.size()), DEV_RAW_STREAMS_CONTROL};
        std::memcpy(request.data(), &h, sizeof h);
        const uint16_t count = static_cast<uint16_t>(profiles.size());
        std::memcpy(request.data() + 6, &count, sizeof count);
        for (size_t i = 0; i < profiles.size(); ++i) {
            auto p = profiles[i];
            p.bOutputMode = requested.enable_fisheye && (p.bSensorID & 31) == Fisheye;
            std::memcpy(request.data() + 8 + i * sizeof p, &p, sizeof p);
        }
        command(request.data(), request.size());
        bulk_message_request_6dof_control slam{};
        slam.header = {sizeof slam, SLAM_6DOF_CONTROL}; slam.bEnable = 1;
        slam.bMode = SIXDOF_MODE_ENABLE_MAPPING | SIXDOF_MODE_ENABLE_RELOCALIZATION;
        command(slam);
        config = requested;
        {
            std::lock_guard<std::mutex> lock(pose_mutex);
            have_pose = false; stream_error = {}; latest = {}; generation = 0;
        }
        stopping = false; running = true;
        try {
            pose_thread = std::thread(&t26x_device::reader, this, false);
            if (config.enable_fisheye) video_thread = std::thread(&t26x_device::reader, this, true);
            // Mark before the command: a lost response still requires a stop attempt.
            started = true;
            simple(DEV_START);
        } catch (...) {
            try { stop(); } catch (...) {}
            throw;
        }
    }

    void stop() {
        stopping = true;
        std::exception_ptr pending;
        if (started && !reset_done) {
            try { simple(DEV_STOP); } catch (...) { pending = std::current_exception(); }
        }
        { std::lock_guard<std::mutex> lock(pose_mutex); running = false; }
        pose_ready.notify_all();
        if (pose_thread.joinable()) pose_thread.join();
        if (video_thread.joinable()) video_thread.join();
        started = false; stopping = false;
        if (pending) std::rethrow_exception(pending);
    }
};

namespace {
void check_device(t26x_device* device) { require(device != nullptr, T26X_INVALID_ARGUMENT, "Device is null"); }
void lifecycle(t26x_device* device) {
    check_device(device);
    require(callback_device != device, T26X_INVALID_STATE, "Do not call camera lifecycle methods from a callback");
}
} // namespace

extern "C" {
const char* t26x_version(void) { return LIBT26X_VERSION; }
const char* t26x_status_string(t26x_status status) {
    switch (status) {
    case T26X_OK: return "Success"; case T26X_INVALID_ARGUMENT: return "Invalid argument";
    case T26X_NOT_FOUND: return "Device not found"; case T26X_ACCESS: return "Access denied";
    case T26X_BUSY: return "Device busy"; case T26X_IO: return "USB/device error";
    case T26X_TIMEOUT: return "Timeout"; case T26X_UNSUPPORTED: return "Unsupported configuration";
    case T26X_PROTOCOL: return "Protocol error"; case T26X_INVALID_STATE: return "Invalid state";
    case T26X_FIRMWARE: return "Firmware error"; case T26X_INTERNAL: return "Internal error";
    default: return "Unknown error";
    }
}
void t26x_open_options_init(t26x_open_options* options) { if (options) { *options = {}; options->boot_timeout_ms = 15000; } }
void t26x_stream_config_init(t26x_stream_config* config) { if (config) *config = {}; }
t26x_status t26x_open(const t26x_open_options* options, t26x_device** output, t26x_error* error) {
    if (output) *output = nullptr;
    return protect(error, [&] {
        require(output != nullptr, T26X_INVALID_ARGUMENT, "Output device pointer is null");
        t26x_open_options defaults{}; t26x_open_options_init(&defaults);
        std::unique_ptr<t26x_device> device(new t26x_device);
        device->open(options ? *options : defaults); *output = device.release();
    });
}
t26x_status t26x_start(t26x_device* device, const t26x_stream_config* config, t26x_error* error) {
    return protect(error, [&] { lifecycle(device); t26x_stream_config defaults{}; device->start(config ? *config : defaults); });
}
t26x_status t26x_stop(t26x_device* device, t26x_error* error) {
    return protect(error, [&] { lifecycle(device); device->stop(); });
}
t26x_status t26x_reset(t26x_device* device, t26x_error* error) {
    return protect(error, [&] {
        lifecycle(device);
        require(!device->reset_done, T26X_INVALID_STATE, "Camera has already been reset");
        try { device->stop(); } catch (...) { /* Reset also recovers a failed stream. */ }
        const int result = libusb_control_transfer(device->handle.value, 0x40, 0x10, 0, 0, nullptr, 0, command_timeout);
        device->reset_done = true;
        { std::lock_guard<std::mutex> lock(device->pose_mutex); device->have_pose = false; }
        // T265 can disconnect before completing the control status stage. Linux
        // then reports IO (observed on .951), PIPE, or NO_DEVICE. Only accept
        // ambiguous errors when enumeration confirms the old device is gone.
        if (result == LIBUSB_ERROR_IO || result == LIBUSB_ERROR_PIPE || result == LIBUSB_ERROR_TIMEOUT) {
            const auto original = libusb_get_device(device->handle.value);
            const auto deadline = Clock::now() + std::chrono::seconds(1);
            do {
                usb_list list(device->context);
                bool present = false;
                for (size_t i = 0; list.devices[i]; ++i) if (list.devices[i] == original) present = true;
                if (!present) return;
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            } while (Clock::now() < deadline);
        }
        if (result != LIBUSB_ERROR_NO_DEVICE) usb_check(result, "Reset T265");
    });
}
t26x_status t26x_close(t26x_device* device, t26x_error* error) {
    return protect(error, [&] {
        if (!device) return;
        lifecycle(device);
        std::unique_ptr<t26x_device> owned(device);
        device->stop();
    });
}
t26x_status t26x_get_device_info(t26x_device* device, t26x_device_info* info, t26x_error* error) {
    return protect(error, [&] { check_device(device); require(info, T26X_INVALID_ARGUMENT, "Info output is null"); *info = device->info; });
}
t26x_status t26x_get_pose(t26x_device* device, t26x_pose* pose, t26x_error* error) {
    return protect(error, [&] {
        check_device(device); require(pose, T26X_INVALID_ARGUMENT, "Pose output is null");
        std::lock_guard<std::mutex> lock(device->pose_mutex);
        if (device->stream_error.code != T26X_OK) throw failure(device->stream_error.code, device->stream_error.message);
        require(device->have_pose, T26X_INVALID_STATE, "No pose has arrived yet"); *pose = device->latest;
    });
}
t26x_status t26x_wait_for_pose(t26x_device* device, t26x_pose* pose, uint32_t timeout_ms, t26x_error* error) {
    return protect(error, [&] {
        check_device(device); require(pose, T26X_INVALID_ARGUMENT, "Pose output is null");
        require(callback_device != device, T26X_INVALID_STATE, "Do not wait for poses from a camera callback");
        std::unique_lock<std::mutex> lock(device->pose_mutex);
        if (device->stream_error.code != T26X_OK) throw failure(device->stream_error.code, device->stream_error.message);
        require(device->running, T26X_INVALID_STATE, "Camera is not streaming");
        const auto generation = device->generation;
        require(device->pose_ready.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&] {
                    return device->generation != generation || !device->running;
                }), T26X_TIMEOUT, "Timed out waiting for a new pose");
        if (device->stream_error.code != T26X_OK) throw failure(device->stream_error.code, device->stream_error.message);
        require(device->generation != generation, T26X_INVALID_STATE, "Camera stopped while waiting for a pose");
        *pose = device->latest;
    });
}
t26x_status t26x_confidence(t26x_device* device, uint32_t* tracker, uint32_t* mapper, t26x_error* error) {
    if (!tracker || !mapper) return set_error(error, T26X_INVALID_ARGUMENT, "Confidence output is null");
    t26x_pose pose{};
    const auto result = t26x_get_pose(device, &pose, error);
    if (result == T26X_OK) { *tracker = pose.tracker_confidence; *mapper = pose.mapper_confidence; }
    return result;
}
int t26x_is_streaming(t26x_device* device) { return device && device->running; }
} // extern C

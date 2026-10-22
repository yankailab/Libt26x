// SPDX-License-Identifier: Apache-2.0
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.
// Copyright (c) 2026 Kai
// Adapted from librealsense v2.53.1 src/tm2: bounded, alignment-safe decoding.
#pragma once
#include <t26x/t26x.h>
#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <type_traits>
#pragma GCC diagnostic push
#include "vendor/t265-messages.h"
#pragma GCC diagnostic pop

#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "Libt26x currently requires a little-endian target"
#endif

namespace t26x_detail {
static_assert(sizeof(float_t) == 4, "The T265 wire protocol uses 32-bit floats");
static_assert(sizeof(t265::pose_data) == 96, "Unexpected pose wire layout");
static_assert(sizeof(t265::supported_raw_stream_libtm_message) == 12, "Unexpected stream layout");
static_assert(sizeof(t265::bulk_message_video_stream) == 44, "Unexpected video layout");

template<class T> T read(const uint8_t* data, size_t length, size_t offset = 0) {
    if (offset > length || sizeof(T) > length - offset)
        throw std::runtime_error("Truncated T265 USB message");
    T value;
    std::memcpy(&value, data + offset, sizeof value);
    return value;
}

inline t265::interrupt_message_header header(const uint8_t* data, size_t size) {
    auto h = read<t265::interrupt_message_header>(data, size);
    if (h.dwLength != size) throw std::runtime_error("Invalid T265 message length");
    return h;
}

inline t26x_pose decode_pose(const uint8_t* data, size_t size, uint64_t sequence) {
    if (header(data, size).wMessageID != t265::DEV_GET_POSE ||
        size != sizeof(t265::interrupt_message_get_pose))
        throw std::runtime_error("Invalid T265 pose message");
    const auto msg = read<t265::interrupt_message_get_pose>(data, size);
    if (msg.bIndex != 0) throw std::runtime_error("Unsupported T265 pose index");
    const auto& p = msg.pose;
    t26x_pose result{};
    result.timestamp_ns = p.llNanoseconds;
    result.frame_number = sequence;
    result.translation[0] = p.flX; result.translation[1] = p.flY; result.translation[2] = p.flZ;
    result.rotation[0] = p.flQi; result.rotation[1] = p.flQj;
    result.rotation[2] = p.flQk; result.rotation[3] = p.flQr;
    result.velocity[0] = p.flVx; result.velocity[1] = p.flVy; result.velocity[2] = p.flVz;
    result.acceleration[0] = p.flAx; result.acceleration[1] = p.flAy; result.acceleration[2] = p.flAz;
    result.angular_velocity[0] = p.flVAX; result.angular_velocity[1] = p.flVAY; result.angular_velocity[2] = p.flVAZ;
    result.angular_acceleration[0] = p.flAAX; result.angular_acceleration[1] = p.flAAY; result.angular_acceleration[2] = p.flAAZ;
    result.tracker_confidence = p.dwTrackerConfidence;
    result.mapper_confidence = p.dwMapperConfidence & 3;
    if (result.tracker_confidence > 3) throw std::runtime_error("Invalid T265 tracking confidence");
    return result;
}

inline t26x_fisheye_frame decode_video(const uint8_t* data, size_t size,
                                      const t265::supported_raw_stream_libtm_message& profile) {
    if (header(data, size).wMessageID != t265::DEV_SAMPLE)
        throw std::runtime_error("Invalid T265 video message ID");
    const auto msg = read<t265::bulk_message_video_stream>(data, size);
    if (msg.rawStreamHeader.bSensorID != profile.bSensorID ||
        (profile.bSensorID & 31) != t265::Fisheye || (profile.bSensorID >> 5) > 1 ||
        profile.bPixelFormat != t265::Y8 || profile.wStride < profile.wWidth ||
        !profile.wWidth || !profile.wHeight || msg.metadata.dwMetadataLength != 8)
        throw std::runtime_error("Invalid T265 fisheye metadata");
    const size_t pixels = size_t(profile.wStride) * profile.wHeight;
    if (msg.metadata.dwFrameLength != pixels || size - sizeof(msg) != pixels)
        throw std::runtime_error("Invalid T265 fisheye payload length");
    t26x_fisheye_frame result{};
    result.timestamp_ns = msg.rawStreamHeader.llNanoseconds;
    result.frame_number = msg.rawStreamHeader.dwFrameId;
    result.camera_index = (profile.bSensorID >> 5) + 1;
    result.width = profile.wWidth; result.height = profile.wHeight; result.stride = profile.wStride;
    result.exposure_us = msg.metadata.dwExposuretime; result.gain = msg.metadata.fGain;
    result.data = data + sizeof(msg); result.data_size = pixels;
    return result;
}
} // namespace t26x_detail

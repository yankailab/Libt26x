// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Kai
#include "protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

namespace {
unsigned checks = 0;

void require(bool condition, const std::string &message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

template<class Action> void rejects(Action action, const std::string &message) {
    bool rejected = false;
    try { action(); } catch (const std::runtime_error &) { rejected = true; }
    require(rejected, message);
}

// Explicit wire offsets and IEEE-754 bit patterns keep these fixtures
// independent of the compiler's packed-structure layout and the decoder.
void put32(std::vector<uint8_t> &bytes, size_t offset, uint32_t value) {
    for (unsigned n = 0; n < 4; ++n) bytes.at(offset + n) = uint8_t(value >> (n * 8));
}

void put64(std::vector<uint8_t> &bytes, size_t offset, uint64_t value) {
    for (unsigned n = 0; n < 8; ++n) bytes.at(offset + n) = uint8_t(value >> (n * 8));
}

std::vector<uint8_t> pose_packet() {
    std::vector<uint8_t> bytes(104, 0);
    put32(bytes, 0, 104);
    bytes[4] = 0x15;                       // DEV_GET_POSE, index 0
    put32(bytes, 8, 0x3f800000);            // translation: 1, -2, 0.5
    put32(bytes, 12, 0xc0000000);
    put32(bytes, 16, 0x3f000000);
    put32(bytes, 20, 0x3e000000);           // quaternion: 0.125, -0.25, 0.5, 0.75
    put32(bytes, 24, 0xbe800000);
    put32(bytes, 28, 0x3f000000);
    put32(bytes, 32, 0x3f400000);
    put32(bytes, 36, 0x40000000);           // velocity: 2, 3, 4
    put32(bytes, 40, 0x40400000);
    put32(bytes, 44, 0x40800000);
    put32(bytes, 48, 0x40a00000);           // angular velocity: 5, 6, 7
    put32(bytes, 52, 0x40c00000);
    put32(bytes, 56, 0x40e00000);
    put32(bytes, 60, 0x41000000);           // acceleration: 8, 9, 10
    put32(bytes, 64, 0x41100000);
    put32(bytes, 68, 0x41200000);
    put32(bytes, 72, 0x41300000);           // angular acceleration: 11, 12, 13
    put32(bytes, 76, 0x41400000);
    put32(bytes, 80, 0x41500000);
    put64(bytes, 84, UINT64_C(0xfedcba9876543210));
    put32(bytes, 92, 3);                   // tracker confidence
    put32(bytes, 96, 0xa5000002);          // mapper confidence, reserved upper bits
    put32(bytes, 100, 4);                  // 6DoF tracker state
    return bytes;
}

void test_pose() {
    const auto bytes = pose_packet();
    const t26x_pose pose = t26x_detail::decode_pose(bytes.data(), bytes.size(), 1234);
    require(pose.timestamp_ns == UINT64_C(0xfedcba9876543210), "pose timestamp must retain all 64 bits");
    require(pose.frame_number == 1234, "pose sequence supplied by reader");
    const float translation[] = {1.0f, -2.0f, 0.5f};
    const float rotation[] = {0.125f, -0.25f, 0.5f, 0.75f};
    for (unsigned i = 0; i < 3; ++i) {
        require(pose.translation[i] == translation[i], "pose translation component");
        require(pose.velocity[i] == 2.0f + i, "pose velocity component");
        require(pose.angular_velocity[i] == 5.0f + i, "pose angular velocity component");
        require(pose.acceleration[i] == 8.0f + i, "pose acceleration component");
        require(pose.angular_acceleration[i] == 11.0f + i, "pose angular acceleration component");
    }
    for (unsigned i = 0; i < 4; ++i)
        require(pose.rotation[i] == rotation[i], "pose quaternion XYZW component");
    require(pose.tracker_confidence == 3 && pose.mapper_confidence == 2,
            "confidence values and reserved mapper flags");

    std::vector<uint8_t> unaligned(bytes.size() + 1, 0);
    std::copy(bytes.begin(), bytes.end(), unaligned.begin() + 1);
    const auto aligned_result = t26x_detail::decode_pose(unaligned.data() + 1, bytes.size(), 9);
    require(aligned_result.timestamp_ns == pose.timestamp_ns && aligned_result.rotation[1] == -0.25f,
            "unaligned USB pose buffer must decode safely");

    for (size_t size = 0; size < bytes.size(); ++size) {
        auto truncated = bytes;
        if (size >= 4) put32(truncated, 0, static_cast<uint32_t>(size));
        rejects([&] { t26x_detail::decode_pose(truncated.data(), size, 0); },
                "reject truncated pose at byte " + std::to_string(size));
    }
    auto changed = bytes;
    put32(changed, 0, 105);
    rejects([&] { t26x_detail::decode_pose(changed.data(), changed.size(), 0); }, "reject pose declared length mismatch");
    changed = bytes; changed.push_back(0); put32(changed, 0, 105);
    rejects([&] { t26x_detail::decode_pose(changed.data(), changed.size(), 0); }, "reject oversized pose packet");
    changed = bytes; changed[4] = 0x11;
    rejects([&] { t26x_detail::decode_pose(changed.data(), changed.size(), 0); }, "reject non-pose message ID");
    changed = bytes; changed[6] = 1;
    rejects([&] { t26x_detail::decode_pose(changed.data(), changed.size(), 0); }, "reject unknown pose index");
    for (uint32_t confidence = 0; confidence < 4; ++confidence) {
        changed = bytes; put32(changed, 92, confidence); put32(changed, 96, 0xfffffffcu | confidence);
        const auto result = t26x_detail::decode_pose(changed.data(), changed.size(), 0);
        require(result.tracker_confidence == confidence && result.mapper_confidence == confidence,
                "all supported confidence levels decode");
    }
    for (uint32_t invalid : {uint32_t(4), UINT32_MAX}) {
        changed = bytes; put32(changed, 92, invalid);
        rejects([&] { t26x_detail::decode_pose(changed.data(), changed.size(), 0); }, "reject invalid tracker confidence");
    }
}

std::vector<uint8_t> video_packet() {
    std::vector<uint8_t> bytes(54, 0);
    put32(bytes, 0, 54);
    bytes[4] = 0x11;                       // DEV_SAMPLE
    bytes[6] = 0x03;                       // first fisheye camera
    put64(bytes, 8, UINT64_C(0x0123456789abcdef));
    put64(bytes, 16, UINT64_C(0x0123456789abcfff));
    put32(bytes, 24, 0x89abcdef);           // frame ID
    put32(bytes, 28, 8);                    // exposure/gain metadata size
    put32(bytes, 32, 7500);                 // exposure in microseconds
    put32(bytes, 36, 0x40200000);           // gain = 2.5
    put32(bytes, 40, 10);                   // 5-byte stride * 2 rows
    for (size_t i = 44; i < bytes.size(); ++i) bytes[i] = uint8_t(0x80 + i - 44);
    return bytes;
}

void test_video() {
    t265::supported_raw_stream_libtm_message profile{};
    profile.bSensorID = 0x03;
    profile.bPixelFormat = 9;               // Y8
    profile.wWidth = 3;
    profile.wHeight = 2;
    profile.wStride = 5;                   // deliberately includes row padding
    profile.wFramesPerSecond = 30;
    const auto bytes = video_packet();
    const auto frame = t26x_detail::decode_video(bytes.data(), bytes.size(), profile);
    require(frame.timestamp_ns == UINT64_C(0x0123456789abcdef), "video integration timestamp");
    require(frame.frame_number == 0x89abcdef && frame.camera_index == 1, "video frame ID and left camera");
    require(frame.width == 3 && frame.height == 2 && frame.stride == 5, "video profile dimensions retain padding");
    require(frame.exposure_us == 7500 && frame.gain == 2.5f, "video exposure and gain");
    require(frame.data == bytes.data() + 44 && frame.data_size == 10, "video references exact payload without copying");
    require(frame.data[0] == 0x80 && frame.data[9] == 0x89, "video pixel and padding bytes preserved");

    auto changed = bytes;
    auto altered_profile = profile;
    changed[6] = 0x23; altered_profile.bSensorID = 0x23;
    require(t26x_detail::decode_video(changed.data(), changed.size(), altered_profile).camera_index == 2,
            "right fisheye camera index");
    std::vector<uint8_t> unaligned(bytes.size() + 1, 0);
    std::copy(bytes.begin(), bytes.end(), unaligned.begin() + 1);
    require(t26x_detail::decode_video(unaligned.data() + 1, bytes.size(), profile).gain == 2.5f,
            "unaligned video metadata must decode safely");

    for (size_t size = 0; size < bytes.size(); ++size) {
        auto truncated = bytes;
        if (size >= 4) put32(truncated, 0, static_cast<uint32_t>(size));
        rejects([&] { t26x_detail::decode_video(truncated.data(), size, profile); },
                "reject truncated video at byte " + std::to_string(size));
    }
    changed = bytes; put32(changed, 0, 53);
    rejects([&] { t26x_detail::decode_video(changed.data(), changed.size(), profile); }, "reject video declared length mismatch");
    changed = bytes; changed.push_back(0); put32(changed, 0, 55);
    rejects([&] { t26x_detail::decode_video(changed.data(), changed.size(), profile); }, "reject oversized video payload");
    changed = bytes; changed[4] = 0x15;
    rejects([&] { t26x_detail::decode_video(changed.data(), changed.size(), profile); }, "reject video message ID mismatch");
    changed = bytes; changed[6] = 0x23;
    rejects([&] { t26x_detail::decode_video(changed.data(), changed.size(), profile); }, "reject sensor/profile mismatch");
    for (uint8_t sensor : {uint8_t(4), uint8_t(0x43)}) {
        changed = bytes; changed[6] = sensor;
        altered_profile = profile; altered_profile.bSensorID = sensor;
        rejects([&] { t26x_detail::decode_video(changed.data(), changed.size(), altered_profile); },
                "reject non-fisheye or third camera sensor");
    }
    for (uint32_t length : {uint32_t(0), uint32_t(7), uint32_t(9), UINT32_MAX}) {
        changed = bytes; put32(changed, 28, length);
        rejects([&] { t26x_detail::decode_video(changed.data(), changed.size(), profile); }, "reject unexpected video metadata length");
    }
    for (uint32_t length : {uint32_t(0), uint32_t(9), uint32_t(11), UINT32_MAX}) {
        changed = bytes; put32(changed, 40, length);
        rejects([&] { t26x_detail::decode_video(changed.data(), changed.size(), profile); }, "reject mismatched video frame length");
    }
    altered_profile = profile; altered_profile.bPixelFormat = 0;
    rejects([&] { t26x_detail::decode_video(bytes.data(), bytes.size(), altered_profile); }, "reject non-Y8 profile");
    altered_profile = profile; altered_profile.wStride = 2;
    rejects([&] { t26x_detail::decode_video(bytes.data(), bytes.size(), altered_profile); }, "reject stride smaller than width");
    altered_profile = profile; altered_profile.wWidth = 0;
    rejects([&] { t26x_detail::decode_video(bytes.data(), bytes.size(), altered_profile); }, "reject empty width");
    altered_profile = profile; altered_profile.wHeight = 0;
    rejects([&] { t26x_detail::decode_video(bytes.data(), bytes.size(), altered_profile); }, "reject empty height");
}
} // namespace

int main() {
    try {
        test_pose();
        test_video();
        const uint8_t bytes[] = {0x11, 0x22, 0x33, 0x44};
        require(t26x_detail::read<uint32_t>(bytes, sizeof bytes) == 0x44332211, "little-endian bounded read");
        rejects([&] { t26x_detail::read<uint32_t>(bytes, sizeof bytes, 1); }, "reject read past buffer end");
        rejects([&] { t26x_detail::read<uint8_t>(bytes, sizeof bytes, std::numeric_limits<size_t>::max()); },
                "reject offset overflow without accessing memory");
        rejects([&] { t26x_detail::read<uint8_t>(nullptr, 0); }, "reject empty buffer before reading");
        std::printf("Protocol: %u checks passed.\n", checks);
        return EXIT_SUCCESS;
    } catch (const std::exception &error) {
        std::fprintf(stderr, "Protocol check failed: %s\n", error.what());
        return EXIT_FAILURE;
    }
}

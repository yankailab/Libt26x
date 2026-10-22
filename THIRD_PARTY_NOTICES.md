# Third-party notices and source provenance

Libt26x's original public interfaces, examples, build configuration, and
documentation use the [MIT license](LICENSE). The Intel-derived implementation
identified below remains subject to Apache License 2.0. The MIT license does not
replace third-party license terms or cover the camera firmware.

## Intel RealSense SDK source

- Project: [librealsense](https://github.com/realsenseai/librealsense)
- Release: [v2.53.1](https://github.com/realsenseai/librealsense/releases/tag/v2.53.1)
- Commit: `23b0904ba126e87327bc2908c1a5f79342eae867`
- Copyright: Copyright(c) 2017 Intel Corporation. All Rights Reserved.
- License: [Apache License, Version 2.0](LICENSES/Apache-2.0.txt)

`src/vendor/t265-messages.h` is an unmodified copy of
[`src/tm2/t265-messages.h`](https://github.com/realsenseai/librealsense/blob/23b0904ba126e87327bc2908c1a5f79342eae867/src/tm2/t265-messages.h).
Its SHA-256 is
`bf24ad01946ff6551c8b03d11fd66d518f67683c1ee9eea82a8ebf7524da973f`.

`src/t26x.cpp` adapts the T265 command sequences, firmware boot protocol, and
stream handling from
[`src/tm2/tm-device.cpp`](https://github.com/realsenseai/librealsense/blob/23b0904ba126e87327bc2908c1a5f79342eae867/src/tm2/tm-device.cpp)
and
[`src/tm2/tm-boot.h`](https://github.com/realsenseai/librealsense/blob/23b0904ba126e87327bc2908c1a5f79342eae867/src/tm2/tm-boot.h).
It replaces the full SDK infrastructure with Libt26x's C interface and direct
libusb transport. These modifications are marked in the source file.

`src/protocol.hpp` also adapts the upstream protocol handling. It adds bounds
and alignment checks for incoming messages and compile-time checks for the
wire layouts. It is marked as modified and licensed under Apache License 2.0.

Apache License 2.0 requires preservation of applicable copyright and attribution
notices, a copy of the license when distributing the work, and notices identifying
modified files. The upstream `NOTICE` describes other third-party components of
the full SDK; those components are not copied into Libt26x.

## libusb

Libt26x dynamically links the system's `libusb-1.0` library. libusb is separately
licensed under [LGPL 2.1 or later](https://github.com/libusb/libusb/blob/master/COPYING).
No libusb source or binary is vendored in this repository. When distributing
libusb itself with an application, its license and corresponding distribution
requirements apply separately.

## Intel T265 firmware

The firmware is a separate Intel binary, downloaded from the same official
location used by librealsense v2.53.1. It is not included in this source repository
and is not licensed under Libt26x's MIT license or the copied source's Apache
license. No explicit grant to redistribute this firmware was located in the
upstream source license, notices, or accompanying download. For that reason,
Libt26x downloads it at configure time instead of committing a copy.

| Property | Value |
| --- | --- |
| Version | `0.2.0.951` |
| Filename | `target-0.2.0.951.mvcmd` |
| Size | 9,323,648 bytes |
| Upstream SHA-1 | `c3940ccbb0e3045603e4aceaa2d73427f96e24bc` |
| Verified SHA-256 | `0265fd111611908b822cdaf4a3fe5b631c50539b2805d2f364c498aa71c007c0` |

Download:
<https://librealsense.intel.com/Releases/TM2/FW/target/0.2.0.951/target-0.2.0.951.mvcmd>

Version and upstream hash source:
[`common/fw/firmware-version.h`](https://github.com/realsenseai/librealsense/blob/23b0904ba126e87327bc2908c1a5f79342eae867/common/fw/firmware-version.h)
and
[`common/fw/CMakeLists.txt`](https://github.com/realsenseai/librealsense/blob/23b0904ba126e87327bc2908c1a5f79342eae867/common/fw/CMakeLists.txt).
The SHA-256 and byte count above were computed from the official download.

The default build embeds the downloaded firmware in the resulting shared
library. This does not change its licensing. A firmware-free build with a
separately supplied runtime firmware file is available; see [README.md](README.md).

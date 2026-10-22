# Libt26x

A small C/C++ SDK for Intel RealSense T265 tracking cameras on Linux.
Libt26x communicates directly over USB and builds as
`libt26x.so`. It does not require an installed RealSense SDK, UVC patches, DKMS,
OpenCV, or a viewer, and can coexist with a newer librealsense installation.
Only one application may own a camera at a time.

The implementation is based on librealsense **v2.53.1**, commit
`23b0904ba126e87327bc2908c1a5f79342eae867`, the last release containing T265
support. Intel removed the T265 code in
[v2.54.1](https://github.com/realsenseai/librealsense/releases/tag/v2.54.1).
Intel's last *validated* T265 release was v2.50.0; that differs from the last
release containing its driver. Libt26x uses the
[direct USB implementation introduced by PR #5213](https://github.com/realsenseai/librealsense/pull/5213),
which replaced the older `libtm` dependency.

## Scope

| Capability | Support |
| --- | --- |
| Pose | Translation, orientation, velocities, accelerations, tracker and mapper confidence; approximately 200 Hz |
| USB 2 | Pose streaming |
| USB 3 | Pose and both fisheye streams, Y8 grayscale, 848 × 800 at 30 Hz |
| Device control | Open, start, stop, latest pose, wait for pose, confidence, hardware reset, close |
| Firmware boot | Automatic upload of firmware 0.2.0.951 when a camera starts in bootloader mode |
| Platforms | Linux, little-endian x86-64, ARM32, and ARM64 |
| Dependencies | System libusb-1.0, C++ runtime, and POSIX threads |

Pose processing runs on the camera. Libt26x handles USB transport and exposes
the results. Fisheye output is raw grayscale; calibration, image rectification,
mapping import/export, recording containers, and a graphical viewer are outside
this SDK's scope. Both fisheye streams are enabled together. Libt26x requires
USB 3 for fisheye output because the
[upstream driver warns that USB 2 video streaming is unreliable](https://github.com/realsenseai/librealsense/blob/v2.53.1/src/tm2/tm-device.cpp#L620).
It rejects fisheye requests on USB 2 while allowing pose streaming. This is an
SDK support policy, not a claim that USB 2 fisheye transfer is physically
impossible. The negotiated USB speed matters, even with a USB-C socket.

The code and build support the listed architectures; hardware validation is
reported separately below. T261 uses the same tracking-module protocol, but
Libt26x has not been validated with T261 hardware.

## Build and install

Building requires CMake 3.16 or newer, a C++14 compiler, and libusb 1.0.20 or
newer. The public C++ wrapper requires C++11; the C interface is usable from C99.
On Debian/Ubuntu, install the build tools and libusb headers:

```sh
sudo apt install build-essential cmake pkg-config libusb-1.0-0-dev
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
(cd build && ctest --output-on-failure)
sudo cmake --install build
sudo ldconfig
```

The default install prefix is `/usr/local`. CMake's `GNUInstallDirs` controls
the library directory. Installation includes the shared library, C and C++
headers, a CMake package, pkg-config metadata, and license notices.
Use `-DCMAKE_INSTALL_PREFIX=/your/prefix` to select another location.

Build options:

| Option | Default | Purpose |
| --- | --- | --- |
| `LIBT26X_BUILD_EXAMPLES` | `ON` | Build the small pose and capture command-line examples |
| `BUILD_TESTING` | `ON` | Build tests registered with CTest |
| `LIBT26X_HARDWARE_TESTS` | `OFF` | Register the camera lifecycle test with CTest; it resets a connected camera |
| `LIBT26X_DOWNLOAD_FIRMWARE` | `ON` | Download and embed the pinned firmware at configure time |
| `LIBT26X_FIRMWARE_FILE` | Empty | Embed an existing 0.2.0.951 image, verified against the pinned SHA-256 |
| `LIBT26X_INSTALL_UDEV_RULES` | `OFF` | Install rules under `<prefix>/lib/udev/rules.d` |

ARM builds use the same sources and system libusb. For cross-compilation, pass
your normal `CMAKE_TOOLCHAIN_FILE` and provide the target architecture's libusb
headers, library, and pkg-config configuration. No x86-only binaries or SIMD
dependencies are required.

## USB access

Install the supplied udev rules once if your account cannot open the camera:

```sh
sudo install -m 644 packaging/99-libt26x.rules /etc/udev/rules.d/99-libt26x.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb
```

Reconnect the camera after installing the rules. The rules cover both the
Movidius bootloader (`03e7:2150`) and running T265 (`8087:0b37`). They grant
access through the active local session and the `plugdev` group. For a headless
service, ensure its account belongs to the appropriate device-access group;
group changes require a new login or service restart. An existing RealSense
udev rule may already provide access.

Installing udev rules is an explicit administrator action. The default library
installation does not change device permissions.

## Firmware

The T265 loads its firmware into **volatile RAM** after each power cycle. This
is the normal device boot process, not persistent firmware flashing. A cold
camera may first enumerate as a Movidius bootloader; `open()` uploads the image
and waits for it to reappear as a tracking device.

The default configuration downloads firmware **0.2.0.951** from the official
Intel URL used by librealsense v2.53.1, verifies its pinned SHA-256, and embeds
it into the shared library. The image is cached in the build directory and is
not committed to this repository. Version, URL, size, hashes, and licensing
details are recorded in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

For an offline build, provide a copy of the same image. CMake checks a supplied
local file against the same pinned SHA-256 and rejects other images:

```sh
cmake -S . -B build \
  -DLIBT26X_DOWNLOAD_FIRMWARE=OFF \
  -DLIBT26X_FIRMWARE_FILE=/path/to/target-0.2.0.951.mvcmd
```

To build a library without an embedded firmware image, disable downloading and
leave `LIBT26X_FIRMWARE_FILE` empty. Applications must then supply the firmware
path when opening a cold camera. A camera whose firmware is already running
does not need another upload.

```cpp
auto options = t26x::default_open_options();
options.firmware_path = "/path/to/target-0.2.0.951.mvcmd";
camera.open(options);
```

The equivalent C field is `t26x_open_options.firmware_path`. The capture example
also accepts `--firmware /path/to/target-0.2.0.951.mvcmd`.

The firmware has separate Intel licensing. No explicit firmware redistribution
grant was located in the upstream source or accompanying download, so it is
downloaded rather than stored in this source repository. Embedding it in a
compiled library does not change those terms. The MIT and Apache source
licenses do not grant firmware redistribution rights.

## Use from CMake, including OpenKAI

After installation, an application can link Libt26x independently of any
RealSense SDK it already uses:

```cmake
find_package(Libt26x CONFIG REQUIRED)
target_link_libraries(your_openkai_target PRIVATE Libt26x::t26x)
```

Replace `your_openkai_target` with the target that consumes camera data.
For a custom installation prefix, configure the application with
`-DCMAKE_PREFIX_PATH=/your/prefix`. The exported target supplies the include
directory and shared-library location. C applications use `<t26x/t26x.h>`;
C++ applications can use the RAII wrapper `<t26x/t26x.hpp>`.

For projects without CMake:

```sh
pkg-config --cflags --libs libt26x
```

## Read a pose

```cpp
#include <t26x/t26x.hpp>
#include <iostream>

int main() try {
    t26x::device camera;
    camera.open();
    camera.start();
    const auto pose = camera.wait_for_pose(1000);
    std::cout << pose.translation[0] << ' '
              << pose.translation[1] << ' '
              << pose.translation[2] << '\n';
    const auto confidence = camera.confidence();
    std::cout << "tracker confidence: " << confidence.tracker << '\n';
    camera.stop();
} catch (const t26x::error &error) {
    std::cerr << error.what() << '\n';
    return 1;
}
```

`wait_for_pose()` waits for a new sample arriving after the call. `pose()` copies
the latest cached sample and fails until the first sample arrives. Device
timestamps are nanoseconds on the camera clock, not wall-clock timestamps.

The included examples exercise the C and C++ interfaces:

```sh
./build/t26x-pose 5
./build/t26x-capture --seconds 5
./build/t26x-capture --seconds 5 --fisheye
./build/t26x-capture --seconds 5 --reset
./build/t26x-capture --help
```

Use `--serial SERIAL` to select a particular camera. `--reset` requests a
hardware reset after capture. The capture example counts and checks incoming
frames; it does not write image files. See
[`examples/t26x-pose.c`](examples/t26x-pose.c) for C error handling and
[`examples/t26x-capture.cpp`](examples/t26x-capture.cpp) for fisheye callbacks.

## API and lifetime

The C interface returns status codes and fills an error object. The C++ wrapper
throws exceptions on failed operations and closes its handle automatically.
Use a stream configuration to select pose alone or pose plus both fisheye
streams. Public declarations and callback contracts are in
[`include/t26x/t26x.h`](include/t26x/t26x.h).

Pose translation is in meters. The coordinate system is right-handed: x points
right, y up, and z toward the user when looking at the camera lenses. Quaternion
components are ordered x, y, z, w. Tracker and mapper confidence range from
0 (failed), through 1 (low) and 2 (medium), to 3 (high).

Serialize lifecycle operations on each handle in the application. Do not race
open/start/stop/reset/close with one another, and keep a handle alive until all
threads using it have finished. Pose getters may run while streaming.
Callbacks run on USB worker threads and should return promptly; callbacks from
different threads may run concurrently. They must not throw exceptions. Their
frame and pixel pointers remain valid only until the callback returns; copy
data you need to retain. Do not call lifecycle methods, including stop, reset,
and close, or wait for a pose from a callback. Signal an application thread
instead. Waiting for a pose from the camera's callback is rejected because it
would block its own reader.

`stop()` ends streaming and permits another `start()` on the same open handle.
`reset()` performs a hardware reset and invalidates the current connection.
In C, close that handle and open the camera again before streaming. The C++
wrapper's `reset()` closes the invalidated handle automatically; call `open()`
again afterward. A disconnected device also requires close and reopen.
A failed reset attempt may also invalidate the handle; close it before trying
to open the device again.

In C, `t26x_close()` consumes the handle even if it reports a USB stop error.
Do not retry close or use that pointer afterward; set your pointer to null.
The sole exception is a close rejected from a callback with
`T26X_INVALID_STATE`, which retains the handle. The C++ wrapper applies this
ownership rule even when `close()` throws an exception.

## Hardware lifecycle test

With `BUILD_TESTING=ON`, the following executable checks pose reception,
stop/start on the same handle, hardware reset, and reopening the camera:

```sh
./build/t26x-test-hardware
```

It resets the selected camera. Stop any other application using that camera
before running it. Optional arguments are `--serial SERIAL`,
`--firmware /path/to/target-0.2.0.951.mvcmd`, and `--boot-timeout-ms N`.
It tests pose only; use `t26x-capture --fisheye` separately on a USB 3 connection
to check both image streams.

The hardware test is excluded from default CTest runs. To register it with
CTest, configure with `-DLIBT26X_HARDWARE_TESTS=ON`. The regular protocol, API,
and ownership tests do not need a camera.

## Validation

Initial validation on 2026-10-22:

- Native Linux x86-64 Release build succeeded; all three protocol, API, and
  ownership tests passed. Protocol and API tests also passed with AddressSanitizer
  and UndefinedBehaviorSanitizer.
- Installed-package consumers built and ran using both C and C++11 with
  `find_package(Libt26x CONFIG REQUIRED)`. pkg-config integration was checked.
- Firmware download, pinned local-file validation, and builds without embedded
  firmware were checked.
- The core source and firmware assembly compiled for ARM32. No ARM runtime or
  ARM64 build validation was performed.
- The connected T265 booted firmware 0.2.0.951 and delivered 897 pose samples
  during a 5.48-second USB 2 capture with no errors and reported tracker
  confidence 2.
- Three stop/start cycles on the same handle each delivered 20 pose samples.
  A subsequent hardware reset, firmware reload, reopen, and fourth capture
  also delivered 20 samples; tracker/mapper confidence was 2/0.
  The full lifecycle test also passed with a runtime firmware file under
  AddressSanitizer, UndefinedBehaviorSanitizer, and LeakSanitizer.

USB 3 fisheye streaming and T261 hardware have not been validated; architecture
and protocol support alone do not establish those results.

## License

Original Libt26x interfaces, examples, build files, and documentation are
[MIT licensed](LICENSE). The extracted Intel protocol header and adapted USB
implementation retain [Apache License 2.0](LICENSES/Apache-2.0.txt).
See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for file provenance,
the system libusb dependency, and the separately licensed firmware.

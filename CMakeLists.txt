cmake_minimum_required(VERSION 3.16)
project(Libt26x VERSION 0.1.0 LANGUAGES C CXX ASM)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "Libt26x supports Linux only.")
endif()
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _libt26x_processor)
if(NOT _libt26x_processor MATCHES "^(x86_64|amd64|aarch64|arm64|arm|armv[5-9].*)$")
    message(FATAL_ERROR "Libt26x supports Linux x86_64 and ARM; got '${CMAKE_SYSTEM_PROCESSOR}'.")
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CTest)

option(LIBT26X_BUILD_EXAMPLES "Build the pose and fisheye capture examples" ON)
option(LIBT26X_DOWNLOAD_FIRMWARE "Download and embed the pinned Intel T265 firmware" ON)
option(LIBT26X_INSTALL_UDEV_RULES "Install USB access rules (requires udev reload afterward)" OFF)
option(LIBT26X_HARDWARE_TESTS "Register the hardware lifecycle test with CTest (resets a connected T265)" OFF)
set(LIBT26X_FIRMWARE_FILE "" CACHE FILEPATH "Existing T265 .mvcmd firmware to embed instead of downloading")

find_package(Threads REQUIRED)
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(LIBUSB QUIET IMPORTED_TARGET libusb-1.0>=1.0.20)
endif()
if(TARGET PkgConfig::LIBUSB)
    set(_libt26x_usb_target PkgConfig::LIBUSB)
else()
    find_path(LIBUSB_INCLUDE_DIR NAMES libusb.h PATH_SUFFIXES libusb-1.0)
    find_library(LIBUSB_LIBRARY NAMES usb-1.0)
    if(NOT LIBUSB_INCLUDE_DIR OR NOT LIBUSB_LIBRARY)
        message(FATAL_ERROR "libusb-1.0 development files are required (for example, libusb-1.0-0-dev).")
    endif()
    add_library(Libt26x_libusb UNKNOWN IMPORTED)
    set_target_properties(Libt26x_libusb PROPERTIES
        IMPORTED_LOCATION "${LIBUSB_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${LIBUSB_INCLUDE_DIR}")
    set(_libt26x_usb_target Libt26x_libusb)
    mark_as_advanced(LIBUSB_INCLUDE_DIR LIBUSB_LIBRARY)
endif()

include(cmake/Firmware.cmake)

add_library(t26x SHARED src/t26x.cpp ${_libt26x_firmware_source})
add_library(Libt26x::t26x ALIAS t26x)
target_compile_features(t26x PRIVATE cxx_std_14)
target_compile_definitions(t26x PRIVATE
    LIBT26X_HAVE_EMBEDDED_FIRMWARE=${_libt26x_have_firmware}
    LIBT26X_VERSION="${PROJECT_VERSION}")
target_include_directories(t26x
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(t26x PRIVATE ${_libt26x_usb_target} Threads::Threads)
target_link_options(t26x PRIVATE "LINKER:--version-script=${CMAKE_CURRENT_SOURCE_DIR}/cmake/t26x.map")
set_target_properties(t26x PROPERTIES
    VERSION "${PROJECT_VERSION}"
    SOVERSION "${PROJECT_VERSION_MAJOR}"
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    CXX_EXTENSIONS NO
    LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/t26x.map")
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(t26x PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wpedantic>)
endif()

if(LIBT26X_BUILD_EXAMPLES)
    add_executable(t26x-pose examples/t26x-pose.c)
    target_compile_features(t26x-pose PRIVATE c_std_99)
    target_link_libraries(t26x-pose PRIVATE Libt26x::t26x)
    add_executable(t26x-capture examples/t26x-capture.cpp)
    target_compile_features(t26x-capture PRIVATE cxx_std_11)
    target_link_libraries(t26x-capture PRIVATE Libt26x::t26x Threads::Threads)
endif()

if(BUILD_TESTING)
    foreach(_test IN ITEMS protocol api)
        add_executable(t26x-test-${_test} tests/${_test}.cpp)
        target_compile_features(t26x-test-${_test} PRIVATE cxx_std_14)
        target_include_directories(t26x-test-${_test} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
        target_link_libraries(t26x-test-${_test} PRIVATE Libt26x::t26x Threads::Threads)
        add_test(NAME ${_test} COMMAND t26x-test-${_test})
    endforeach()
    add_executable(t26x-test-ownership tests/ownership.cpp)
    target_compile_features(t26x-test-ownership PRIVATE cxx_std_11)
    target_include_directories(t26x-test-ownership PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
    add_test(NAME ownership COMMAND t26x-test-ownership)
    add_executable(t26x-test-hardware tests/hardware.cpp)
    target_compile_features(t26x-test-hardware PRIVATE cxx_std_14)
    target_link_libraries(t26x-test-hardware PRIVATE Libt26x::t26x Threads::Threads)
    if(LIBT26X_HARDWARE_TESTS)
        add_test(NAME hardware COMMAND t26x-test-hardware)
        set_tests_properties(hardware PROPERTIES TIMEOUT 20 RUN_SERIAL TRUE LABELS hardware)
    endif()
endif()

install(TARGETS t26x EXPORT Libt26xTargets
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(DIRECTORY include/t26x DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
install(EXPORT Libt26xTargets
    FILE Libt26xTargets.cmake
    NAMESPACE Libt26x::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/Libt26x")
configure_package_config_file(cmake/Libt26xConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/Libt26xConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/Libt26x")
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/Libt26xConfigVersion.cmake"
    VERSION "${PROJECT_VERSION}" COMPATIBILITY SameMajorVersion)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/Libt26xConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/Libt26xConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/Libt26x")
foreach(_directory IN ITEMS LIBDIR INCLUDEDIR)
    if(IS_ABSOLUTE "${CMAKE_INSTALL_${_directory}}")
        set(LIBT26X_PC_${_directory} "${CMAKE_INSTALL_${_directory}}")
    else()
        set(LIBT26X_PC_${_directory} "\${prefix}/${CMAKE_INSTALL_${_directory}}")
    endif()
endforeach()
configure_file(cmake/libt26x.pc.in "${CMAKE_CURRENT_BINARY_DIR}/libt26x.pc" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/libt26x.pc"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")
install(FILES LICENSE DESTINATION "${CMAKE_INSTALL_DOCDIR}")
foreach(_doc IN ITEMS README.md NOTICE THIRD_PARTY_NOTICES.md)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_doc}")
        install(FILES "${_doc}" DESTINATION "${CMAKE_INSTALL_DOCDIR}")
    endif()
endforeach()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/LICENSES")
    install(DIRECTORY LICENSES DESTINATION "${CMAKE_INSTALL_DOCDIR}")
endif()
if(LIBT26X_INSTALL_UDEV_RULES)
    # udev searches lib/udev, never a multiarch directory such as lib/x86_64-linux-gnu.
    install(FILES packaging/99-libt26x.rules DESTINATION "lib/udev/rules.d")
endif()

message(STATUS "Libt26x ${PROJECT_VERSION}: embedded T265 firmware = ${_libt26x_have_firmware}")

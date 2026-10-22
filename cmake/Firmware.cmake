# The firmware is a separate Intel binary; the project's MIT license does not
# grant rights to it. Fetch it from Intel instead of placing it in the source tree.
set(_libt26x_firmware_version "0.2.0.951")
set(_libt26x_firmware_sha256 "0265fd111611908b822cdaf4a3fe5b631c50539b2805d2f364c498aa71c007c0")
set(_libt26x_have_firmware 0)
set(_libt26x_firmware_source "")
set(_libt26x_firmware_path "")

if(LIBT26X_FIRMWARE_FILE)
    get_filename_component(_libt26x_firmware_path "${LIBT26X_FIRMWARE_FILE}" ABSOLUTE
        BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    if(NOT EXISTS "${_libt26x_firmware_path}" OR IS_DIRECTORY "${_libt26x_firmware_path}")
        message(FATAL_ERROR "LIBT26X_FIRMWARE_FILE must name an existing .mvcmd file: ${_libt26x_firmware_path}")
    endif()
    file(SHA256 "${_libt26x_firmware_path}" _libt26x_actual_hash)
    if(NOT _libt26x_actual_hash STREQUAL _libt26x_firmware_sha256)
        message(FATAL_ERROR
            "LIBT26X_FIRMWARE_FILE SHA256 does not match the pinned T265 "
            "firmware ${_libt26x_firmware_version}. Expected ${_libt26x_firmware_sha256}, "
            "got ${_libt26x_actual_hash}.")
    endif()
elseif(LIBT26X_DOWNLOAD_FIRMWARE)
    set(_libt26x_firmware_dir "${CMAKE_CURRENT_BINARY_DIR}/_deps/firmware")
    file(MAKE_DIRECTORY "${_libt26x_firmware_dir}")
    set(_libt26x_firmware_path "${_libt26x_firmware_dir}/target-${_libt26x_firmware_version}.mvcmd")
    set(_libt26x_download_needed TRUE)
    if(EXISTS "${_libt26x_firmware_path}")
        file(SHA256 "${_libt26x_firmware_path}" _libt26x_actual_hash)
        if(_libt26x_actual_hash STREQUAL _libt26x_firmware_sha256)
            set(_libt26x_download_needed FALSE)
        endif()
    endif()
    if(_libt26x_download_needed)
        set(_libt26x_firmware_url
            "https://librealsense.intel.com/Releases/TM2/FW/target/${_libt26x_firmware_version}/target-${_libt26x_firmware_version}.mvcmd")
        message(STATUS "Downloading Intel T265 firmware ${_libt26x_firmware_version}")
        file(DOWNLOAD "${_libt26x_firmware_url}" "${_libt26x_firmware_path}.tmp"
            STATUS _libt26x_download_status
            TLS_VERIFY ON TIMEOUT 180 INACTIVITY_TIMEOUT 30)
        list(GET _libt26x_download_status 0 _libt26x_download_code)
        list(GET _libt26x_download_status 1 _libt26x_download_message)
        if(NOT _libt26x_download_code EQUAL 0)
            file(REMOVE "${_libt26x_firmware_path}.tmp")
            message(FATAL_ERROR
                "Cannot download T265 firmware: ${_libt26x_download_message}. "
                "Set LIBT26X_FIRMWARE_FILE to a local .mvcmd image, or set "
                "LIBT26X_DOWNLOAD_FIRMWARE=OFF and supply firmware at runtime.")
        endif()
        file(SHA256 "${_libt26x_firmware_path}.tmp" _libt26x_actual_hash)
        if(NOT _libt26x_actual_hash STREQUAL _libt26x_firmware_sha256)
            file(REMOVE "${_libt26x_firmware_path}.tmp")
            message(FATAL_ERROR "Downloaded T265 firmware SHA256 does not match the pinned image.")
        endif()
        file(RENAME "${_libt26x_firmware_path}.tmp" "${_libt26x_firmware_path}")
    endif()
endif()

if(_libt26x_firmware_path)
    file(SIZE "${_libt26x_firmware_path}" _libt26x_firmware_size)
    if(_libt26x_firmware_size EQUAL 0)
        message(FATAL_ERROR "T265 firmware is empty: ${_libt26x_firmware_path}")
    endif()
    # GAS accepts the same incbin syntax on x86_64, ARM32, and AArch64.
    string(REPLACE "\\" "\\\\" LIBT26X_FIRMWARE_ASM_PATH "${_libt26x_firmware_path}")
    string(REPLACE "\"" "\\\"" LIBT26X_FIRMWARE_ASM_PATH "${LIBT26X_FIRMWARE_ASM_PATH}")
    set(_libt26x_firmware_source "${CMAKE_CURRENT_BINARY_DIR}/generated/firmware.S")
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/generated")
    configure_file("${CMAKE_CURRENT_LIST_DIR}/firmware.S.in" "${_libt26x_firmware_source}" @ONLY)
    set_source_files_properties("${_libt26x_firmware_source}" PROPERTIES
        OBJECT_DEPENDS "${_libt26x_firmware_path}")
    set(_libt26x_have_firmware 1)
endif()

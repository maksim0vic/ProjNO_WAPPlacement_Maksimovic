cmake_minimum_required(VERSION 3.18)

set(SOLUTION_NAME wapProject)

# BEGIN common part for all solutions
project(${SOLUTION_NAME})
set(SOURCE_ROOT ${CMAKE_CURRENT_LIST_DIR})

if(WIN32)
    set(HOME_ROOT $ENV{USERPROFILE})
    string(REPLACE "\\" "/" HOME_ROOT "${HOME_ROOT}")
else()
    set(HOME_ROOT $ENV{HOME})
endif()

set(NATID_SDK_ROOT ${HOME_ROOT}/natID.SDK)
include(${NATID_SDK_ROOT}/DevEnv/Common.cmake)
# END common part for all solutions

include(${NATID_SDK_ROOT}/DevEnv/MatrixLib.cmake)
include(${NATID_SDK_ROOT}/DevEnv/natGUI.cmake)


include(no_project.cmake)

# --- Global macOS Xcode Code-Signing and Bundle Bypass ---
if(APPLE)
    set(CMAKE_XCODE_ATTRIBUTE_CODE_SIGN_STYLE "Manual" CACHE STRING "" FORCE)
    set(CMAKE_XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "" CACHE STRING "" FORCE)
    set(CMAKE_XCODE_ATTRIBUTE_CODE_SIGNING_REQUIRED "NO" CACHE STRING "" FORCE)
    set(CMAKE_XCODE_ATTRIBUTE_CODE_SIGNING_ALLOWED "NO" CACHE STRING "" FORCE)
    set(CMAKE_XCODE_ATTRIBUTE_EXPANDED_CODE_SIGN_IDENTITY "" CACHE STRING "" FORCE)
    set(CMAKE_XCODE_ATTRIBUTE_CODE_SIGN_ENTITLEMENTS "" CACHE STRING "" FORCE)
endif()

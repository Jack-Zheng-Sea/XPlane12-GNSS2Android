cmake_minimum_required(VERSION 3.10)
project(XP12GNSS LANGUAGES CXX)

# X-Plane 12 plugin: broadcast aircraft state to an Android phone.
# Build: cmake -B build && cmake --build build
# The resulting win.xpl / lin.xpl / mac.xpl goes into
#   <X-Plane 12>/Resources/plugins/XPlane12-GNSS2Android/<platform>/

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PLUGIN_SOURCES
    src/XP12GNSS.cpp
    src/GpsUdpServer.cpp)

if(WIN32)
    add_library(gnss2android MODULE ${PLUGIN_SOURCES})
    target_compile_definitions(gnss2android PRIVATE WIN32 XPLM200 _CRT_SECURE_NO_WARNINGS _WINSOCK_DEPRECATED_NO_WARNINGS)
    target_include_directories(gnss2android PRIVATE src SDK/CHeaders/XPLM)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        target_link_libraries(gnss2android PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/SDK/Libraries/Win/XPLM_64.lib" Ws2_32)
    else()
        target_link_libraries(gnss2android PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/SDK/Libraries/Win/XPLM.lib" Ws2_32)
    endif()
    set_target_properties(gnss2android PROPERTIES PREFIX "" SUFFIX ".xpl")
elseif(APPLE)
    add_library(gnss2android MODULE ${PLUGIN_SOURCES})
    target_compile_definitions(gnss2android PRIVATE APL=1 XPLM200)
    target_include_directories(gnss2android PRIVATE src SDK/CHeaders/XPLM)
    set_target_properties(gnss2android PROPERTIES PREFIX "" SUFFIX ".xpl")
else()
    # Linux: XPLM symbols are resolved at load time, no import library needed.
    add_library(gnss2android MODULE ${PLUGIN_SOURCES})
    target_compile_definitions(gnss2android PRIVATE LIN=1 XPLM200)
    target_include_directories(gnss2android PRIVATE src SDK/CHeaders/XPLM)
    set_target_properties(gnss2android PROPERTIES PREFIX "" SUFFIX ".xpl")
endif()

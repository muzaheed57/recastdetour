# This module defines the macro COMPUTE_PLATFORM_STR

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.7)

MACRO(COMPUTE_PLATFORM_STR varname)
    IF(MSVC)
        MATH(EXPR msvc_normalized_version "(${MSVC_VERSION} - 600) / 10" )
        IF( ${CMAKE_SIZEOF_VOID_P} EQUAL 4 )
            SET(${varname} vc${msvc_normalized_version})
        ELSE()
            SET(${varname} vc${msvc_normalized_version}_x64)
        ENDIF()
    ELSEIF(DEFINED XCODE_VERSION)
        STRING(REPLACE "." "_" ${varname} "xcode${XCODE_VERSION}")
    ELSEIF(UNIX)
        SET(${varname} linux)
    ENDIF()
    MESSAGE("Detected platform: ${${varname}}")
ENDMACRO()
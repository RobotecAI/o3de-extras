# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

function(target_depends_on_ros2_package TARGET_NAME )
    list (GET ARGN 0 _package)
    find_package(${ARGN})
    include(${${_package}_DIR}/${_package}Config.cmake OPTIONAL)
    include(${${_package}_DIR}/${_package}config.cmake OPTIONAL)
    include(${${_package}_DIR}/${_package}-config.cmake OPTIONAL)
    include(${${_package}_DIR}/${_package}-Config.cmake OPTIONAL)
    if( ${${_package}_FOUND} )
        message(DEBUG "Package ${_package} was found (${${_package}_DIR}) version ${${_package}_VERSION} targets : ${${_package}_TARGETS}")
        target_link_libraries(${TARGET_NAME} PUBLIC ${${_package}_TARGETS})
    endif()
endfunction()

function(target_depends_on_ros2_packages TARGET_NAME)
    foreach(_package IN LISTS ARGN)
        message(DEBUG "Processing package: ${_package}")
        target_depends_on_ros2_package(${TARGET_NAME} ${_package} REQUIRED)
    endforeach()
endfunction()





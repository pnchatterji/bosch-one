# Copyright (c) 2023 Bosch Sensortec GmbH
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

#-DUSE_SENSOR=<sensor name> -DUSE_EXAMPLE=<example name> 
#should be passed as command line arguments to CMake or West when building from console. 
#If building using VSCode, the SENSOR and EXAMPLE variables should be passed as 
#'Additional CMake Arguments' in the Build Configuration settings
if(USE_SENSOR)
	set(SENSOR ${USE_SENSOR})
	set(EXAMPLE ${USE_EXAMPLE})
else()
	set(SENSOR bmi270)
	set(EXAMPLE accel)
endif()
message("Building for Sensor ${SENSOR} Example ${EXAMPLE}")

set(SENSOR_DIR $ENV{ZEPHYR_BASE}/../bosch-one/sensor-api/${SENSOR})
set(EXAMPLE_DIR ${SENSOR_DIR}/examples/${SENSOR}/${EXAMPLE})
set(COMMON_DIR ${SENSOR_DIR}/examples/${SENSOR}/common)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sensor_app)

message("example dir ${EXAMPLE_DIR}")
FILE(GLOB example_sources ${EXAMPLE_DIR}/*.c ${COMMON_DIR}/*.c ${SENSOR_DIR}/*.c)

#add the source files of this application
target_include_directories(app PUBLIC ${SENSOR_DIR} ${COMMON_DIR} ${SENSOR_DIR})
target_sources(app PRIVATE  ${example_sources})

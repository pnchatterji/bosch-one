#
# Copyright (c) 2022 Bosch Sensortec GmbH
#
# SPDX-License-Identifier: Apache-2.0
#
#TIP: This is a test makefile that builds the sample and driver
#together into a single test app, prior to deployment
#this file will not actually be deployed to Zephyr

cmake_minimum_required(VERSION 3.20.0)

#TIP: below line is only required to make this test app build with AB3 DTS
#as long as the AB3 DTS is not added to Zephyr repository 

list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../boards)

#TIP: explicitely load sample overlay and conf file, as sample is not the app
#root directory for the test project. when sample directory is getting
#built after deployment, the overlay and conf will get loaded automatically

set(DTC_OVERLAY_FILE sample/boards/${BOARD}.overlay)
list(APPEND CONF_FILE ${CMAKE_CURRENT_SOURCE_DIR}/sample/prj.conf)

find_package(Zephyr HINTS $ENV{ZEPHYR_BASE})
project(testbxx)


FILE(GLOB drv_sources driver/*.c)
FILE(GLOB sample_sources sample/src/*.c)
target_sources(app PRIVATE ${drv_sources} ${sample_sources})

target_include_directories(app PUBLIC driver sample/src)

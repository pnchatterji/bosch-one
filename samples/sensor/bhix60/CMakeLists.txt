#
# Copyright (c) 2023 Bosch Sensortec GmbH
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.20.0)

#add AB3 specific configurations
if("${BOARD}" STREQUAL "bosch_app3")
	set(CONF_FILE "prj.conf")
	list(APPEND CONF_FILE "boards/bosch_app3x_overlay.conf")
endif()

find_package(Zephyr HINTS $ENV{ZEPHYR_BASE})
project(bhix60)

#The required sample to be built has to be selected by passing
#e.g. -DBUILD_PDR in the build command line 
#Or, if using VSCode, by entering -DBUILD_PDR:STRING="1" in 
#the field "Extra CMake Argument" in the Build Configuration dialog.
if(BUILD_SWIM)
	SET(app_sources src/main_swim.c)
	message(STATUS "Building SWIM Sample")
elseif(BUILD_PDR)
	SET(app_sources src/main_pdr.c)
	message(STATUS "Building PDR Sample")
elseif(BUILD_KLIO)
	SET(app_sources src/main_klio.c)
	message(STATUS "Building KLIO Sample")
elseif(BUILD_BSEC)
	SET(app_sources src/main_bsec.c)
	message(STATUS "Building BSEC Sample")
elseif(BUILD_AUX)
	SET(app_sources src/main_aux.c)
	message(STATUS "Building AUX Sample")
elseif(BUILD_WAKE)
	SET(app_sources src/main_wake.c)
	message(STATUS "Building WAKE Sample")
elseif(BUILD_BHI3)
	SET(app_sources src/main_bhi3.c)
	message(STATUS "Building BHI3 Sample")
else()
	SET(app_sources src/main.c)
	message(STATUS "Building Default Sample")
endif()

target_sources(app PRIVATE ${app_sources})

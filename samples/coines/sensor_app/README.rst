.. _sensor_app:

Sensor App
###########

Overview
********

A generic project to build and test sensor examples in the standard COINES examples directory 
using COINES for Zephyr.

This project contains no source code, just the CMake file and config files

The example source code is added automatically by the CMake file from 
<ZephyrRoot>/bosch-one/sensor-api/<sensor>/examples/<sensor>/<example>

The sensor name and example name have to be passed as additional build arguments, as described below


Building and Running with VSCode
********************************

This project can be built using VS Code with nrfConnect plugin, and executed on BOSCH Application Board 3:
Currently, it only supports the board bst_ab3_nrf52840

The macros 
-DUSE_SENSOR=<sensor name> 
-DUSE_EXAMPLE=<example name> 

need to be passed as 'Additional CMake Arguments' in the Build Configuration settings

Note that in the VSCode *Edit Build Configuration* dialog box, both macros need to be added
individually by clicking on the *Add argument* button, specifying the macro value as a string. 
Example: 
-DUSE_SENSOR:STRING="bmi270"; 
-DUSE_EXAMPLE:STRING="accel";


The output can be viewed in HTERM console or in Segger RTT Viewer,, depending on which method
is activated in prj.conf

Output
******

This depends on the selected sensor and example
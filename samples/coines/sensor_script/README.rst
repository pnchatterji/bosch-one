.. _sensor_script:

Sensor Script
#############

Overview
********

A CMD script to build and run all the COINES examples of a selected sensor 
using COINES for Zephyr.

This project contains no source code, just the CMake, config and BAT files

The example source code is added automatically by the CMake file, one example at a time, from 
<ZephyrRoot>/bosch-one/sensor-api/<sensor>/examples/<sensor>

The sensor name has to be passed on the command line to the script, as described below


Automated Testing
*****************

The testall script builds and runs all available examples in the examples directory for a particular 
sensor. The sensor name has to be passed on the command line of the script. 
The results are logged to testlog-<sensorname>-<timestamp>.log
Note that testall.bat uses Segger Jrun. The path to JRun.exe may have to be modified in testone.bat
if Segger J-Link package is not installed in its default directory. Ensure that Segger J-Link is connected
to the board and switched on prior to running this test, otherwise the script will fail.
 
Arguments: SENSOR_NAME - Name of Sensor to be processed  
           
Example: testall BMI270 

The buildone and testone scripts can also be used to test a single example. Refer the comments in the
script file for usage help

NOTE1: These scripts should only be called in the console created by clicking on the Zephyr-shell.bat
file included in this package. Zephyr environment variables need to be set properly for these script 
to work. The scripts will fail if run in an ordinary CMD console

NOTE2: The path to the current Zephyr installation needs to be modified in Zephyr-shell.bat prior
to first use, and subsequently if Zephyr is reinstalled to another directory

Output
******
In case a particular example does not build, it will appear in the testlog as "Build Failed", and the 
buildlog of that particular example will be saved in 
buildlog-<sensorname>-<example name>-<timestamp>.log
Note that successful builds are not logged.
If a particular example crashes during execution, it will be logged as FATAL ERROR in the testlog file.
In this case, the particular example can be degugged using the VSCode method explained in the previous section



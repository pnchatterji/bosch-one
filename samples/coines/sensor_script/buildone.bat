@echo off
:: Brief: Build ONE standard COINES examples for a sensor using COINES for Zephyr
:: Arguments: SENSOR_NAME 		- Name of Sensor to be processed 
::			  EXAMPLE_NAME 		- Name of Example to be built
:: Example: buildone BMI270 accel
::
:: NOTE: This script should only be called in the Zephyr Shell created by clicking 
:: on the zephyr-shell.bat command as Zephyr Environment Variables need to be set
:: properly for this script to work. It may be necessary to edit zephyr-shell.bat
:: to update the Zephyr installation path.

if [%1]==[] goto argerr
if [%2]==[] goto argerr

set "APPDIR=%cd%"
west build --build-dir "%APPDIR%\build" "%APPDIR%" --pristine --board bosch_app30 -- -DNCS_TOOLCHAIN_VERSION:STRING="NONE" -DUSE_SENSOR=%1 -DUSE_EXAMPLE=% -DCOINES_AUTO_TEST=1

exit /b

:argerr
echo Expected arguments SENSOR_NAME EXAMPLE_NAME 
echo e.g. buildone BMI270 accel
exit /b

::Open command shell and Set build environment variables for Zephyr.
::NOTE: Change NCS Version Path to current version
@echo off
set ZVER=V2.3.0
set ZEPHYR_BASE="%USERPROFILE%\ncs\%ZVER%\zephyr"
echo Using nrf Connect Version %ZVER%
echo Zephyr BASE: %ZEPHYR_BASE%
echo Verify if Zephyr Base is correctly set prior to running the build and test scripts
cmd /k "%ZEPHYR_BASE%\zephyr-env.cmd"


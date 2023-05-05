@echo off
:: Brief: Build and test all standard COINES examples for a sensor using COINES for Zephyr
:: Arguments: SENSOR_NAME -	Name of Sensor to be processed  
:: Example: testall BMI270
::
:: NOTE: This script should only be called in the Zephyr Shell created by clicking 
:: on the zephyr-shell.bat command as Zephyr Environment Variables need to be set
:: properly for this script to work. It may be necessary to edit zephyr-shell.bat
:: to update the Zephyr installation path.

if [%1]==[] goto argerr

set expath=%ZEPHYR_BASE%\..\bosch-one\sensor-api\%1\examples\%1
echo processing example dir %expath%
set timestamp=%DATE:/=-%T%TIME::=-%
set timestamp=%timestamp: =%
set timestamp=%timestamp:.=-%
set timestamp=%timestamp:,=-%
set LOGFL="testlog-%1-%timestamp%.log"
for /f %%G in ('dir /b /A:D %expath%') do (
if %%G NEQ common (
rmdir /S /Q build 2>NUL
echo Testing Sensor %1 Example %%G
echo Testing Sensor %1 Example %%G >> %LOGFL%
call buildone %1 %%G 1>buildlog.temp 2>&1
if EXIST build\zephyr\zephyr.elf (
call testone >> %LOGFL%
echo Testing Over Sensor %1 Example %%G >> %LOGFL%
echo Testing Over Sensor %1 Example %%G
) else (
echo Sensor %1 Example %%G Build Failed! >> %LOGFL%
echo Sensor %1 Example %%G Build Failed!
copy /Y buildlog.temp "buildlog-%1-%%G-%timestamp%.log"
)
echo ====================================================================== >> %LOGFL%
)
@echo off
)
echo ALL TESTS OVER!!!!
exit /b


:argerr
echo Expected arguments SENSOR_NAME 
echo e.g. testall BMI270
echo on
exit /b

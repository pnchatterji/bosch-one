.. _BMI270_Accel_app:

COINES BMI270 Accel Sample App
##############################

Overview
********

This COINES for Zephyr (ZCOINES) sample application demonstrates the basic
acclerometer functions of the BMI270 sensor using the BMI270 Sensor API
integrated with ZCOINES

*Abbreviations used:* 
AB3 - Application Board 3.x

Wiring
******

AB3 mounted with BMI270 shuttle board is required for this application


Building and Running
********************
Prerequisite: nrf Connect is installed on the host machine, and bosch-one module is installed

1. Open nrf Connect plugin on VS Code
2. Click on Create a new Application
3. In the template field, select bosch-one/coines/bmi270/accel
4. Enter the desired project name and location
5. Create a build configuration for the application for bosch_app30 or bosch_app31
6. Perform a pristine build 
7. Follow the instructions in the bosch-one/doc folder for flashing and running on the target board
8. Launch a console application such as HTERM and connect to the target for performing the tests.

Recommended Settings for HTERM
******************************
Send on Enter: CR 
Show newlines: No
Newline At: CR
Baud:115200

Sample Output
*************

.. code-block:: console





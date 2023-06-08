.. _COINES_app:

COINES Bridge Firmware
######################

Overview
********

COINES Bridge Firmware on ZCOINES  
Application Board 3.0 and Arduino Nicla Sense ME

*Abbreviations used:* 
AB3 - Application Board 3.0 
Nicla - Arduino Nicla Sense ME

Wiring
******


Building and Running
********************
Prerequisite: nrf Connect is installed on the host machine, and bosch-one module is installed

1. Open nrf Connect plugin on VS Code
2. Click on Create a new Application
3. In the template field, select bosch-one/coines_app
4. Enter the desired project name and location
5. Create a build configuration for the application using any of the supported boards
	- bst_arduino_nicla
	- bst_ab3_nrf52840
6. Perform a pristine build 
7. Follow the instructions in the bosch-one/doc folder for flashing and running on the target board
8. Build and launch coines_bridge_test.exe from the baremetal COINES installation


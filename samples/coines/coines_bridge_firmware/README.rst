.. _COINES_app:

COINES Bridge Firmware
######################
COINES Bridge is a firmware that allows applications running on a desktop
to communicate with a BST application board via USB/BLE and execute COINES
operations on it remotely, without having to flash the embedded application.
 
Overview
********

COINES Bridge Firmware on ZCOINES  
Application Board 3.x and Arduino Nicla Sense ME

*Abbreviations used:* 
AB3 - Application Board 3.x 
Nicla - Arduino Nicla Sense ME

Wiring
******
For AB3.x: Connect desktop to board via J-Link debug probe on the debug pins of the board
to flash the firmware. Connect to USB port of the board for application communication
via USB
For Nicla: Connect to USB port of the board both for flashing and application communication
via USB

Building and Running
********************
Prerequisite: nrf Connect is installed on the host machine, and bosch-one module is installed

1. Open nrf Connect plugin on VS Code
2. Click on Create a new Application
3. In the template field, select bosch-one/coines_bridge_firmware
4. Enter the desired project name and location
5. Create a build configuration for the application using any of the supported boards
	- bosch_nicla_sense
	- bosch_app3x
6. Perform a pristine build 
7. Follow the instructions in the bosch-one/doc folder for flashing and running on the target board
8. Build and launch coines_bridge_test.exe in a windows console from the baremetal COINES installation
   to test the communication with the coines bridge
9. Use any of the BST Windows/Linux Applications that use COINES bridge as the backend.


.. _COINES_app:

COINES Sample App
#################

Overview
********

This COINES for Zephyr (ZCOINES) sample application tests the complete COINES API on 
Application Board 3.0 and Arduino Nicla Sense ME

*Abbreviations used:* 
AB3 - Application Board 3.0 
Nicla - Arduino Nicla Sense ME

Wiring
******

In case of AB3, it is assumed that the BMI270 shuttle board is mounted on it. This affects principally
the SPI and I2C tests. For the other tests, the shuttle board is ignored.

In case of Nicla, only the primary SPI test is available, and it tests the communication with the host SPI
interface of the on-board BHI260 sensor.

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
8. Launch a console application such as HTERM and connect to the target for performing the tests.

Recommended Settings for HTERM
******************************
Send on Enter: CR 
Show newlines: No
Newline At: CR
Baud:115200

Sample Output
*************

There are over ten semi-interactive tests covering different parts of the COINES API. The desired tests
can be activated in the main() function, and the others commented out. The output depends on the
selected test. Self-explanatory console prompts guide the user in performing the tests.
There is also an interactive shell for running the desired test from the console. This can be
run by activating shell_main() in main().

This is the output displayed by the interactive test shell. Enter the desired test number to execute it.
After the test, press ENTER to re-display the menu and run another test (This requires the
setting *Send on Enter: CR* in HTERM to function properly).  

.. code-block:: console

	1. test_usb_cdc
	2. test_ble
	3. test_i2c
	4. test_spi
	5. test_bat_temp
	6. test_gpio_led
	7. test_gpio_int
	8. test_timed_gpio_int
	9. test_timer_interrupt
	10. test_sys_cmds
	11. test_fs
	12. fs_format_remount
	13. print_dir
	100. Quit TEST Application
	Enter Test no:






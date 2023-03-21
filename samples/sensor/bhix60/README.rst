.. _bhix60:

BHIx60: Programmable smart sensor combining accelerometer, gyroscope and fusion software
########################################################################################

Description
***********
The BHIx60 series of devices (BHI260, BHI360 etc.) are smart sensors that process 
information from multiple physical sensors to produce computed abstract data. 
Each computed data type is referred to as a “virtual sensor”. The virtual sensors 
supported by a particular device depends on the firmware that is currently loaded 
on it.

This set of sample applications demonstrate some of the features of BHIx60 devices.

1. The default sample (main.C) demonstrates using the driver both via the Zephyr 
Sensor API, and the raw interface defined in bhix60.h. It demonstrates accessing 
virtual sensor data using sensor_channel_get, the raw interface, and using triggers, 
and how to set up the fetch loop. This sample uses only the on-board sensors 
of the BHIx60

2. The sample main_aux.C demonstrates reading the virtual sensors that require 
require auxiliary BME688, BMM150, BMP390 devices to be connected to BHIx60
This sample also demonstrates setting threshold triggers.

3. The samples main_swim.C, main_pdr.C, main_klio.C, main_bsec.C demonstrate the use
of the complex virtual sensors SWIM, PDR, KLIO and BSEC.

4. The sample main_wake.C demonstrates how the wakeup features of BHIx60 can be 
integrated with the Zephyr Power Management module to reduce power consumption

5. The sample main_custom.C demonstrates how custom (user defined) virtual sensors
can be supported via the bhix60 driver.

There is a common CMake file to build all the above samples. The required sample
can be built by passing the appropriate command-line option to the build manager.
Refer the next section for details.

Note: the main_custom.C sample is for information only, and cannot be built unless
the user creates a suitable BHIx60 firmware to support it.

There is further documentation available in the form of detailed code comments
in the individual C files.

Refer the bhix60 driver user guide (available on the Bosch Sensortec website) for more
information on this driver.

References
**********

 - BHI260: https://www.bosch-sensortec.com/products/smart-sensors/bhi260ab/

Wiring
*******

These samples are based on the arduino_nicla_sense_me board, which has an integrated BHI260AP sensor 
connected via SPI to a host nRF52832, with BMP390 (pressure sensor), BMM150 (3-axis magnetometer) 
and BME688 (environment sensor) as auxiliary sensors connected via i2c and SPI to the BHI260. 
However, they can be adapted to any other board with a similar configuration.

Building and Running
********************

Other than the default sample, the src directory contains other samples for demonstrating
advanced sensors like PDR, Swim, AI, Air Quality etc. To build one of these other samples,
it is necessary to pass a build qualifier in the build command line. Check out CMakeList.txt
for details. e.g. to build the PDR sample, one has to pass -DBUILD_PDR in the build command line 
Or, if using VSCode with nrf Connect plugin, by entering -DBUILD_PDR:STRING="1" in 
the field "Extra CMake Argument" in the Build Configuration dialog.

These are the qualifiers required for each sample:
+---------------+------------------+-----------------------------------------------+
| Sample Source	| Qualifiers       | Remarks                                       |
+===============+==================+===============================================+
| main.C 		| no qualifiers    | Default Sample (Onboard Sensors)              |
+---------------+------------------+-----------------------------------------------+
| main_aux.c	| -DBUILD_AUX	   | Auxiliary Sensors Sample                      |
+---------------+------------------+-----------------------------------------------+
| main_bsec.c	| -DBUILD_BSEC 	   | Air Quality (BSEC) Sensor Sample              |
+---------------+------------------+-----------------------------------------------+
| main_klio.c	| -DBUILD_KLIO     | AI Self-Learning (Klio) Sensor Sample         |  
+---------------+------------------+-----------------------------------------------+
| main_swim.c	| -DBUILD_SWIM     | Swim Sensor Sample                            |
+---------------+------------------+-----------------------------------------------+
| main_pdr.c 	| -DBUILD_PDR      | Pedestrian Dead Reckoning (PDR) Sensor Sample | 
+---------------+------------------+-----------------------------------------------+
| main_wake.c	| -DBUILD_WAKE     | Wakeup Features Sample                        |
+---------------+------------------+-----------------------------------------------+
| main_custom.c |      x		   | Custom Virtual Sensor Sample (for info only)  |
+---------------+------------------+-----------------------------------------------+


In this example below the :ref:`arduino_nicla_sense_me` board is used.


.. zephyr-app-commands::
   :zephyr-app: samples/sensor/bhix60
   :board: arduino_nicla_sense_me
   :goals: build flash

Sample Output (For Default Sample)
=================================

.. code-block:: console
Game Rotation: x: -3490, y: -9100, z: -12293, w: 4724; acc: 0; AT s:13 ns:235406250
AX: 11205.000000; AY: 5813.000000; AZ: 17974.000000; GX: 3520.000000; GY: 6287.000000; GZ: 927.000000;
Game Rotation: x: 5193, y: -15427, z: -1586, w: 967; acc: 0; AT s:14 ns:238500000
Tilt Detected!
AX: 14288.000000; AY: 5182.000000; AZ: -23371.000000; GX: -2164.000000; GY: -443.000000; GZ: 1273.000000;
Game Rotation: x: 2492, y: -12171, z: -8572, w: 6371; acc: 0; AT s:15 ns:241531250

   <repeats endlessly>

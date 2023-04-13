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

These samples are tested on :

1. The Arduino Nicla Sense ME board, which has an integrated BHI260AP sensor 
connected via SPI to a host nRF52832, with BMP390 (pressure sensor), BMM150 (3-axis magnetometer) 
and BME688 (environment sensor) as auxiliary sensors connected via i2c and SPI to the BHI260. 
NOTE: The actual board definition to be used for building the samples is bst_ardunio_nicla
which is a part of this repository, and not the arduino_nicla_sense_me board definition
which is in the upstream Zephyr repository. The board definition bst_ardunio_nicla is pre-configured
with Bosch Sensortec drivers that are available in this repository.
 
2. The Bosch Application Board 3.0 with a BHI260AP shuttle board mounted on it.
This shuttle board has a BHI260AP sensor that connects via SPI or I2C to the host nRF52832
on the main board. Additionally, it has a BMP390, BMM150 and BME688 as auxiliary sensors 
connected via i2c and SPI to the BHI260, as with the Nicla board. 

However the connections details between BHI260 and the peripheral devices vary between the 
shuttle board and Nicla, hence, in order to use the peripheral devices, it is
necessary to use two different sets of firmware for the two boards. 
This can be seen in the AUX and BSEC samples.

These samples can be adapted to any other board with a similar configuration to either Nicla
or AB3.

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
| Sample Source |  Qualifiers      |      Remarks                                  |
+===============+==================+===============================================+
| main.C        | no qualifiers    | Default Sample (Onboard Sensors)              |
+---------------+------------------+-----------------------------------------------+
| main_aux.c    | -DBUILD_AUX      | Auxiliary Sensors Sample                      |
+---------------+------------------+-----------------------------------------------+
| main_bsec.c   | -DBUILD_BSEC     | Air Quality (BSEC) Sensor Sample              |
+---------------+------------------+-----------------------------------------------+
| main_klio.c   | -DBUILD_KLIO     | AI Self-Learning (Klio) Sensor Sample         |
+---------------+------------------+-----------------------------------------------+
| main_swim.c   | -DBUILD_SWIM     | Swim Sensor Sample                            |
+---------------+------------------+-----------------------------------------------+
| main_pdr.c    | -DBUILD_PDR      | Pedestrian Dead Reckoning (PDR) Sensor Sample |
+---------------+------------------+-----------------------------------------------+
| main_wake.c   | -DBUILD_WAKE     | Wakeup Features Sample                        |
+---------------+------------------+-----------------------------------------------+
| main_bhi3.c   | -DBUILD_BHI3     | BHI3 Sample                                   |
+---------------+------------------+-----------------------------------------------+
| main_custom.c |      x           | Custom Virtual Sensor Sample (for info only)  |
+---------------+------------------+-----------------------------------------------+

Additionally, in order to build the samples for the advanced sensors like SWIM, PDR, etc.
the corresponding options below needs to be activated in prj.conf 

+---------------+----------------------------+
| Sample Source |  Config option             |
+===============+============================+
| main_bsec.c   | CONFIG_BOSCH_BHY2_BSEC=y   |
+---------------+----------------------------+
| main_klio.c   | CONFIG_BOSCH_BHY2_KLIO=y   |
+---------------+----------------------------+
| main_swim.c   | CONFIG_BOSCH_BHY2_SWIM=y   |
+---------------+----------------------------+
| main_pdr.c    | CONFIG_BOSCH_BHY2_PDR=y    |
+---------------+----------------------------+
| main_wake.c   | CONFIG_PM=y                |   
|               | CONFIG_PM_DEVICE=y         |
|               | CONFIG_PM_DEVICE_RUNTIME=y |
+---------------+----------------------------+
| main_bhi3.c   | CONFIG_BOSCH_BHY2_BHI3=y   |
+---------------+----------------------------+

Most of the samples use BHI260 firmware that are designed for uploading to BHI260 RAM
, so the following setting should be made in *prj.conf*:

	CONFIG_BHIX60_UPLOAD_FW_TO_RAM=y
	
	CONFIG_BHIX60_UPLOAD_FW_TO_FLASH=n

However, if using the Nicla board, the AUX and BSEC samples use a Nicla-specific firmware
that is designed for uploading to BHI260 Flash. So, *only* if using the Nicla board, and 
that too *only* for AUX and BSEC samples, following setting should be done in *prj.conf*

	CONFIG_BHIX60_UPLOAD_FW_TO_RAM=n
	
	CONFIG_BHIX60_UPLOAD_FW_TO_FLASH=y

This is verified by #ifdef pre-compiler checks in the code, to prevent inadvertent errors.

Build instructions for :ref:`bst_arduino_nicla` board.


.. zephyr-app-commands::
   :zephyr-app: samples/sensor/bhix60
   :board: bst_arduino_nicla
   :goals: build flash

Build instructions for :ref:`bst_ab3_nrf52840` board.


.. zephyr-app-commands::
   :zephyr-app: samples/sensor/bhix60
   :board: bst_ab3_nrf52840
   :goals: build flash

Sample Output
*************

For Default Sample
==================
This BHix60 driver sample demonstrates the use of virtual sensors that use
the onboard hardware sensors, i.e. the ones that do not require additional 
slave sensors.

Specifically: Accelerometer, Gyroscope,Tilt Detector, Game Rotation Vector
It demonstrates accessing data using the Sensor API as well as the extended API
(for Game Rotation Vector) as well as the setting of scales and 
data-ready triggers, threshold triggers and event triggers 

.. code-block:: console

	Game Rotation: x: -3490, y: -9100, z: -12293, w: 4724; acc: 0; AT s:13 ns:235406250
	AX: 11205.000000; AY: 5813.000000; AZ: 17974.000000; GX: 3520.000000; GY: 6287.000000; GZ: 927.000000;
	Game Rotation: x: 5193, y: -15427, z: -1586, w: 967; acc: 0; AT s:14 ns:238500000
	Tilt Detected!
	AX: 14288.000000; AY: 5182.000000; AZ: -23371.000000; GX: -2164.000000; GY: -443.000000; GZ: 1273.000000;
	Game Rotation: x: 2492, y: -12171, z: -8572, w: 6371; acc: 0; AT s:15 ns:241531250

   <repeats endlessly>

For Aux Sample
==============
This BHIx60 driver sample demonstrates reading the virtual sensors that require 
auxiliary BME688, BMM150, BMP390 devices to be connected to BHIx60
This sample also demonstrates setting threshold triggers. It sets a 
threshold trigger for orientation sensor (triggered if Heading, Pitch or Roll
exceeds +/- 150 degrees ).
   
.. code-block:: console

	T=19.590000 C, GR=0.000000 Ohm, P=100.147656 kPa, H=57.000000 pH
	Magnetometer X=-0.402844 Gs, Y=-0.215918 Gs, Z=-0.593585 Gs
	Orientation H=0.340576 d, P=-0.109863 d, R=-23.708496 d
	
   <repeats endlessly>

For BSEC Sample
===============
This BHIx60 driver sample demonstrates using the BSEC virtual sensor

.. code-block:: console

	BSEC at: 7.406578125 iaq=25, iaqs=25, acc=0, bvoc=0.490000ppm, co2=500.000000ppm,  temp=20.976500C, hum=52.000000pH gas=6.640659 Ohm
	BSEC at: 10.408625000 iaq=25, iaqs=25, acc=0, bvoc=0.490000ppm, co2=500.000000ppm,  temp=21.374976C, hum=51.060000pH gas=6.048913 Ohm


   <repeats endlessly>

For KLIO Sample
===============
This BHIx60 driver sample demonstrates using the KLIO Self-Learning virtual sensor. Move the board
in a regular pattern to trigger learning and recognition of physical movements. The following output
is for waving the board back-and-forth in an arc. The output will be different in each case depending
on nature of movement.

.. code-block:: console

	KLIO: T: 22.560640625; Learning [Id:-1 Progress:0 Change:0]; Recognition[Id:255 Count:0]
	KLIO: T: 26.137656250; Learning [Id:-1 Progress:20 Change:0]; Recognition[Id:255 Count:0]
	KLIO: T: 26.535140625; Learning [Id:-1 Progress:40 Change:0]; Recognition[Id:255 Count:0]
	KLIO: T: 26.932609375; Learning [Id:-1 Progress:60 Change:0]; Recognition[Id:255 Count:0]
	KLIO: T: 27.330062500; Learning [Id:-1 Progress:80 Change:0]; Recognition[Id:255 Count:0]
	KLIO: T: 27.727531250; Learning [Id:6 Progress:100 Change:0]; Recognition[Id:255 Count:0]
	KLIO status: 0 (bhix60_klio_read_pattern)
	KLIO T: 27.727531250; PATTERN LEARNT: 52423106036db304410ad7233c5a4d10c1d1da9dc0651c6fbf8182cf3fd0af44c04332403f1ec751bffcf173c1753e0f41b2115fbf693b903e66ba953f34eba93f85be0dc1500ee540203f64bd4b9b6bbfa58e2b3fccfce23e1ada9c3d37d13fbf2b1969bebe91abbdc4118d3c025e4b3d43501540d16db240db03e8bec8f302bf80ae2f3e0b8c1dbedf2727c04c2b17c15b78033ee37c02bf7c4ba13e
	KLIO status: 0 (bhix60_klio_write_pattern)

	KLIO status: 0 (bhix60_klio_similarity_score_multiple)
	KLIO T: 27.727531250; SIMILARITY SCORE TO ALREADY STORED PATTERNS: 0: 0 <\r>
	KLIO status: 0 (bhix60_klio_set_pattern_states)
	KLIO: T: 28.601953125; Learning [Id:-1 Progress:100 Change:0]; Recognition[Id:1 Count:0]
	KLIO: T: 29.595515625; Learning [Id:-1 Progress:100 Change:0]; Recognition[Id:1 Count:1]
	KLIO: T: 30.231390625; Learning [Id:-1 Progress:100 Change:0]; Recognition[Id:1 Count:2]
	KLIO: T: 31.105796875; Learning [Id:-1 Progress:100 Change:0]; Recognition[Id:1 Count:3]


   <repeats endlessly>

For SWIM Sample
===============
This BHIx60 driver sample demonstrates using the SWIM virtual sensor. Move
the board in a regular motion resembling a swimmer's hand movements to simulate
swim data.

.. code-block:: console

	Swim Data at: 19.806187500; 0, 0, 0, 0, 0, 0, 0
	Swim Data at: 20.809921875; 0, 0, 0, 0, 0, 0, 0
	Swim Data at: 21.813625000; 0, 0, 0, 0, 0, 0, 0
	Swim Data at: 22.817296875; 0, 0, 0, 0, 0, 0, 0
	Swim Data at: 23.820953125; 0, 0, 0, 0, 0, 0, 0
	Swim Data at: 24.824625000; 0, 0, 0, 0, 0, 0, 0
	Swim Data at: 25.828328125; 0, 0, 0, 0, 0, 0, 1
	Swim Data at: 26.832000000; 0, 0, 0, 0, 0, 0, 1

   <repeats endlessly>

For PDR Sample
===============
This BHIx60 driver sample demonstrates using the PDR virtual sensor. Move the board in various
directions to trigger PDR data. Following output is for a particular series of movements. It
will be different in each case.

.. code-block:: console

	PDR at 3.576656250  x=0.000000 y=0.000000 hac=0.000000 hd=0.000000 hdac=0.000000 scnt=0 fr=1 tr=0
	PDR at 10.385031250  x=0.000000 y=0.000000 hac=0.200000 hd=0.000000 hdac=0.000000 scnt=0 fr=0 tr=0
	PDR at 11.279500000  x=0.000000 y=0.000000 hac=0.400000 hd=286.300000 hdac=0.000000 scnt=1 fr=0 tr=0
	PDR at 11.955296875  x=0.000000 y=0.000000 hac=0.200000 hd=264.700000 hdac=0.000000 scnt=2 fr=0 tr=0
	PDR at 12.551515625  x=-1.900000 y=0.000000 hac=0.200000 hd=292.900000 hdac=0.000000 scnt=3 fr=0 tr=0
	PDR at 13.088203125  x=-1.900000 y=0.000000 hac=0.200000 hd=288.200000 hdac=0.000000 scnt=4 fr=0 tr=0
	PDR at 14.082093750  x=-1.800000 y=0.000000 hac=0.200000 hd=349.200000 hdac=0.000000 scnt=5 fr=0 tr=0
	PDR at 15.175343750  x=-1.800000 y=0.000000 hac=0.200000 hd=349.200000 hdac=0.000000 scnt=5 fr=0 tr=0
	PDR at 15.672218750  x=-1.800000 y=0.000000 hac=0.400000 hd=349.200000 hdac=0.000000 scnt=5 fr=0 tr=0
	PDR at 16.169125000  x=-1.800000 y=0.000000 hac=0.600000 hd=349.200000 hdac=0.000000 scnt=5 fr=0 tr=0
	PDR at 16.864796875  x=-1.800000 y=0.000000 hac=0.800000 hd=349.200000 hdac=0.000000 scnt=5 fr=0 tr=0
	PDR at 17.659843750  x=-1.800000 y=0.000000 hac=1.000000 hd=349.200000 hdac=0.000000 scnt=5 fr=0 tr=0
	PDR at 18.276031250  x=-1.800000 y=0.100000 hac=1.200000 hd=0.500000 hdac=0.000000 scnt=6 fr=0 tr=0
	PDR at 18.594046875  x=-1.900000 y=0.100000 hac=0.200000 hd=4.400000 hdac=0.000000 scnt=7 fr=0 tr=0
	PDR at 19.289703125  x=-1.900000 y=0.100000 hac=0.200000 hd=4.400000 hdac=0.000000 scnt=7 fr=0 tr=0
	PDR at 19.786625000  x=-1.900000 y=0.100000 hac=0.400000 hd=147.100000 hdac=0.000000 scnt=8 fr=0 tr=0
	PDR at 20.283515625  x=-1.900000 y=0.100000 hac=0.200000 hd=206.000000 hdac=0.000000 scnt=9 fr=0 tr=0
	PDR at 20.820218750  x=-1.900000 y=0.100000 hac=0.200000 hd=206.000000 hdac=0.000000 scnt=9 fr=0 tr=0
	PDR at 21.396625000  x=-1.900000 y=0.100000 hac=0.400000 hd=206.000000 hdac=0.000000 scnt=9 fr=0 tr=0
	PDR at 21.893546875  x=-1.900000 y=0.100000 hac=0.600000 hd=206.000000 hdac=0.000000 scnt=9 fr=0 tr=0
	PDR at 22.768046875  x=-1.900000 y=0.100000 hac=0.800000 hd=206.000000 hdac=0.000000 scnt=9 fr=0 tr=0
	PDR at 23.443781250  x=-1.900000 y=0.100000 hac=1.000000 hd=206.000000 hdac=0.000000 scnt=9 fr=0 tr=0
	PDR at 23.940656250  x=-1.800000 y=0.000000 hac=1.200000 hd=215.800000 hdac=0.000000 scnt=10 fr=0 tr=0
	PDR at 24.556750000  x=-1.800000 y=0.000000 hac=0.200000 hd=215.800000 hdac=0.000000 scnt=10 fr=0 tr=0
	PDR at 25.053625000  x=-1.800000 y=0.000000 hac=0.400000 hd=215.800000 hdac=0.000000 scnt=10 fr=0 tr=0
	PDR at 25.828734375  x=-1.800000 y=0.000000 hac=0.600000 hd=215.800000 hdac=0.000000 scnt=10 fr=0 tr=0


   <repeats endlessly>

For WAKE Sample
===============
This BHIx60 driver sample demonstrates the use of Wakeup type Virtual
Sensors in conjunction with the Zephyr Power Management Module. It uses 
the motion and stationary sensors to detect motion and no motion. If 
the board is physically stationary for longer than 10 seconds the host
CPU goes into Soft Off state. It resumes when board is moved again.

Accelerometer reading is dumped in active state to simulate normal
functioning of the application

This sample will only work on a Nordic nrf52 platform.  There are certain
operations that cannot currently be written in a platform-independent manner.
Refer comments in the source code for details.

This sample is based on the nrf52 sample `system_off`_ *zephyr/samples/board/nrf/system_off*
When resuming after a Soft Off, the application restarts from the beginning
and all data stored in RAM is lost. The above Nordic sample demonstrates a 
way to retain critical data on RAM between restarts, which can be used in a 
real-life application (not implemented in this sample). 

.. code-block:: console

	bhix60: Boot status (preboot)
	bhix60: Boot status (postboot):31
	bhix60: Kernel version 5991
	X=-9 Y=0 Z=2
	X=-9 Y=0 Z=2
	X=-9 Y=0 Z=2
	X=-9 Y=0 Z=2
	Board is stationary!
	X=-9 Y=0 Z=2
	X=-9 Y=0 Z=2
	X=-9 Y=0 Z=2
	X=-9 Y=0 Z=2
	Entering standby mode
	bhix60: Boot status (preboot)
	bhix60: Boot status (postboot):31
	bhix60: Kernel version 5991
	
Note: After the message, "Entering standby mode", there is no further output until
the board is physically moved. The above output has been simplified to remove 
timestamps and other boot messages.

For BHI3 Sample
===============
This BHIx60 driver sample demonstrates using the BHI360 specific features.

**To be defined**





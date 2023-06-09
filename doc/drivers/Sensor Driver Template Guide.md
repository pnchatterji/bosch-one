# Bosch Sensortec Sensor Driver Template Guide

This document explains how to use the sensor_driver_template project in the samples/sensor
folder of Bosch One in order to create a Zephyr sensor driver for a Bosch Sensortec sensor
and install it as a part of Bosch One and/or the upstream Zephyr repository.

The directory structure is like this:

		 sensor_driver_template
			driver
			sample
				boards
			dts
				bindings 	

The driver sub-directory contains a template for the driver. Once finalized,
its *contents* will go into the *<repo-root>/drivers/sensor/bxx*

The sample sub-directory contains the sample project for the driver . Once finalized,
its *contents* will go into the *<repo-root>/samples/sensor/bxx* directory

The dts/bindings directory contains the binding file for the driver.
The *binding file*, once finalized, will go into *<repo-root>/dts/bindings/sensor*

Here, *repo-root* stands for the root of the repository to which the driver is being installed. 
This could be the Bosch One repository or the upstream Zephyr repository. 

The other files in the root of the template directory (*Kconfig* and *CMakeLists.txt*),
are only there so that the test project can be built to test the driver prior to
deployment. The files in the template root directory will *not* be deployed.

The test project is basically just the driver and sample code, built into a
single app in order to test the driver before deployment to the Zephyr or Bosch One tree 

**NOTE1:** the template is actually a fully functional (although minimal) driver which 
can be built and run without changes (it uses BMI270 as an example, but with just 
bare minimum support). After building, the template can be flashed and run
on a Bosch Application Board 3.0 with a BMI270 shuttle board. The example
board DTS overlay in *sample/boards* directory is an overlay of the AB3 DTS
(see point 9 below)

**NOTE2:** The driver supports both I2C and SPI. If the BXX driver is a sub-node
of an I2C node in the DTS overlay, it uses I2C, and the same for SPI. SPI
has precedence. Refer the example overlay in sample/boards for further details.
This is useful for devices that support both SPI and I2C, e.g. BMI270
If the BXX device supports only one of these protocols, the driver can be 
modified to remove all code and settings pertaining to the unused protocol,
to avoid confusion. There is a separate note at the end of this document that
explains how to do this.

**NOTE3:** Read the important comments regarding definition of SDO, CS and interrupt
pins for I2C in the DTS overlay file and in  *dts\bindings\bosch,bxx-i2c.yaml*

**NOTE4:** The template supports one hardware interrupt for triggers. If the BXX
device does not have a hardware interrupt, or if this feature is not required,
all trigger-related code can be removed from *bxx.c* to avoid confusion
Basically, all the code protected by *#ifdef CONFIG_BXX_TRIGGER*
The *CONFIG_BXX_TRIGGER* config symbol itself should also be removed from
*driver/Kconfig*, and *hirg-gpios* from binding and overlay files 

## References

Zephyr Driver model:
https://docs.zephyrproject.org/latest/kernel/drivers/index.html

Zephyr Sensor API
https://docs.zephyrproject.org/latest/hardware/peripherals/sensor.html

## CREATING THE DRIVER

1. In **VS Code**, using the **nrf Connect** plugin, create a new application. Use the sample
*bosch-one/samples/sensor/sensor_drive_template* as an application template. Name the 
project *bxx-driver-proj* or something similar.

2. Replace *bxx* and *BXX* in the filenames and in the code, comments and readme text 
within each file with the sensor name e.g. bmi270. 
E.g. *bxx.c* should be renamed *bmi270.c*, and the variable *bxx_data* in the code 
should be renamed *bmi270_data* etc. etc.

	This should be done for *all* the files in *all* the sub-directories
Please write the device name in small letters or caps as appropriate (e.g. *bxx* should
be replaced with *bmi270* and *BXX* with *BMI270*). You can open all the files in
Notepad++ and perform "search/replace in all opened files".

3. Read through all the comments marked *"TIP:"* in all the files before you start. 
These comments tell you what you need to do for each of these files.

4. If there is an existing COINES version of the driver, read the section below 
entitled *Adapting an Existing COINES Driver*. 

5. Implement functions for initializing and controlling the bxx device in *driver/bxx.c*
at the location marked *"TIP: BXX internal functions"*. If you need to add additional 
source files, you will need to modify *driver/CMakeLists.txt* and add the additional 
C files like this: 

	`zephyr_library_sources(bxx.c bxx_foo.c bxx_bar.c) `

6. To access the bxx device registers vis SPI or I2C, call *reg_read()* and *reg_write()*
which are pre-defined in bxx.c

7. Modify the Zephyr API functions like *bxx_init()*, *bxx_channel_get()* etc. which are 
pre-defined in bxx.c by calling the initialization and control functions that you have 
created in the previous step.

8. Control functions that have no relevance for the Zephyr API functions, but still need 
to be called by the application to access advanced device features, can be exposed to application
code in a "Raw API" that is declared in the *bxx.h* header file. 

	Note that, in this case, the *bxx.h* should be placed in *zephyr/include* so that it is
visible to the application. This is explaned in the next section *Deployment to Zephyr*

	This should be done only if essential. Only a handful of Zephyr sensor drivers export 
a raw API. Most are accessed exclusively through the Zephyr Sensor API, and have no public header file. 

9. The initialization function *bxx_init()* should be modified to add any additional device specific 
initialization that may be required. It may be necessary to define additional DTS initialization 
parameters to pass to the bxx device.
To add an init parameter, it has to be added to: 
	 1. The binding yaml files *dts/binding/bxx-i2c/bosch,bxx-???.yaml*
	 2. The sample DTS overlay files *sample/board/<board>.overlay*
	 3. In the device config structure *bxx_config*
	 4. In bxx.c in the macro *BXX_INIT*

		This is further clarified in the below steps. 

10. The bindings file *dts/binding/bosch,bxx.yaml* (to be renamed to the device name e.g. *bosch,bmi270.yaml* ) should
be modified to include any additional device configuration parameters required.

11. Overlay files should be created in the *sample/boards* directory for all the boards to be
supported in the sample. The overlay files should at a minimum extend the spi and/or
i2c nodes to add the bxx driver node, along with any configuration parameters defined above.
The existing overlay file in the template *sample/boards* directory is an example of how
the overlay can be written (the example is an overlay of the **Application Board 3.0 DTS**).

12. The device usage sample (*sample/src/main.c*) should be updated to provide a simple sample of
how the driver is to be used.

13. Add any additional build configuration parameters that are required in *driver/Kconfig*
(the existing configuration parameters (*config BXX*) inside this file should *also* be renamed appropriately) 

14. *sample/prj.cfg* should be modified if necessary for any additional build configuration parameters that
are added above, or standard Zephyr config variables that are needed by the driver

15. The file *driver/CMakelist.txt* should be modified appropriately.
At the least, the following line should be modified to reflect the new driver C file name:

	`zephyr_library_sources(bxx.c) `

	If more source files have been added to driver, they should also be added
 
16. *sample/readme.rst* should be updated

17. Cleanup the files: remove all "*TIP:*" comments from all the files. Use Notepad++
to search for "TIP". Since these files will be posted on a public forum, it is essential
that they look professional.

     You can use the following regular expression to automatically search/replace all the 
 TIP comments from all open files using Notepad++:

    ` (?:/\*TIP(?:(?:[^*]|\*(?!/))*)\*/) `
 
     **Note:** Remember to click on the regex radio button in the Search/Replace dialog box
in Notepad++ to use this regex. This regex will not work for the Tips in the Cmake files 
and Kconfig files with "#" style comments. Those files have to be edited by hand.

18. In the *nrf Connect* plugin, create a build configuration for a supported Bosch Sensortec
board and build and test the project on the target board.

### ADAPTING AN EXISTING BOSCH SENSORTEC SENSOR API
If there is an existing Bosch Sensortec Sensor API for the sensor, it is possible to implement
the Zephyr driver as a thin layer on top of the Sensor API, rather than writing all of the 
sensor control logic from scratch.

The Sensor APIs are written as abstract control logic. Access to MCU hardware for things like 
SPI, i2c, GPIO, timers etc. is done through a *sensor-specific* Hardware Abstraction Layer, so that
the code is easily portable to different platforms. The Sensor APIs are usually packaged with an
example implementation of the HAL using COINES. For Zephyr drivers, the HAL implimentation has to be 
done using appropriate Zephyr API calls. When the implementation is complete, this is what
the architecture will resemble:
<style>
table,  tl, tr {
   border: 2px solid black;
}
th {
        display: none;
    }
</style>

|                     |
|:-------------------:|
|     Application     |
|Zephyr Sensor Driver |
|     Sensor API      |
|     Sensor HAL      |
|    Zephyr RTOS      |
|     MCU Hardware    |

The Bosch Sensortec Sensor APIs are usually organized as follows:

		 bxx					//Sensor API root directory
		 	bxx.c				//core sensor control logic
			bxx.h				//Sensor API declaration
			bxx_foo.c			//internal implementation files
			bxx_bar.c
		 	examples			//examples subdirectory
				common			//common files for all examples
					common.c	//example implementation of Sensor HAL using COINES
					common.h
				example1		//one or more example apps
					main.C
				example2
					main.C

1. Add the Sensor API to Bosch One repository in the *sensor-api* folder, if not already available. 
This can be done by either physically copying the files to a new *sensor-api* sub-folder, or by 
adding a link to the Sensor API GitHub repository in *bosch-one.yaml* in the Bosch One root folder
 
2. Alternatively, if the driver is being written for deployment to upstream Zephyr, it is also
possible to copy the Sensor API files to the Zephyr driver folder ( *bxx_driver_impl/driver* ). 

	1. Needless to say, *bxx/bxx.c* and *bxx/bxx.h* should be renamed *bxx_impl.c* and 
	*bxx_impl.h*, or something similar, to avoid conflict with the files having the same 
	name in the Zephyr driver directory.

	2. Modify *driver/CMakeLists.txt* to add the additional C files like this: 

		`zephyr_library_sources(bxx_impl.c bxx_foo.c bxx_bar.c) `

	3. The examples directory should *not* be copied to the Zephyr driver, although one or more
	of the examples can be used as the basis for the sample in *bxx_driver_impl/sample*

3. The functions defined in *examples/common/common.c* and *common.h* act as a sensor-specific 
hardware abstraction layer implemented on COINES. That is, they contain functions that are called 
by the core driver logic whenever there is a need to access hardware features like SPI, I2C, 
GPIO, Timers or Interrupts. These files should be re-implemented in *bxx_driver_impl/bxx.c*
by calling the *reg_read()* and *reg_write()* functions which are pre-defined in bxx.c for
accessing SPI and I2C, and with direct Zephyr API calls for other things (e.g. for sleep). 

4. The Zephyr API functions like *bxx_init(), bxx_channel_get()* etc. in *bxx_driver_impl/bxx.c* 
should be modified to call the appropriate device initialization and control functions
in *bxx_impl.c* .

5. Device control functions in *bxx_impl.c* that have no relevance for the Zephyr API functions,
but still need to be called by the application to access advanced device features, can be 
declared in *bxx_driver_impl/bxx.h* and made a part of the Raw API of the driver, as discussed
in the previous section.

## DEPLOYMENT TO BOSCH ONE

**NOTE: bxx is a stand-in for the actual driver name (eg bmi270) in all of the instructions below**

1. Create a directory for the driver *bosch-one/drivers/sensor/bxx*

2. Copy the *contents* of the driver sub-directory to *bosch-one/drivers/sensor/bxx*

3. It is assumed that the corresponding Sensor API has already been installed in Bosch One as 
described in the previous section

4. Modify *bosch-one/drivers/sensor/Kconfig* to add the line

    `source "drivers/sensor/bxx/Kconfig"`

5. Modify *bosch-one/drivers/sensor/CMakeLists.txt* to add the line:

	`add_subdirectory_ifdef(CONFIG_BXX	bxx)`

	**NOTE:** *CONFIG_BXX* is the config variable that you would have created in *src/driver/Kconfig* in **Step 11**
	in the previous section.

6. Copy *dts/bindings/bosch,bxx-???.yaml* to *bosch-one/dts/bindings/sensor*

7. Create a directory for the driver sample in *bosch-one/samples/sensor/bxx* 

8. Copy the *contents* of the sample sub-directory to *bosch-one/samples/sensor/bxx* 

9. Build and test the bxx sample in *VS Code* in the *nrf Connect plugin*, as described in the Bosch One
installation guide and various other driver samples

10. Contact the Bosch-One repository administrator to push the changes to the Bosch-One Github repository.

## DEPLOYMENT TO UPSTREAM ZEPHYR

Read the contribution guidelines
https://docs.zephyrproject.org/latest/contribute/index.html

**NOTE: bxx is a stand-in for the actual driver name (eg bmi270) in all of the instructions below**

1. Make a fork of Zephyr

2. Create a directory for the driver *Zephyr/drivers/sensor/bxx*

3. Copy the *contents* of the driver sub-directory to *Zephyr/drivers/sensor/bxx*

	If the driver exports a raw API, the *bxx.h* file should be copied to 
*zephyr/include/zephyr/drivers/sensor* so that the API is visible to Zephyr
applications using `#include <bxx.h>` . As mentioned previously, this should be done
only if essential. Very few sensor drivers export a raw API.

4. Modify *zephyr/drivers/sensor/Kconfig* to add the line

    `source "drivers/sensor/bxx/Kconfig"`

5. Modify *zephyr/drivers/sensor/CMakeLists.txt* to add the line:

	`add_subdirectory_ifdef(CONFIG_BXX	bxx)`

	**NOTE:** *CONFIG_BXX* is the config variable that you would have created in *src/driver/Kconfig* in **Step 11**
	in the previous section.

6. Copy *dts/bindings/bosch,bxx-???.yaml* to *zephyr/dts/bindings/sensor*

7. Create a directory for the driver sample in *zephyr/samples/sensor/bxx* 

8. Copy the *contents* of the sample sub-directory to *zephyr/samples/sensor/bxx* 

9. Build and test the bxx sample in the Zephyr tree as described for the Blinky in

	https://docs.zephyrproject.org/latest/develop/getting_started/index.html

	This is to ensure that the driver and sample is deployed successfully in the Zephyr tree
	and will work for any user.

10. Push the changes to Zephyr and do a pull request as described in the contribution guidelines

## Converting to Single-Protocol driver

The driver supports both I2C and SPI, as described in the introduction.
This is useful for devices that support both SPI and I2C.
If the BXX device supports only one of these protocols, the driver can be 
modified to remove all code and settings pertaining to the unused protocol,
to avoid confusion. 

This can be done as follows (e.g. to remove I2C):

- In bxx.c, delete *reg_read_i2c(),reg_write_i2c(), reg_read(), reg_write()* . 
- Rename *reg_read_spi(),reg_write_spi()* to *reg_read(), reg_write()* .
- Remove all I2C parameters from *struct bxx_cfg* and from the *BXX_INIT()* macro
- Remove the binding file *dts/binding/bosch,bxx-i2c.yaml*
- Remove all references to I2C from *sample/prj.cfg*, overlay files in *sample/boards*
and *driver/Kconfig*

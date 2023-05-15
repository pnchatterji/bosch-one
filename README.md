# Bosch One
## Zephyr External Module for Bosch Sensortec GmbH.

Bosch One is the Bosch Sensortec SDK for Zephyr. It contains the Zephyr-related Bosch Sensortec software infrastructure for Bosch Sensortec sensor devices and application boards. This includes board definitions, sensor drivers, other device drivers, filesystems, libraries including COINES for Zephyr (Z-COINES) and samples and user guides.

Bosch One is integrated into a Zephyr and Nordic *nrf Connect SDK* installation as an external module. It is located in a directory named *bosch-one* in the Zephyr root directory.

Bosch One can be added to an existing *nrf Connect SDK* / Zephyr installation, or it can be installed as the manifest repository, together with all dependencies.

The procedure for installing Bosch One is explained here: [Bosch One Installation Guide](<./doc/Bosch One Installation Guide.md>)

Bosch One supports two kinds of sensor applications: 
1. Apps that use the native Zephyr sensor drivers available in the *drivers* folder
2. Z-COINES based apps that directly call the sensor api available in the *sensor-api* folder, using Z-COINES for hardware interface.

This is the folder organization for the Bosch One Repository:

Folder          | Description
--------------  | --------------------------------------------------------------- 
**boards**      | Zephyr board definitions for Bosch Sensortec application boards
**dts**         | Zephyr binding files required for board definitions and device drivers
**doc**         | User guides for device drivers and COINES
**drivers**     | Native Zephyr device drivers for Bosch Sensortec sensors; other device drivers required by the application boards.
**lib**         | Bosch Sensortec libraries, including COINES for Zephyr (Z-COINES)
**samples**     | Sample Zephyr applications using Bosch Sensortec device drivers, Z-COINES, and board definitions
**sensor-api**  | Bosch Sensortec Sensor APIs. These are used both by Z-COINES based apps, and indirectly by the native sensor drivers
**subsys**      | Embedded filesystems supported by Bosch One, including FLog and others
**zephyr**      | Administration files required for integrating Bosch One into Zephyr. This is not the main Zephyr directory, which is in the parent directory of Bosch One.  

### User Guides
User guides for Z-COINES and sensor device drivers are available in the *doc* folder.

[Z-COINES User Guide](<./doc/COINES/COINES for Zephyr User Guide.md>)

[BHIx60 Driver User Guide](<./doc/bhix60/BHIX60 Zephyr Driver User Guide.md>)

### Samples
Samples for Z-COINES and sensor device drivers are available  in the *samples* folder.

[BHIx60 Sample App](<./samples/sensor/bhix60/README.rst>)
This is a master sample for BHIx60 native Zephyr Driver. It contains a number of sub-samples to demonstrate different sets of virtual sensors which can be individually built by passing the appropriate build arguments.

[Z-COINES Sample App](<./samples/coines/coines_app/README.rst>)
This is a sample for creating a Zephyr application using the Z-COINES library. It contains several test functions for demonstrating use of different parts of the COINES API.

[Z-COINES BMI270 Accel Sample](<./samples/coines/bmi270/accel/README.rst>)
This Z-COINES sample demonstrates building a standard COINES example for a sensor on Zephyr. It uses the sources of the "accel" COINES example of the BMI270 sensor, which is included as a part of the BMI270 Sensor API. Other examples for this and other sensors can be created on the same lines, by copying the necessary example sources from the desired example folder into the app project folder.

[Z-COINES Generic Sensor Sample](<./samples/coines/sensor_app/README.rst>)
This is an alternative approach to building a standard COINES example for a sensor on Zephyr. It can build any standard example of any sensor in the sensor-api folder, by passing the sensor name and example name as build arguments. In this case, the example sources do not need to be copied into the app project folder.

[Z-COINES Sensor Script](<./samples/coines/sensor_script/README.rst>)
This sample contains a batch script for building and running *all* the standard examples of a sensor in the sensor-api folder, and logging the results in a log file. It is based on the generic sensor sample described above. It can be used as a template for automated testing scripts for user applications based on Z-COINES.

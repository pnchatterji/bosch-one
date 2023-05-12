# Bosch One
## Zephyr External Module for Bosch Sensortec GmbH.

Bosch One is the Bosch Sensortec SDK for Zephyr. It contains the Zephyr-related Bosch Sensortec software infrastructure for Bosch Sensortec sensor devices and application boards. This includes board definitions, sensor drivers, other device drivers, filesystems, libraries including COINES for Zephyr (Z-COINES) and samples and user guides.

Bosch One is integrated into a Zephyr and Nordic *nrf Connect SDK* installation as an external module. It is located in a directory named *bosch-one* in the Zephyr root directory.

Bosch One can be added to an existing *nrf Connect SDK* / Zephyr installation, or it can be installed as the manifest repository, together with all dependencies.

The procedure for installing Bosch One is explained here: [Bosch One Installation Guide](<./doc/Bosch One Installation Guide.md>)

Bosch One supports two kinds of sensor applications: 
1. Apps that use the native Zephyr sensor drivers available in the *drivers* folder
2. Z-COINES based apps that directly call the sensor api available in the *sensor-api* folder, using Z-COINES for hardware interface.

User guides for Z-COINES and sensor device drivers are available in the *doc* folder. Samples for both of these kinds of apps are available in the *samples* folder.

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
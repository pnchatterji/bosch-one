.. _flogfs_fs:
Copyright (c) 2023 Bosch Sensortec GmbH
FLogFS Zephyr port source
#########################

Overview
********
FLogFS is an open source filesystem optimized for logging operations 
on external NAND Flash chips. 
It can be downloaded at https://github.com/bnahill/FLogFS

This is the source code of the FlogFS file system wrapped in a 
Zephyr File System Driver. 
flogfs_fs.c is the adapter for interfacing FLogFS into the Zephyr file system.
The other files are the FlogFS file system sources
Note the FlogFS code here is a modified version of 
the sources available on Github, in order to fix some bugs. Changes have not yet
been pushed 

It needs to be associated in the application DTS overlay to a 
suitable NAND Flash driver. 

It has been tested with Bosch Sensortic Application Board 3.0 
which a Winbond W25M02GW flash chip connected to the MCU via 
SPI0 or SPI3 (for higher frequency).

The board DTS is bosch_app30

As configured in bosch_app30 DTS, the The FlogFS file system is auto-mounted
and available under the Zephyr filesystem as the volume /flog0. 
The path to a file called foo.txt on this volume would be /flog0/foo.txt

The Zephyr filesystem commands can be used to access this volume by using /flog0 as
the path root.

(Also tested with Application Board 3.1 DTS bosch_app31)
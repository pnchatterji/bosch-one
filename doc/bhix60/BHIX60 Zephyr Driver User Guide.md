# BHIx60 Sensor Driver for Zephyr

# User Guide

© Bosch Sensortec GmbH 2023

Document Version: 1.3

Date: 25 March 2023

Contents

[Overview 3](#_Toc129950255)

[Prerequisites 3](#_Toc129950256)

[References 3](#_Toc129950257)

[Introduction 4](#_Toc129950258)

[Sensor API and Raw API 4](#_Toc129950259)

[Configuring the DTS 4](#_Toc129950260)

[Firmware Upload 5](#_Toc129950261)

[Initializing the driver and virtual sensors 5](#_Toc129950262)

[FIFO Processing 6](#_Toc129950263)

[Accessing Sensor Value 7](#_Toc129950264)

[Triggers 8](#_Toc129950265)

[Triggers and the Fetch Loop 10](#_Toc129950266)

[Disabling a Virtual Sensor 10](#_Toc129950267)

[Complex Virtual Sensors 11](#_Toc129950268)

[Custom Virtual Sensor 11](#_Toc129950269)

[Power Management 11](#_Toc129950270)

[Project Configuration 12](#_Toc129950271)

[Full Example 12](#_Toc129950272)

[Including the BHY2 sources 13](#_Toc129950273)

[Building and Running 13](#_Toc129950274)

[Configuration Variables 14](#_Toc129950275)

[Raw API 15](#_Toc129950276)

[Raw API Functions 15](#_Toc129950277)

[Data Accessor Macros 19](#_Toc129950278)

[Table of Sensor Channels 21](#_Toc129950279)

[Table of Data Formats 24](#_Toc129950280)

[Table of Sensor Triggers 25](#_Toc129950281)

[BHIX60 Sensor Channel to Zephyr Sensor Channel Mapping 26](#_Toc129950282)

[Nicla Debugging 27](#_Toc129950283)

# Overview

This document is a user guide and general description for the BHIx60 Zephyr Sensor driver

# Prerequisites

This document assumes that the reader is familiar with application development in Zephyr, specifically the sensor API, ideally using the nrf Connect SDK from Nordic and the nrf Connect plugin for VS Code

The reader should also have a broad familiarity with the BHI260 or BHI360 product specifications.

References

Developing with Zephyr (basics)

[https://docs.zephyrproject.org/3.2.0/develop/index.html](https://docs.zephyrproject.org/3.2.0/develop/index.html)

Developing Zephyr apps using NRF Connect SDK and plugin

[https://developer.nordicsemi.com/nRF\_Connect\_SDK/doc/latest/nrf/getting\_started.html](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/getting_started.html)

Zephyr Sensor API:

[https://docs.zephyrproject.org/3.2.0/hardware/peripherals/sensor.html](https://docs.zephyrproject.org/3.2.0/hardware/peripherals/sensor.html)

BHI260 Product specification (Datasheet):

[https://www.bosch-sensortec.com/products/smart-sensors/bhi260ab/](https://www.bosch-sensortec.com/products/smart-sensors/bhi260ab/)

[BHY\_DFMT\_TAB] Table 'FIFO Data Types and Formats' in the BHIx60 product specification

# Introduction

The BHIx60 series of devices (BHI260, BHI360 etc.) are smart sensors that process information from multiple physical sensors to produce computed abstract data. Each computed data type is referred to as a "virtual sensor". The virtual sensors supported by a particular device depends on the firmware that is currently loaded on it. The bhix60\_get\_firmware() API described below can be used to load a new firmware.

The BHIx60 devices are controlled using the BHY2 Sensor API from Bosch. The bhix60 driver is a thin wrapper on the BHY2 Sensor API, and provides a Zephyr-compatible interface
to BHIx60 devices.

# Sensor API and Raw API

It is possible to use the BHIx60 driver exclusively using the Zephyr Sensor API. The standard Zephyr channels like SENSOR\_CHAN\_ACCEL\_XYZ can be used to access the required virtual sensors. It is also possible to use the extended sensor channels (refer _enum sensor\_channel\_bhix60_ in _bhix60.h_). The extended sensor channels allow access to the wakeup/non-wakeup virtual sensors and special sensors like PDR, SWIM etc. The output values for the various virtual sensors using _sensor\_channel\_get()_ are in SI units, as specified in the Zephyr Sensor API.

There is also a Raw API, defined in bhix60.h, which provides access to the BHY2 layer BHIx60 for more fine-grained control of the device. It is also the only way to access some advanced virtual sensors such as Swim, PDR, etc. The output values from the various sensors using the Raw API correspond to the format described in the BHIx60 product specification (refer table 'FIFO Data Types and Formats') [BHY\_DFMT\_TAB]

**PLEASE NOTE: The outputs from** _**sensor\_channel\_get()**_ **and the Raw API are NOT the same, as noted above.**

# Configuring the DTS

Before the bhix60 driver can be called in the application code, it needs to be defined in the corresponding board DTS, if the definition does not exist. In case of some boards, e.g. Arduino Nicla Sense ME, the DTS is pre-configured for bhix260, so no further action is required. In other cases, e.g. for Bosch application boards with a BHIx60 shuttle board mounted, the driver needs to be added as a node to the appropriate SPI node in an overlay DTS file.

```
    /* myboard.overlay */
    
	&spi2 {
		compatible = "nordic,nrf-spi";
		status = "okay";
		pinctrl-0 = <&spi2default>;
		pinctrl-1 = <&spi2sleep>;
		pinctrl-names = "default", "sleep";
		/*0: CS of BHI260 1:CS of Flash*/
		cs-gpios = <&gpio0 31 GPIOACTIVEHIGH>;
		bhix60dev: bhix60@0 {
			compatible = "bosch,bhix60";
			reg = <0>;
			spi-max-frequency = <1000000>;
			label = "bhix60";
			status = "okay";
			reset-gpios = <&gpio0 18 GPIOACTIVELOW>;	/*reset pin of BHI260*/
			hirq-gpios = <&gpio0 14 GPIOACTIVEHIGH>;	/*Interrupt-OUT pin of BHI260*/
		};
	};
```

The SPI module refers to the SPI channel used to connect the BHIx60 device to the host CPU. The pin numbers refer to the host GPIO pins connected to the BHIx60 chip select, reset and hirq.

"_Myboard"_ is a placeholder for the name of the actual board being used. The overlay file is kept in the _boards_ subfolder of the application.

# Firmware Upload

In case BHIx60 is configured for run-time firmware loading to the RAM or FLASH of BHIx60 (CONFIG\_BHIX60\_UPLOAD\_FW\_TO\_FLASH or CONFIG\_BHIX60\_UPLOAD\_FW\_TO\_RAM), the application needs to define the following accessor function to provide the BHIx60 driver access to the application-defined firmware.
```
	int bhix60_get_firmware(const struct device *dev, unsigned char **fw, unsigned int *fwsz);

	dev:    BHIx60 device instance pointer
	fw:     byte array pointer to be initialized with a pointer to the byte array of the firmware
	fwsz:   integer pointer to be filled with the size of the byte array of the firmware
	Return: 0 if no errors, -1 firmware not available
```

Example: The BHIx60 firmwares generated by the BHY2 firmware SDK are typically available as C header files with names such as "BHI260AP.fw.h", containing an array definition like this:

` const unsigned char bhy2_firmware_image[] = {...}; `

The application should define the accessor function as follows:

```
	#include <bhi260.h>
	#include "BHI260AP.fw.h" /*change to the name of the actual firmware file*/
	int bhix60_get_firmware(const struct device *dev, unsigned char **fw,unsigned int *fw_sz)
	{
		(void)dev;
		*fw = bhy2_firmware_image;
		*fw_sz = sizeof(bhy2_firmware_image);
		return 0;
	}
```
NOTE1: In case a firmware is not (yet) available, an error code (-1) can be returned instead of 0 in a place-holder function to avoid build and runtime errors.

NOTE2: In case the application supports multiple BHIx60 devices, the dev argument passed by the driver can be used to load different firmware in different devices, if required.

# Initializing the driver and virtual sensors

The _bhix60\_dev_ label defined in the DTS overlay above can be used to get the device pointer of the bhix60 driver. Before any virtual sensor can be used, they need to be registered and configured. The registration can be done using the raw API call _bhix60\_register\_virtual\_sensor(),_ or using the sensor API call _sensor\_attr\_set()_ with the attribute SENSOR\_ATTR\_CONFIGURATION. The following example demonstrates the latter approach to register and configure the Game Rotation virtual sensor:

```
	void main(void)
	{
		const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(bhix60_dev));
		struct sensor_value sv_cfg;
		sv_cfg.val1 = 100; /* Sampling rate in hz */
		sv_cfg.val2 = 0;   /* Latency */
		sensor_attr_set(dev, SENSOR_CHAN_BHIX60_GAMERV,
			SENSOR_ATTR_CONFIGURATION,
			&sv_cfg);
		...
```

Note that in the _sv\_cfg_ structure, val1 is set to the desired sampling rate in Hz (how often the BHIx60 should sample the associated hardware sensor), and val2 to the latency (how long in ms an event should reside in the BHIx60 FIFO buffer, before signaling an interrupt to the host CPU. Setting this to 0 implies no latency. This is specific to bhix60. The Zephyr Sensor API does not specify the values to be passed for this attribute, it is left to specific drivers to define this as per their internal design.

The above configuration has to be done for each virtual sensor being used by the application. Virtual sensors that are not configured as above cannot be used. It is possible to configure only those virtual sensors that are supported by the firmware currently uploaded to the BHIx60 device (refer section Firmware Upload).

After configuration, if required, the scale setting can be performed, as in the below example for virtual sensor Acceleration. NOTE: scale setting is done in SI units as specified in the Zephyr Sensor API. Scale setting is only possible for virtual sensors with Scale Factor of type "Dynamic" in [BHY\_DFMT\_TAB]
```
	/* Setting Accl scale in m/s^2 */
	full_scale.val1 = 20; /* 20 m/s^2 = 2G */
	full_scale.val2 = 0;
	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
```

# FIFO Processing

The virtual sensors running on the BHIx60 store events and outputs in a FIFO. The host CPU extracts the events from the FIFO via SPI or I2C. The FIFO processing is central to the interaction with BHIx60. In case of the bhix60 driver, this processing can be done by polling the INT pin of BHIx60 at regular intervals in the user application to see if new data has arrived, or by using the INT pin as a hardware interrupt. In case of hardware interrupt, the interrupt handler only flags the event. The actual FIFO processing happens either in an independent FIFO processing thread, or in the context of the main thread using the Zephyr Work Queue kernel object.

One of these three modes of FIFO processing can be selected by activating one of the following Config variables in _prj.conf_:

1. CONFIG\_BHIX60\_FIFO\_POLL
2. CONFIG\_BHIX60\_FIFO\_INT\_GLOBAL\_THREAD
3. CONFIG\_BHIX60\_FIFO\_INT\_FIFO\_THREAD

In case of polling, the actual polling of the INT pin and the processing of FIFO happens in _sensor\_sample\_fetch()_. In case CONFIG\_BHIX60\_FIFO\_POLL is selected, it is necessary for the user application to call fetch in a loop at least as frequently as the highest sampling frequency set for the registered virtual sensors, otherwise it can lead to a FIFO overflow and data loss. For example:
```
	void main(void)
	{
		const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(bhix60_dev));
		
		... (initialization code here)
		
		while (1) {
			/* 10ms period, 100Hz Sampling frequency */
			k_sleep(K_MSEC(10));
			
#ifdef CONFIG_BHIX60_FIFO_POLL
			sensor_sample_fetch(dev);
#endif
			... (data processing code here)
			
			/*get data from one or more channels*/
			sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, sv);
			/*process the data*/
			printf("%d.%06d\n",sv.val1,sv.val2);
		}
	}
```
In case CONFIG\_BHIX60\_FIFO\_INT\_GLOBAL\_THREAD or CONFIG\_BHIX60\_FIFO\_INT\_FIFO\_THREAD is selected, fetch does not have to be called. _sensor\_channel\_get()_ can be called at any moment, and will always get the latest sensor data. In these modes, fetch has no effect. However, it can still be called for compatibility. This will avoid having to change the code if polling is activated at a later date.

In the above sample code, fetch is called inside #ifdef/endif macro guards. This is recommended for writing code that can quickly be switched between polling mode and interrupt driven mode.

**When to use the different modes:**

Polling is useful in the initial stages of development, for ease of debugging. It avoids data contention issues.

The interrupt driven modes are useful in production code, to reduce power consumption and improve response time. The interrupt modes are most useful when used in conjunction with triggers (refer the section on triggers). However, care must be taken to avoid data contention using mutexes etc. In case there is only one thread in the application (the main thread), and it is lightly loaded, the global thread option can be used to reduce the overheads of a separate FIFO thread and associated synchronization and data contention issues. If the main thread is heavily loaded, the FIFO thread is the better option to improve response time.

# Accessing Sensor Value

Sensor data access can be done after fetch in poll mode, or inside triggers for interrupt modes, using _sensor\_channel\_get()_ for virtual sensors that are supported by the Zephyr Sensor API.

Example:
```
	while (1) {
		struct sensor_value acc[3];
		k_sleep(K_MSEC(10));
		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc);
		printf("AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d; \n",
		acc[0].val1, acc[0].val2,
		acc[1].val1, acc[1].val2,
		acc[2].val1, acc[2].val2,
	}
```
The value returned by _sensor\_channel\_get()_ for supported channels is in SI units as specified by the Zepyr Sensor API. Refer the _Table of Sensor Channels_ in the Raw API Reference section for details. However, _sensor\_channel\_get()_ cannot be used for complex virtual sensors like KLIO, PDR, Swim, BSEC and for custom virtual sensors. If _sensor\_channel\_get()_ is used on such channels, it returns a _–ENOTSUP_ error value.

For complex virtual sensors and custom virtual sensors, it is necessary to use the Raw API. The Raw API Reference section and the Complex Virtual Sensors section contains the details of all the functions available for accessing raw data from the virtual sensors. The Raw API functions return data as per the BHIx60 FIFO data format specification [BHY\_DFMT\_TAB].

Note that it is also possible to access the standard or non-complex channels using the Raw API, but in this case also, it should be noted that the returned value will be as per [BHY\_DFMT\_TAB] and not as per the Zephyr Sensor API specification.

The following example shows how to access a virtual sensor channel using _sensor\_channel\_get()_ and the Raw API.
```
	/*access temperature using sensor_channel_get*/
	struct sensor_value sv_temp;
	sensor_channel_get(dev,SENSOR_CHAN_AMBIENT_TEMP,&sv_temp);
	printf("Temperature: %d.%06d C \n",sv_temp.val1,sv_temp.val2);

	/*access temperature using Raw API*/
	float temperature;
	uint64_t timestamp;
	uint32_t s, ns;
	bhix60_get_temperature_celsius(dev, SENSOR_CHAN_AMBIENT_TEMP,&temperature,&timestamp);
	bhix60_convert_time(timestamp, &s, &ns);
	printf("Temperature: %f C at %d:%d seconds \n",temperature,s,ns);
```
An advantage of the Raw API is that it provides access to the timestamp data also, which is the time at which the value was sensed by the underlying hardware sensor. Timestamp is given in nanoseconds since the start of application.

# Triggers

Sensor Triggers are callback functions that get called when a particular sensor state is reached. Triggers can be set using the _sensor\_trigger\_set()_ Zephyr Sensor API function. Three types of triggers are currently supported:

**SENSOR\_TRIG\_DATA\_READY** : this trigger is fired when data for a particular virtual sensor is pulled from the BHIx60 FIFO. This is supported by all virtual sensors, including Event type sensors (in this case, data is 0). This is an example of how it can be set and processed:
```
	void main(void)
	{
		... (device, virtual sensor initialization)
		
		struct sensor_trigger rotation_trig = {
			.chan =SENSOR_CHAN_BHIX60_GAMERV,
			.type=SENSOR_TRIG_DATA_READY,
		};

		sensor_trigger_set(dev, &rotation_trig, rotation_trigger_handler);

		while (1) 
		{
			... (sleep etc.)
			
			sensor_sample_fetch(dev);
		}
	}
	
	void rotation_trigger_handler(const struct device *dev,
		const struct sensor_trigger *trigger)
	{
		struct bhy2_data_quaternion data;
		uint64_t timestamp;
		bhix60_get_quaternion(dev, trigger->chan, &data, &timestamp);
		printf("Game Rotation: x: %d, y: %d, z: %d, w: %d; \n",
			data.x, data.y, data.z, data.w);
	}
```
**SENSOR\_TRIG\_BHIX60\_EVENT** : this trigger is fired when a sensor event is pulled from the BHIx60 FIFO. This is supported by all virtual sensors that produce Event-type output (e.g. Tilt Detected).
```
	void main(void)
	{
		... (dev, virtual sensor initialization)
		
		struct sensor_trigger tilt_trig = {
			.chan =SENSOR_CHAN_BHIX60_TILT,
			.type=SENSOR_TRIG_BHIX60_EVENT,
		};
		sensor_trigger_set(dev, &tilt_trig, tilt_trigger_handler);

		while (1) 
		{

			... (sleep etc.)

			sensor_sample_fetch(dev);
		}
	}

	void tilt_trigger_handler(const struct device *dev,
		const struct sensor_trigger *trigger)
	{
		printf("Tilt Detected!\n");
	}
```
**SENSOR\_TRIG\_THRESHOLD** : this trigger is fired when the sensor value goes above or below a threshold value. Threshold values are set using the _sensor\_attr\_set()_ function with the attribute SENSOR\_ATTR\_UPPER\_THRESH or SENSOR\_ATTR\_LOWER\_THRESH. Threshold values are set in SI units as per the Zephyr Sensor API specification. This trigger can only be set for virtual sensors that return integer, 3D Vector, Quaternion and Euler data types. In case of 3D Vector and Euler, the threshold trigger fires if ANY of sub-elements exceeds the threshold (X,Y or Z or H,P or R). In case of Quaternion, the threshold is applied only to the W sub-element.
```
	void main(void)
	{
		... (dev, virtual sensor initialization)
		
		struct sensor_trigger temp_trig ={
			.chan = SENSOR_CHAN_BHIX60_TEMP,
			.type = SENSOR_TRIG_BHIX60_EVENT,
		};

		/*set threshold at 75.5 degrees Centigrade*/
		struct sensor_value max_temp = {
			.val1 = 75; 	 /*integral part (75 degrees C) */
			.val2 = 5000000; /*fractional part * 1000000 (0.5C)*/
		};

		bhix60_attr_set(dev,
			SENSOR_CHAN_BHIX60_TEMP,
			SENSOR_ATTR_UPPER_THRESH,
			&max_temp);

		sensor_trigger_set(dev, &temp_trig, temperature_trigger_handler);

		while (1) 
		{

			... (sleep etc.)

			sensor_sample_fetch(dev);
		}
	}

	void temperature_trigger_handler(const struct device *dev,
		const struct sensor_trigger *trigger)
	{
		printf("Temperature exceeds 75.5C!\n");
	}
```
## Triggers and the Fetch Loop

It can be observed that the fetch function is called in a loop in all of the above examples. This is because polling mode is assumed (CONFIG\_BHIX60\_FIFO\_POLL). If an interrupt mode is selected, the fetch loop is not required, as explained previously in the FIFO processing section, and some other work can be done in the main thread. In polling mode, triggers are fired when the fetch function detects the necessary trigger condition while processing the BHIx60 FIFO. If fetch is not called cyclically, the triggers will not fire.

A side-effect of this is that the trigger handler is actually called in the thread context of the fetch loop. The advantage of this is that there is no need to provide data locking using mutex etc., as long as all processing is being done in the main thread (as in the above examples). This is also true for CONFIG\_BHIX60\_FIFO\_INT\_MAIN\_THREAD.

If hardware interrupt with FIFO Thread mode (CONFIG\_BHIX60\_FIFO\_INT\_FIFO\_THREAD) is selected, the trigger handler is called in the context of the FIFO thread created by the bhix60 driver. In this case, if sensor data is extracted in the trigger thread and shared globally with other threads, access needs to be protected by mutexes.

# Disabling a Virtual Sensor

Virtual sensors that have been registered and configured using SENSOR\_ATTR\_CONFIGURATION (refer section _Initializing the Driver and Virtual Sensors_) can be temporarily disabled using SENSOR\_ATTR\_SAMPLING\_FREQUENCY, and setting sampling frequency to 0. They can be re-enabled by setting the sampling frequency to a positive value. This can be useful, for example, when the CPU is entering sleep mode. This prevents the FIFO from overflowing during sleep. The virtual sensor remains registered, and retains any other configuration, but it no longer pushes data into the FIFO. Example:
```
	struct sensor_value sv_cfg;
	sv_cfg.val1 = 0; /* Sampling rate 0: disable */
	sv_cfg.val2 = 0; /* Latency (ignored for disable)*/
	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY,&sv_cfg);
	sv_cfg.val1 = 100; /* Sampling rate 100: re-enble at 100Hz */
	sv_cfg.val2 = 0;   /* Latency (needs to be re-specified, can be same as before or different)*/
	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sv_cfg);
```
# Complex Virtual Sensors

The data from complex sensors such as SWIM, PDR, KLIO and BSEC can be accessed via the Raw API. They cannot be accessed via _sensor\_channel\_get()._ However, helper macros are available to make the job easier. There is a header file defined for each of these complex sensors which contains the corresponding helper macros, e.g. bhix60\_swim.h, bhix60\_pdr.h etc.

Please refer the documentation in the header files for more details. There is a sample for each of these sensors in _samples/sensor/bhix60_ (e.g. _main\_swim.c, main\_pdr.c_ etc.).

# Custom Virtual Sensor

It is possible to install custom firmware on a BHIx60 device, with custom virtual sensors. Refer the BHIx60 product specifications and SDK document for more details. To support custom virtual sensors using the bhix60 driver, the following points need to be taken care of:

1. The custom virtual sensor must be allocated a BHY2 sensor ID in the range BHY2\_SENSOR\_ID\_CUSTOM\_START and BHY2\_SENSOR\_ID\_CUSTOM\_END
2. This can be converted into a bhix60 channel ID using the Raw API macro _bhix60\_sid\_to\_chan()_
3. _sensor\_channel\_get()_ cannot be used to access the data. It is necessary to use the Raw API.
4. In order to use the Raw API, a raw data parser and an accessor macro must be defined. Many examples of parsers and accessor macros can be seen in _bhix60.h_, for example _bhix60\_get\_quaternion(),_ and can be used as a reference.
5. The DATA\_READY trigger is available for custom channels, as for standard channels.
6. If the custom channel is of type Event, the general-purpose trigger ID for events SENSOR\_TRIG\_BHIX60\_EVENT can be used to create a trigger. It is also possible to use any of the other Event-type trigger IDs where appropriate. E.g., SENSOR\_TRIG\_SIG\_MOTION etc.
7. If the custom sensor supports scaling, it is necessary to use low-level BHY2 API calls to set the scale factor. _sensor\_set\_attr()_ with SENSOR\_ATTR\_FULL\_SCALE cannot be used.
8. Everything else written in this document for the pre-defined virtual sensors applies to custom virtual sensors also.

Refer the sample _main\_custom.c_ in _samples/sensor/bhix60_ for a complete custom sensor example.

# Power Management

BHIx60 has inbuilt support for sleep mode of the host processor. By calling the Raw API function _bhix60\_host\_standby(dev,true)_, the BHIx60 device can be informed that the host CPU is entering sleep/standby mode. Thereafter, the BHIx60 will not toggle the INT pin when new data is fetched for **Non-Wakeup Virtual Sensors**. The virtual sensors can also be disabled to reduce power consumption of the BHIx60, as described in the section _Disabling a Virtual Sensor_. However, the INT pin will continue to be toggled for **Wakeup Virtual Sensors**. The host CPU can use this to resume when a wakeup event is detected. It is up to the application to select the Wakeup/Non-Wakeup versions of virtual sensors by registering the appropriate sensor channels. The _Table of Sensor Channels_ in the Raw API reference section provides the Wakeup/Non-Wakeup nature of each channel. The host CPU should call _bhix60\_host\_standby(dev,false)_ to reactivate all channels.

The bhix60 sample _main\_wakeup.c_ in _zephyr/samples/sensor/bhix60_ demonstrates how the BHIx60 sleep/wakeup feature can be integrated with the **Zephyr Power Management (PM) Module**.

# Project Configuration

Following is the _prj.conf_ settings for the BHIx60 samples (see next section). Most BHIx60 applications will require these settings. A complete list of config settings relevant to BHIx60 is available in the API section.
```
	#BHIx60 requires a large stack for FIFO processing
	#it also requires a heap. Following settings are CRITICAL
	#otherwise there will be build and runtime errors
	CONFIG_MAIN_STACK_SIZE=8192
	CONFIG_HEAP_MEM_POOL_SIZE=2048
	#Enable the below option to get debug messages
	#from the BHIx60 onboard MPU
	CONFIG_BHIX60_STATUS_DEBUG=n
	#Change below setting to choose upload of firmware to
	#RAM, Flash, or boot without upload if firmware is pre-flashed
	#Only enable ONE of the below three options.
	#Note that the firmware being uploaded also has to be
	#appropriate for the choice (RAM or Flash versions)
	CONFIG_BHIX60_UPLOAD_FW_TO_RAM=y
	CONFIG_BHIX60_UPLOAD_FW_TO_FLASH=n
	CONFIG_BHIX60_FLASH_AUTO_BOOT=n
	#Specify how the BHIx60 FIFO will be processed
	#1. The BHIx60 INT pin will be polled in an application fetch loop
	#2. Set BHIx60 INT pin as hw interrupt, handle in the global thread.
	#3. Same as 2, but handle in a special FIFO thread.
	#Activate only ONE of the below three options
	CONFIG_BHIX60_FIFO_POLL=y
	CONFIG_BHIX60_FIFO_INT_GLOBAL_THREAD=n
	CONFIG_BHIX60_FIFO_INT_FIFO_THREAD=n
	#SPI is required by BHIx60
	CONFIG_SPI=y
	#enable the Sensor API for BHIx60 (and any others activated in the DTS)
	CONFIG_SENSOR=y
	#Settings for console output
	CONFIG_STDOUT_CONSOLE=y
	CONFIG_PRINTK=y
	CONFIG_LOG=y
	CONFIG_LOG_BACKEND_UART=y
	CONFIG_LOG_MODE_IMMEDIATE=y
	CONFIG_UART_LINE_CTRL=y
	#The following Power Management Module configurations are
	#required for the WAKE sample. They can be set to n for
	#other samples
	CONFIG_PM=y
	CONFIG_PM_DEVICE=y
	CONFIG_PM_DEVICE_RUNTIME=y
```
# Full Example

A number of bhix60 sample applications are available in the samples directory under _samples/sensor/bhix60_

These application samples incorporate all the concepts and code snippets presented in this document.

# Building and Running

Refer to the _Readme.rst file_ in the bhix60 samples directory for instructions to build and run in the native Zephyr environment.

To build and run the above sample in the nrf Connect environment, follow the below steps. It is presumed that an appropriate version of nrf Connect is installed on the host PC, and a Nicla Sense ME board is connected to the USB port.

1. Launch VS Code
2. Click on the nrf Connect icon in the Toolbar
3. Click on Create a New Application
4. Select bhix60 under "Application Template"
5. Give an appropriate name to the application, e.g. my\_bhix60\_app
6. Verify other settings
7. Click on Create application
8. In the Applications sub-menu, select the newly created my\_bhix60\_app item, and click on the "Add Build Configuration" icon.
9. In the pop-up dialog box, select the board _arduino\_nicla\_sense\_me_ board (assuming that that is the desired target).
10. Click on "Build Configuration" to build
11. If build is successful, click on the Run and Debug icon in the main tool bar to flash the application on the Nicla board and run it (Note, it may be necessary to copy a special-for-Nicla _launch.json_ file in the _.vscode_ directory. Refer section Nicla debugging)

# Configuration Variables

The following configuration variables are defined for bhix60 driver

| Name | Default | Description |
| --- | --- | --- |
| CONFIG\_BHIX60 | y | Enable BHIx60 smart sensor driver. The BHIx60 driver requires a heap and a larger than usual stack size to support the BHY2 Sensor API |
| CONFIG\_BHIX60\_STATUS\_DEBUG | n | Enables BHI260 FIFO for status and debug events |
| _BHIX60\_BOOT\_ACTION_ | | |
| CONFIG\_BHIX60\_FLASH\_AUTO\_BOOT | n | boot BHI260 from its flash at startup, if firmware has been pre-flashed |
| CONFIG\_BHIX60\_UPLOAD\_FW\_TO\_RAM | y | upload app-defined FW to BHIx60 RAM, and boot from it |
| CONFIG\_BHIX60\_UPLOAD\_FW\_TO\_FLASH | n | upload app-defined FW to BHIx60 FLASH, and boot from it |
| CONFIG\_BHIX60\_VIRTUAL\_SENSORS\_USED | 10 | The number of BHIx60 virtual sensors actually used by the application. This should be set as precisely as possible. Setting it too large wastes RAM, as a mailbox slot is maintained for each virtual sensor used. Setting it too low can lead to runtime errors as all the required virtual sensors cannot be initialized |
| CONFIG\_BHIX60\_MAX\_SLOT\_TRIGGERS | 2 | Maximum number of triggers supported per virtual sensor |
| _BHIX60\_FIFO\_MODE_ | | |
| CONFIG\_BHIX60\_FIFO\_POLL | y | The BHIx60 INT pin will be polled in a fetch loop. |
| CONFIG\_BHIX60\_FIFO\_INT\_GLOBAL\_THREAD | n | Set BHIx60 INT pin as hw interrupt, handle in the global thread. |
| CONFIG\_BHIX60\_FIFO\_INT\_FIFO\_THREAD | n | Set BHIx60 INT pin as hw interrupt, handle in a special FIFO thread.. |
| CONFIG\_BHIX60\_FIFO\_THREAD\_STACK\_SIZE | 1024 | Stack size of FIFO thread if CONFIG\_BHIX60\_FIFO\_INT\_FIFO\_THREAD is active |
| CONFIG\_BHIX60\_FIFO\_THREAD\_PRIORITY | 10 | Priority of FIFO thread if CONFIG\_BHIX60\_FIFO\_INT\_FIFO\_THREAD is active |


# Raw API

## Raw API Functions

#### **int16\_t bhix60\_chan\_to\_sid (enum sensor\_channel  chan)**

Convert Zephyr Sensor Channel to BHY2 Sensor ID A negative return signifies an invalid/unsupported channel argument.

1. If a sensor\_channel\_bhix60 enum is passed, it is converted to the corresponding BHY2 sensor ID.

2. If the value passed is not a sensor\_channel\_bhix60 enum , but it is in the range SENSOR\_CHAN\_BHIX60\_START to SENSOR\_CHAN\_BHIX60\_END, it is converted into a BHY2 Sensor ID by applying a formula, on the assumption that it is a specialized/custom virtual sensor. It is up to the application to interpret the data by using the raw data interface (these channels will not be supported by sensor\_channel\_get())

3. If a standard Zephyr sensor channel is passed, it is mapped to a BHY2 sensor ID where possible, otherwise an ENOTSUP (not supported) error is returned. Note that for multiaxis sensors, the individual X, Y and Z channels are not supported, but the multiaxis \_XYZ channel is.

4. -ENOTSUP is returned for all other arguments

##### **Parameters**

  _chan_   Zephyr Sensor Channel  

##### **Returns**

\>0: BHY2 Sensor ID, \-ENOTSUP: Not supported Error

#### **int bhix60\_channel\_parse\_get(const struct device \*dev, enum sensor\_channel chan, bhix60\_parser\_t parse\_func, void \*parsed\_data, uint64\_t \*timestamp)**

Get parsed data from a BHIx60 virtual sensor

This function can be used to retrieve the raw data from a virtual sensor as specified in the BHIx60 product data sheet (refer section "FIFO Data Types and Formats") and parse it to a suitable data structure.

This function is a low-level alternative to accessing BHIx60 vitual sensor data via sensor\_channel\_get(). It is especially useful in case of custom virtual sensors or complex sensors which are not supported in sensor\_channel\_get(). However, it can be used as a general replacement to sensor\_channel\_get() even otherwise, as it is marginally more efficient, and provides access to the timestamp, which is not supported in the standard Zephyr sensor API

It requires a parser function to be passed as a parameter, which will be applied on the raw data from FIFO. There are various parsers pre-defined in bhy2\_parse.h in the BHY2 sensor API. The below helper macros combine bhix60\_channel\_raw\_get() with a standard parser.

In case of custom virtual sensor or unsupported virtual sensors, it is up to application to provide a custom parser on the same lines as the standard parsers. Even if no parsing is required, i.e. direct access to the raw data is desired, it is necessary to supply a copy parser that will use _memcpy_ to copy the FIFO raw data to an application buffer of sufficient size.

##### **Parameters**

  _dev_   			BHIx60 device  
  _chan_   			BHIx60 virtual sensor channel  
  _parse\_func_   	Function pointer of parser function to be applied to raw data  
  _parsed\_data_   	pointer to application-specific data structure to be filled by the parser function  
  _timestamp_   	timestamp of data, in ns since start of application  

##### **Returns**

0: success, -errno: failure

#### **int bhix60\_config\_virtual\_sensor (const struct device \*dev , enum sensor\_channel chan , float \*sample\_rate , uint32\_t report\_latency\_ms)**

Configure a virtual sensor Configure the sampling rate and FIFO latency of a virtual sensor.

A virtual sensor can be configured only after it has been registered, otherwise this call will fail.

This function is automatically called when calling sensor\_attr\_set() with the attribute SENSOR\_ATTR\_CONFIGURATION. This function can be called directly if using the BHIx60 raw API exclusively.

Sampling rate is the rate in hz at which the virtual sensor data is read by BHIx60. Sampling rate is used also to enable/disable the virtual sensor. Set it to 0 for disable, a positive value for enable.

The corresponding way of enabling/disabling a virtual sensor via the standard sensor API is to call sensor\_attr\_set() with the attribute SENSOR\_ATTR\_SAMPLING\_FREQUENCY.

Latency indicates how much time in ms a new value is retained in the BHIx60 FIFO before a notification to the host CPU is sent via interrupt pin. The default value is 0 (notify CPU immediately). A higher value can be set prior to going to sleep state.

##### **Parameters**

  _dev_   BHIx60 device  
  _chan_   BHIx60 virtual sensor channel  
  _sample\_rate_   Rate in hz at which sensor data is read by BHIx60.  
  _report\_latency\_ms_   Time in ms a new value is retained in BHIx60 FIFO  

##### **Returns**

0: success, -errno: failure

#### **void bhix60\_convert\_time (uint64\_t timestamp , uint32\_t \*s , uint32\_t \*ns)**

Convert BHIx60 time stamp to seconds and nanoseconds since application start.

##### **Parameters**

  _timestamp_   timestamp returned by *bhix60\_channel\_raw\_get()* and helper macros  
  _s_   seconds component of timestamp  
  _ns_   nanoseconds component of timestamp  

#### **struct bhy2\_dev \*bhix60\_get\_bhy2\_dev (const struct device \*dev)**

get BHY2 device from Zephyr BHIX60 device this macro converts a BHIx60 Zephyr device pointer to a BHY2 dev pointer required for making raw calls to the BHY2 Sensor API

##### **Parameters**

  _dev_   Zephyr device pointer of a BHIX60 device  

##### **Returns**

struct bhy2\_dev\* corresponding BHY2 device pointer

#### **int bhix60\_get\_firmware (const struct device \*dev , unsigned char \*fw , unsigned int \*fw\_sz )**

template for application-defined firmware accessor

In case BHIx60 is configured for run-time firmware loading to the RAM or FLASH of BHIx60 (CONFIG\_BHIX60\_UPLOAD\_FW\_TO\_FLASH or CONFIG\_BHIX60\_UPLOAD\_FW\_TO\_RAM), the application needs to define the following accessor function to provide the BHIx60 driver access to the application-defined firmware.

##### **Parameters**

  _BHIx60_   device instance pointer  
  _fw_   to be filled with a pointer to the byte array of the firmware  
  _fw\_sz_   to be filled with the size of the byte array of the firmware  

##### **Returns**

0: no errors, -1: firmware not available

#### **int bhix60\_register\_virtual\_sensor (const struct device \*dev , enum sensor\_channel chan)**

Register a virtual sensor prior to usage.

This function sets up the internal buffer, locks and callbacks for polling a virtual sensor. This function is automatically called when calling sensor\_attr\_set() with the attribute SENSOR\_ATTR\_CONFIGURATION. This function can be called directly if using the BHIx60 raw API exclusively.

##### **Parameters**

  _dev_   BHIx60 device  
  _chan_   BHIx60 virtual sensor channel  

##### **Returns**

0: success, -errno: failure

#### **int bhix60\_host\_standby(const struct device \*dev, bool status)**

Signal to BHIx60 that host CPU is entering standby mode, and non-wakeup FIFO interrupts should be suspended. Or that it is resuming, and non-wakeup interrupts can resume.

##### **Parameters**

  _dev_   BHIx60 device  
  _status_   entering suspend/exiting suspend  

##### **Returns**

0: success, -errno: failure

#### **int bhix60\_soft\_reset(const struct device \*dev)**

Reset BHIx60 device

##### **Parameters**

  _dev_   BHIx60 device  

##### **Returns**

0: success, -errno: failure

#### **int bhix60\_flush\_fifo(const struct device \*dev)**

Flush all FIFOs, discarding data.

##### **Parameters**

  _dev_   BHIx60 device  

##### **Returns**

0: success, -errno: failure

## Data Accessor Macros

Helper macros that combine bhix60\_channel\_parse\_get with a BHY2 parser . The BHY2 sensor data types are defined in bhy2\_defs.h
```
	int bhix60_get_orientation(const struct device *dev, 
		enum sensor_channel chan,
		struct bhy2_data_orientation *orientation_data,
		uint64_t *timestamp);

	int bhix60_get_temperature_celsius(const struct device *dev,
		enum sensor_channel chan,
		float *temperature,
		uint64_t *timestamp);
		
	int bhix60_get_humidity(const struct device *dev,
		enum sensor_channel chan,
		float *humidity,
		uint64_t *timestamp);

	int bhix60_get_pressure(const struct device *dev,
		enum sensor_channel chan,
		float *pressure,
		uint64_t *timestamp);

	int bhix60_get_altitude(const struct device *dev,
		enum sensor_channel chan,
		float *altitude,
		uint64_t *timestamp);

	int bhix60_get_quaternion(const struct device *dev,
		enum sensor_channel chan,
		struct bhy2_data_quaternion *quaternion,
		uint64_t *timestamp);

	int bhix60_get_3d_vector(const struct device *dev,
		enum sensor_channel chan,
		struct bhy2_data_xyz *vector,
		uint64_t *timestamp);

	int bhix60_get_32u(const struct device *dev,
		enum sensor_channel chan,
		uint32_t *data,
		uint64_t *timestamp)

	int bhix60_get_24u(const struct device *dev,
		enum sensor_channel chan,
		uint32_t *data,
		uint64_t *timestamp);

	int bhix60_get_16u(const struct device *dev,
		enum sensor_channel chan,
		uint16_t *data,
		uint64_t *timestamp);

	int bhix60_get_pdr_frame(const struct device *dev,
		enum sensor_channel chan,
		struct bhy2_pdr_frame *pdr_frame,
		uint64_t *timestamp);

	int bhix60_get_swim(const struct device *dev,
		enum sensor_channel chan,
		struct bhy2_swim_algo_output *swim,
		uint64_t *timestamp);

	int bhix60_get_air_quality(const struct device *dev,
		enum sensor_channel chan,
		struct bhy2_bsec_air_quality *air_quality,
		uint64_t *timestamp);
```
## Table of Sensor Channels

The following table lists the Extended Sensor Channels defined in bhix60.h, and the corresponding BHY2 Sensor ID and the Data Format of the output data as accessed by _sensor\_channel\_get()_ and Raw API.

| **Sensor Channel** | **SID** | **Description** | **WU** | **Scale** | **Format** | **Trigger** |
| --- | --- | --- | --- | --- | --- | --- |
| SENSOR\_CHAN\_BHIX60\_ACC\_PASS  | 1 | Accelerometer passthrough |   | yes | Accl | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_ACC\_RAW   | 3 | Accelerometer uncalibrated |   | yes | Accl | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_ACC    | 4 | Accelerometer corrected |   | yes | Accl | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_ACC\_BIAS  | 5 | Accelerometer offset |   | yes | Accl | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_ACC\_WU  | 6 | Accelerometer corrected wake up | yes | yes | Accl | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_ACC\_RAW\_WU | 7 | Accelerometer uncalibrated wake up | yes | yes | Accl | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_SI\_ACCEL  | 8 | Virtual Sensor ID for Accelerometer |   |   | Num | DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_GYRO\_PASS  | 10 | Gyroscope passthrough |   | yes | Gyro | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_GYRO\_RAW  | 12 | Gyroscope uncalibrated |   | yes | Gyro | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_GYRO   | 13 | Gyroscope corrected |   | yes | Gyro | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_GYRO\_BIAS  | 14 | Gyroscope offset |   | yes | Gyro | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_GYRO\_WU   | 15 | Gyroscope wake up | yes | yes | Gyro | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_GYRO\_RAW\_WU  | 16 | Gyroscope uncalibrated wake up | yes | yes | Gyro | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_SI\_GYROS  | 17 | Virtual Sensor ID for Gyroscope |   |   | Num | DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_MAG\_PASS  | 19 | Magnetometer passthrough |   | yes | Magn | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_MAG\_RAW   | 21 | Magnetometer uncalibrated |   | yes | Magn | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_MAG    | 22 | Magnetometer corrected |   | yes | Magn | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_MAG\_BIAS  | 23 | Magnetometer offset |   | yes | Magn | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_MAG\_WU  | 24 | Magnetometer wake up | yes | yes | Magn | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_MAG\_RAW\_WU | 25 | Magnetometer uncalibrated wake up | yes | yes | Magn | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_GRA    | 28 | Gravity vector |   | yes | Grav | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_GRA\_WU  | 29 | Gravity vector wake up | yes | yes | Grav | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_LACC   | 31 | Linear acceleration |   | yes | Accl | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_LACC\_WU   | 32 | Linear acceleration wake up | yes | yes | Accl | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_RV   | 34 | Rotation vector |   |   | Quat | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_RV\_WU   | 35 | Rotation vector wake up | yes |   | Quat | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_GAMERV  | 37 | Game rotation vector |   |   | Quat | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_GAMERV\_WU  | 38 | Game rotation vector wake up | yes |   | Quat | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_GEORV   | 40 | Geo-magnetic rotation vector |   |   | Quat | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_GEORV\_WU  | 41 | Geomagnetic rotation vector wakeup | yes |   | Quat | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_ORI    | 43 | Orientation |   |   | Euler | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_ORI\_WU  | 44 | Orientation wake up | yes |   | Euler | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_TILT   | 48 | Tilt detector | yes |   | Event | TILT |
| SENSOR\_CHAN\_BHIX60\_STD    | 50 | Step detector |   |   | Event | STEP |
| SENSOR\_CHAN\_BHIX60\_STC    | 52 | Step counter |   |   | Num | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_STC\_WU  | 53 | Step counter wake up | yes |   | Num | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_SIG    | 55 | Significant motion | yes |   | Event | SIG\_MOTION |
| SENSOR\_CHAN\_BHIX60\_WAKE\_GESTURE | 57 | Wake gesture | yes |   | Event | WAKE\_GEST |
| SENSOR\_CHAN\_BHIX60\_GLANCE\_GESTURE | 59 | Glance gesture | yes |   | Event | GLANCE\_GEST |
| SENSOR\_CHAN\_BHIX60\_PICKUP\_GESTURE | 61 | Pickup gesture | yes |   | Event | PICKUP\_GEST |
| SENSOR\_CHAN\_BHIX60\_AR   | 63 | Activity recognition | yes |   | Activity | DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_WRIST\_TILT\_GESTURE | 67 | Wrist tilt gesture | yes |   | Event | WRIST\_TILT\_GEST |
| SENSOR\_CHAN\_BHIX60\_DEVICE\_ORI | 69 | Device orientation |   |   | Orient | DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_DEVICE\_ORI\_WU   | 70 | Device orientation wake up | yes |   | Orient | DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_STATIONARY\_DET | 75 | Stationary detect | yes |   | Event | STATIONARY |
| SENSOR\_CHAN\_BHIX60\_MOTION\_DET | 77 |  Motion detect | yes |   | Event | MOTION |
| SENSOR\_CHAN\_BHIX60\_ACC\_BIAS\_WU  | 91 | Accelerometer offset wake up | yes | yes | Accl | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_GYRO\_BIAS\_WU | 92 | Gyroscope offset wake up | yes | yes | Gyro | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_MAG\_BIAS\_WU  | 93 | Magnetometer offset wake up | yes | yes | Magn | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_STD\_WU  | 94 | Step detector wake up | yes |   | Event | STEP |
| SENSOR\_CHAN\_BHIX60\_TEMP   | 128 | Temperature |   |   | Temp | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_BARO   | 129 | Barometer |   |   | Pres | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_HUM    | 130 | Humidity |   |   | Hum | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_GAS    | 131 | Gas |   |   | Gas | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_TEMP\_WU   | 132 | Temperature wake up | yes |   | Temp | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_BARO\_WU   | 133 | Barometer wake up | yes |   | Pres | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_HUM\_WU  | 134 | Humidity wake up | yes |   | Hum | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_GAS\_WU  | 135 | Gas wake up | yes |   | Gas | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_STC\_HW  | 136 | Hardware Step counter |   |   | Num | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_STD\_HW  | 137 | Hardware Step detector |   |   | Event | STEP |
| SENSOR\_CHAN\_BHIX60\_SIG\_HW  | 138 | Hardware Significant motion |   |   | Event | SIG\_MOTION |
| SENSOR\_CHAN\_BHIX60\_STC\_HW\_WU  | 139 | Hardware Step counter wake up | yes |   | Num | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_STD\_HW\_WU  | 140 | Hardware Step detector wake up | yes |   | Event | STEP |
| SENSOR\_CHAN\_BHIX60\_SIG\_HW\_WU  | 141 | Hardware Significant motion wakeup | yes |   | Event | SIG\_MOTION |
| SENSOR\_CHAN\_BHIX60\_ANY\_MOTION | 142 | Any motion |   |   | Event | MOTION |
| SENSOR\_CHAN\_BHIX60\_ANY\_MOTION\_WU   | 143 | Any motion wake up | yes |   | Event | MOTION |
| SENSOR\_CHAN\_BHIX60\_EXCAMERA  | 144 | External camera trigger |   |   | Num | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_GPS    | 145 | GPS |   |   | GPS | DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_LIGHT   | 146 | Light |   |   | Light | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_PROX   | 147 | Proximity |   |   | Num | DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_LIGHT\_WU  | 148 | Light wake up | yes |   | Light | THRESHOLD, DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_PROX\_WU   | 149 | Proximity wake up | yes |   | Num | DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_KLIO | 112 | Sef-learning AI (KLIO) |   |   | KLIO | DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_PDR | 113 | Pedestrian Dead Reckoning (PDR) |   |   | PDR | DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_SWIM | 114 | Swim |   |   | SWIM | DATA\_READY |
| SENSOR\_CHAN\_BHIX60\_AIR\_Q | 115 | Air Quality (BSEC) |   |   | BSEC | DATA\_READY |
| **BHI360 SPECIFIC SENSORS** |   |   |   |   |   |   |
| SENSOR\_CHAN\_BHI3\_MULTI\_TAP  | 153 | Multi-tap detector (wear/hear) |   |   | Multi-Tap | DATA\_READY |
| SENSOR\_CHAN\_BHI3\_AR\_WEAR\_WU  | 154 | Activity recognition (wear/hear) |   |   | Activity | DATA\_READY |
| SENSOR\_CHAN\_BHI3\_WRIST\_GEST\_LP\_WU | 156 | Wrist gesture |   |   | Event | WRIST\_TILT\_GEST |
| SENSOR\_CHAN\_BHI3\_WRIST\_WEAR\_LP\_WU   | 158 | Wrist wear wake-up |   |   | Event | WRIST\_TILT\_GEST |
| SENSOR\_CHAN\_BHI3\_NO\_MOTION\_LP\_WU  | 159 | No motion detector |   |   | Event | STATIONARY |
| **CUSTOM CHANNELS** |   |   |   |   |   |   |
| SENSOR\_CHAN\_BHIX60\_XXXX | 160-191 | User Defined Channels | ? | ? | Custom | DATA\_READY |

##

NOTES:

1. **Sensor Channel:** Name of an extended BHIx60 Extended Sensor Channel as defined in bhix60.h. Some of these are mapped to standard Zephyr sensor channels, as listed in the next section
2. **SID:** BHY2 Sensor ID. The corresponding Virtual Sensor ID used by the BHIx60 Firmware
3. **WU:** Wakeup Channels. These channels can be used for Wakeup, as described in the section _Power Management_.
4. **Scale:** Channels where scaling can be applied using _sensor\_attr\_set()_ with the attribute SENSOR\_ATTR\_FULL\_SCALE, as described in section _Initializing the Driver and Virtual Sensors_.
5. **Format:** The format of data received from Virtual sensor using _sensor\_channel\_get()_ and the Raw API. Each of these data formats is described in more detail in the next section.
6. **Trigger:** The triggers that can be set for the channel. Refer the list of triggers in the following section for more details.

## Table of Data Formats

The following table explains the data formats mentioned in the _Table of Sensor Channels_

| **Format** | **Return value from sensor\_channel\_get()** | **Threshold** | **Scaling** |
| --- | --- | --- | --- |
| **Accl** | sensor\_value[0]: X in m/s2<br /> sensor\_value[1]: Y in m/s2<br /> sensor\_value[2]: Z in m/s2 | sensor\_value in m/s2<br /> common for X,Y or Z | sensor\_value in m/s2<br /> for X,Y and Z |
| **Quat** | sensor\_value[0]: X factor in range 1 to -1<br /> sensor\_value[1]: Y factor in range 1 to -1<br /> sensor\_value[2]: Z factor in range 1 to -1<br /> sensor\_value[3]: W factor in range 1 to -1<br /> sensor\_value[4]: Accuracy in rad | sensor\_value in range 1 to -1<br /> for W only | --- |
| **Euler** | sensor\_value[0]: Heading in degrees<br /> sensor\_value[1]: Pitch in degrees<br /> sensor\_value[2]: Roll in degrees | sensor\_value in degrees<br /> common for X,Y or Z | --- |
| **Grav** | sensor\_value[0]: X in m/s2<br /> sensor\_value[1]: Y in m/s2<br /> sensor\_value[2]: Z in m/s2 | sensor\_value in m/s2<br /> common for X,Y or Z | sensor\_value in m/s2<br /> for X,Y and Z |
| **Gyro** | sensor\_value[0]: X in rad/s<br /> sensor\_value[1]: Y in rad/s<br /> sensor\_value[2]: Z in rad/s | sensor\_value in rad/s common for X,Y or Z | sensor\_value in rad/s<br /> for X,Y and Z |
| **Magn** | sensor\_value[0]: X in Gauss<br /> sensor\_value[1]: Y in Gauss<br /> sensor\_value[2]: Z in Gauss | sensor\_value in Gauss<br /> common for X,Y or Z | sensor\_value in Gauss<br /> for X,Y and Z |
| **Gas** | sensor\_value in Ohm | sensor\_value in Ohm | --- |
| **Hum** | sensor\_value in pH % | sensor\_value in pH % | --- |
| **Light** | sensor\_value in Lux | sensor\_value in Lux | --- |
| **Pres** | sensor\_value in kilo Pascal | sensor\_value in kilo Pascal | --- |
| **Temp** | sensor\_value in degrees C | sensor\_value in degrees C | --- |
| **Activity** | sensor\_value.val1: Activity Bitfield<br /> as per [BHY\_DFMT\_TAB]<br /> sensor\_value.val2: Unused | --- | --- |
| **Orient** | sensor\_value.val1: Orientation enum<br /> as per [BHY\_DFMT\_TAB]<br /> sensor\_value.val2: Unused | --- | --- |
| **Num** | sensor\_value.val1: Unitless Numeral<br /> (count etc)<br /> sensor\_value.val2: Unused | sensor\_value.val1: count etc<br /> sensor\_value.val2: Unused | --- |
| **Multi-Tap** | sensor\_value.val1: Multitap Bitfield<br /> as per [BHY\_DFMT\_TAB]<br /> sensor\_value.val2: Unused | --- | --- |
| **Event** | No Data | --- | --- |
|   |   |   |   |
| **BSEC** | Not Supported (Use Raw API) | --- | --- |
| **SWIM** | Not Supported (Use Raw API) | --- | --- |
| **PDR** | Not Supported (Use Raw API) | --- | --- |
| **GPS** | Not Supported (Use Raw API) | --- | --- |
| **KLIO** | Not Supported (Use Raw API) | --- | --- |
|   |   |   |   |
| **Custom** | Not Supported (Use Raw API) | Not Supported (use Raw API) | Not Supported (use Raw API) |

NOTES:

1. As defined in the Zephyr Sensor API, the _sensor\_value_ data structure has two 32-bit integer elements:
    - **val1:** integer part of value.
    - **val2:** fractional part of value multiplied by 1000000.
2. The above table only lists the value returned by _sensor\_value\_get()_. In case of Raw API, please refer to bhix60.h and the various complex sensor header files like bhix60\_swim.h etc. for further details.
3. Please refer the section _Custom Virtual Sensors_ for more information on how to handle custom virtual sensors using the bhix60 driver.

## Table of Sensor Triggers

The following table lists the triggers mentioned in the _Table of Sensor Channels_

SENSOR\_TRIG\_BHIX60\_EVENT is the common trigger for ALL virtual sensors of type EVENT. The other triggers mentioned below are all mapped to SENSOR\_TRIG\_BHIX60\_EVENT internally. The only reason for having the other triggers is for code clarity and for compatibility with the Zephyr Sensor API, so that the application code is portable with other sensor devices.

| **Trigger Type** | **Trigger Name** |
| --- | --- |
| Common for all Event Sensors | SENSOR\_TRIG\_BHIX60\_EVENT |
| GLANCE | SENSOR\_TRIG\_GLANCE\_GEST |
| MOTION | SENSOR\_TRIG\_MOTION |
| PICKUP\_GEST | SENSOR\_TRIG\_PICKUP\_GEST |
| SIG\_MOTION | SENSOR\_TRIG\_SIG\_MOTION |
| STATIONARY | SENSOR\_TRIG\_STATIONARY |
| STEP | SENSOR\_TRIG\_STEP |
| TILT | SENSOR\_TRIG\_TILT |
| WAKE\_GEST | SENSOR\_TRIG\_WAKE\_GEST |
| WRIST\_TILT\_GEST | SENSOR\_TRIG\_WRIST\_TILT\_GEST |

## BHIX60 Sensor Channel to Zephyr Sensor Channel Mapping

Some of the BHIx60 custom sensor channels can also be accessed using standard Zephyr Sensor Channels. The following table provides the mapping between the BHIX60 Sensor Channel and Zephyr Sensor Channels. When using the bhix60 driver via the Zephyr Sensor API, either of the channel IDs can be used interchangeably.

| **Zephyr Channel** | **Extended BHIX60 Channel** |
| --- | --- |
| SENSOR\_CHAN\_ACCEL\_XYZ | SENSOR\_CHAN\_BHIX60\_ACC |
| SENSOR\_CHAN\_GYRO\_XYZ | SENSOR\_CHAN\_BHIX60\_GYRO |
| SENSOR\_CHAN\_MAGN\_XYZ | SENSOR\_CHAN\_BHIX60\_MAG |
| SENSOR\_CHAN\_AMBIENT\_TEMP | SENSOR\_CHAN\_BHIX60\_TEMP |
| SENSOR\_CHAN\_PROX | SENSOR\_CHAN\_BHIX60\_PROX |
| SENSOR\_CHAN\_HUMIDITY | SENSOR\_CHAN\_BHIX60\_HUM |
| SENSOR\_CHAN\_LIGHT | SENSOR\_CHAN\_BHIX60\_LIGHT |
| SENSOR\_CHAN\_GAS\_RES | SENSOR\_CHAN\_BHIX60\_GAS |
| SENSOR\_CHAN\_PRESS | SENSOR\_CHAN\_BHIX60\_BARO |
| SENSOR\_CHAN\_ROTATION\_QTR | SENSOR\_CHAN\_BHIX60\_RV |
| SENSOR\_CHAN\_GAME\_ROTATION\_QTR | SENSOR\_CHAN\_BHIX60\_GAMERV |
| SENSOR\_CHAN\_GEOMAG\_ROTATION\_QTR | SENSOR\_CHAN\_BHIX60\_GEORV |
| SENSOR\_CHAN\_ORIENTATION\_HPR | SENSOR\_CHAN\_BHIX60\_ORI |
| SENSOR\_CHAN\_GRAVITY\_XYZ | SENSOR\_CHAN\_BHIX60\_GRA |
| SENSOR\_CHAN\_LINEAR\_ACCEL\_XYZ | SENSOR\_CHAN\_BHIX60\_LACC |
| SENSOR\_CHAN\_STEP\_CNT | SENSOR\_CHAN\_BHIX60\_STC |
| SENSOR\_CHAN\_DEVICE\_ORIENTATION | SENSOR\_CHAN\_BHIX60\_ORI |
| SENSOR\_CHAN\_CAMERA\_SHUTTER | SENSOR\_CHAN\_BHIX60\_EXCAMERA |
| SENSOR\_CHAN\_SIG\_MOTION | SENSOR\_CHAN\_BHIX60\_SIG |
| SENSOR\_CHAN\_STEP | SENSOR\_CHAN\_BHIX60\_STD |
| SENSOR\_CHAN\_TILT | SENSOR\_CHAN\_BHIX60\_TILT\_DETECTOR |
| SENSOR\_CHAN\_WAKE\_GEST | SENSOR\_CHAN\_BHIX60\_WAKE\_GESTURE |
| SENSOR\_CHAN\_GLANCE\_GEST | SENSOR\_CHAN\_BHIX60\_GLANCE\_GESTURE |
| SENSOR\_CHAN\_PICKUP\_GEST | SENSOR\_CHAN\_BHIX60\_PICKUP\_GESTURE |
| SENSOR\_CHAN\_WRIST\_TILT\_GEST | SENSOR\_CHAN\_BHIX60\_WRIST\_TILT\_GESTURE |
| SENSOR\_CHAN\_STATIONARY | SENSOR\_CHAN\_BHIX60\_STATIONARY\_DET |
| SENSOR\_CHAN\_MOTION | SENSOR\_CHAN\_BHIX60\_MOTION\_DET |

# Nicla Debugging

In order to flash and debug the Nicla Sense ME board from VSCODE, the following text should be copied to a file named _launch.json_, and placed in the application folder in a subdirectory called "_.vscode"._
```
	{
		"version": "0.2.0",
		"configurations": [
			{
				"name": "Nicla Debug (PyOCD)",
				"cwd": "${workspaceFolder}",
				"executable": "./build/zephyr/zephyr.elf",
				"request": "launch",
				"type": "cortex-debug",
				"runToEntryPoint": "main",
				"servertype": "pyocd",
				"targetId": "nrf52",
				"device": "nrf52832",
				"svdFile": "${config:nrf-connect.topdir}/modules/hal/nordic/nrfx/mdk/nrf52.svd",
			}
		]
	}
```
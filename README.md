# bosch-one
## Zephyr External Module for Bosch Sensortec GmbH.

This module should be installed after nrf Connect is installed

Follow [nRF Connect SDK Getting Started](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/gs_installing.html)
for details on how to setup nRF Connect SDK based projects.

### Add bosch-one to the nrf Connect Zephyr manifest
Subsequently, add the following entry to *ncs/vx.y.z/nrf/west.yml* in the *projects* section of the manifest.
The entry should be in alphabetical order with respect to the existing projects. 

Note: *vx.y.z* represents the latest currently installed version of nrf Connect

```
manifest:
  ...
  projects:
    ...
	# Bosh-One repository.
	- name: bosch-one
	  path: bosch-one
      revision: main
      url: https://github.com/BoschSensortec/bosch-one

```
Alternatively, copy the file *bosch-one.yaml* available in the bosch-one repository to *ncs/vx.y.z/nrf/zephyr/submanifests*
and add *bosch-one* to the allowlist of zephyr project imports in *ncs/vx.y.z/nrf/west.yml*. The entry should be in 
alphabetical order with respect to the other names in the allowlist. 
```
manifest:
  ...
  projects:
    - name: zephyr
      import:
        name-allowlist:
          ...
          - bosch-one
```
### Update the nrf Connect Zephyr module repository 

After adding bosch-one to the west manifest as above, open a cmd console using the [nrf Toolchain Manager](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/getting_started/assistant.html#id13)

*cd* to *ncs/vx.y.z* and enter the following command:

```
west update
```
If the update is successful, there should now be subdirectory named *bosch-one* under *ncs/vx.y.z/* containing the Bosch Sensortec Zephyr Drivers, Sensor API, Libraries, Documents and Samples.

### Use Bosch-One to create a Zephyr application for Bosch Sensortec devices
Refer the sample readme files in bosch-one/samples to build a Zephyr application for a supported Bosch Sensortec device. Detailed documentation is available under bosch-one/docs

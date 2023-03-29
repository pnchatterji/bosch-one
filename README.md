# bosch-one
## Zephyr External Module for Bosch Sensortec GmbH.

This module should be installed after nrf Connect is installed

Follow [nRF Connect SDK Getting Started](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/gs_installing.html)
for details on how to setup nRF Connect SDK based projects.

### Add *Bosch-One* to the *nrf Connect* Zephyr manifest
Subsequently, add the following entry to *ncs/vx.y.z/nrf/west.yml* in the *projects* section of the manifest.
The entry should be in alphabetical order with respect to the existing projects. 

Note: *vx.y.z* represents the latest currently installed version of *nrf Connect*.

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
Alternatively, copy the file *bosch-one.yaml* available in the bosch-one repository to *ncs/vx.y.z/zephyr/submanifests*
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
### Update the *nrf Connect* Zephyr module repository 

After adding bosch-one to the west manifest as above, open a cmd console using the [nrf Toolchain Manager](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/getting_started/assistant.html#id13)

*cd* to *ncs/vx.y.z* and enter the following command:

```
west update
```
If the update is successful, there should now be subdirectory named *bosch-one* under *ncs/vx.y.z/* containing the Bosch Sensortec Zephyr Drivers, Sensor API, Libraries, Documents and Samples.

Note: it is necessary to use the cmd console created by the *nrf Toolchain Manager*. If you open an ordinary Windows cmd console, the necessary environment variables are not set. If for some reason you do not wish to use *nrf Toolchain Manager*, you will need to set the environment variables yourself in the following manner:
*cd* to *ncs/vx.y.z* and enter the following command:
```
zephyr\zephyr-env.cmd
```

### Use *Bosch-One* to create a Zephyr application for Bosch Sensortec devices
Refer the sample readme files in bosch-one/samples to build a Zephyr application for a supported Bosch Sensortec device. Detailed documentation is available under bosch-one/docs

In summary, the general procedure is as follows:
- Launch VS Code
- Click on the nrf Connect plugin icon
- In the nrf Connect control pane, click on *Create a new application*
- In the New Application dialog box:
  - Select *Freestanding Application*
  - Select an appropriate location for the application, e.g. ncs/myapps
  - Select an appropriate bosch-one sample to act as template in the template field
  - Provide a name for the application
- A clone of the selected sample is created in the selected location
- Using the sample as base, create your own application
- Add a build configuration for the applicaton by clicking on the *Add Build Configuration* icon in the *Applications* sub-pane
- Select the desired bosch-one board in the *Board* combo-box (Refer next section if bosch-one boards are not visible)
- Add additional CMake arguments if required, as specified in the sample readme
- Enable Debug Options if debugging is desired
- Click on *Build Configuration* to build
- To flash and run, follow the instructions for the specific board, as described in the sample readme

### Using *Bosch-One* boards in *VS Code*

The *bosch-one* specific boards defined in the boards sub-directory (e.g. bst_ab3_nrf52840 and bst_arduino_nicla) are automatically
integrated into the Zephyr build system after *bosch-one* module is installed as described above.

If these boards are not visible inside *VS Code* when creating a new build configuration, it may be necessary to do the following:

- Click on the *VS Code* Settings Icon at the bottom-Left of the window
- In the search box, enter *nrf-connect.boardRoots*
- Click on *Add Item*, and enter the following text:
```
${config:nrf-connect.topdir}/bosch-one/
```
- Click on OK
- The bosch-one boards should now be visible in the *Build Configuration* dialog box.
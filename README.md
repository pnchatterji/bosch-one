# bosch-one
## Zephyr External Module for Bosch Sensortec GmbH.

This module should be installed after nrf Connect is installed

Follow [nRF Connect SDK Getting Started](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/gs_installing.html)
for details on how to setup nRF Connect SDK based projects.

### Add *Bosch-One* to the *nrf Connect* Zephyr manifest
Once nrf Connect SDK is installed, add the following entry to *ncs/vx.y.z/nrf/west.yml* 
at the end of the third-party projects section.
```
manifest:
  ...
  projects:
    ...
    # Other third-party repositories.
    ...
    - name: bosch-one
      path: bosch-one
      revision: main
      url: https://github.com/BoschSensortec/bosch-one
      import: bosch-one.yaml
```
### Update the *nrf Connect* Zephyr module repository 

After adding bosch-one to the west manifest as above, open a Windows cmd console 
 and enter the following commands:

```
cd \ncs\vx.y.z
zephyr\zephyr-env.cmd
west update
```
If the update is successful, there should now be subdirectory named *bosch-one* under *ncs/vx.y.z/* containing the Bosch Sensortec Zephyr Drivers, Sensor API, Libraries, Documents and Samples.

**Notes:**
1. *vx.y.z* in the above description refers to version of *nrf Connect SDK* currently active in the *nrfConnect plugin*
in *VS Code*. E.g. *V2.3.0* Check the currently active SDK version in the *Welcome Page* of *nrf Connect Plugin* if you are not sure.
2. *zephyr-env.cmd* is a Zephyr script for setting the necessary environment variables for *west update* to function properly.
3. The above procedure has to be repeated every time a new version of *nrf Connect SDK* is installed
4. It is also necessary to update the settings of the *nrf Connect Plugin* in *VS Code* after updating the version of the 
*nrf Connect SDK*. This can be done in the *Quick Setup* in the *Welcome Page* of  *nrf Connect Plugin*.

### Use *Bosch-One* to create a Zephyr application for Bosch Sensortec devices
Refer the sample readme files in *bosch-one/samples* to build a Zephyr application for a supported Bosch Sensortec device. Detailed documentation is available under *bosch-one/docs*

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

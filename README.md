# hoverboard-firmware-hack-FOC used to accomplish an electric affordable upgrade on wheelchairs
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

This is a implementation of the amazing [hoverboard FOC firmware](https://github.com/EFeru/hoverboard-firmware-hack-FOC) from [Emmanuel Feru](https://github.com/EFeru) into an adapted wheelchair for people with disabilities.

The project is aimed to promote sustainability and upcycling activity, along with the ability to offer an affordable alternative to people that needs a similar device.

The BLDC motors have been altered to reuse the original wheels from the wheelchair.
The TORQUE mode has been used from the firmware to achieve freewheeling plus some of the other benefits of FOC.

The arduino code included as example has been updated to include rudimentary code to use a standard dual axis joystick and a 8 led WS2812b ring to display when the wheelchair is ON and show permanently the level of the battery.

The project and this GitHub page is still under work.
 
![mainboard_pinout](/docs/pictures/mainboard_pinout.png)

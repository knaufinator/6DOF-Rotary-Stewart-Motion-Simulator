# 6DOF Rotary Stewart Motion Simulator Platform
Compact yet powerful motion simulator platform utilizing 6 AC servo motors. High presicion planetary gears used to multiply the torque. Custom PCB using a ESP32 microcontroller to process the platform position.

This platform is scalable, and most dimensions are changeable within reason. Certain general design rules will need to be followed, in order for the platform to function correctly. This should not be considered a tutorial, but more of a general guide and conceptual starting point for this platform design



## Disclaimer 
This is a DANGEROUS project, and if absolute care is not taken you will be injured or killed.

<a align="center" href="http://www.youtube.com/watch?feature=player_embedded&v=mN0IrtdKdVY
" target="_blank"><img align="center" src="http://img.youtube.com/vi/mN0IrtdKdVY/0.jpg" 
alt="Motion Sim Example" height="340" width="auto" border="5" /></a>

# Projects Included
These are the components of the project that are included in this repository

## Controller
This is an ESP32 Arduino project. This interfaces with the PC through software like simtools, after correctly configuring.  You will need the custom MCP23S17 library in order to compile the project.

## MCP23S17
A Custom MCP23S17 library, modified so the outputs of all 6 motors can be set at one time instead of setting them individually, this saves time and allows for more pulses per second. This increase allows for higher movement precision on the rotational arm. 

## Controller Schematic
Schematic of the current Controller PCB

## Controller PCB
Gerber files for ordering current PCB

## Platform Test Application
.Net Application for testing position limits and speed of platform. Allows for manual setting of each DOF / Axis. As well works with XBOX360 controller through the PC USB wireless adapter.


# Parts
These are some key parts I used, others can be used in their place, but variations of the AC Servo motor may not be compadible with the PCB, and may require a modified PCB schematic. 

## Controller 
Main components on the PCB
* [ESP32 Dev board](https://amzn.to/2OkGpuj) - ESP32 Dev kit
* [MCP23S17](https://amzn.to/32UCSsQ) -
* [3.3V to 5V TTL Shifter Module](https://amzn.to/2VRh3sA) -

## Base
- Steel plate ½ inch thick 31” diameter
- 6 - Coupler https://amzn.to/2slOiIa

## Drive
- 6 - 750w AC servo Motors: https://www.aliexpress.com/item/32844239563.html?spm=a2g0s.9042311.0.0.5c3e4c4dQCTg33
- 6 - 50:1 Planetary Gears: https://www.aliexpress.com/item/32967571001.html?spm=a2g0s.9042311.0.0.5c3e4c4dQCTg33
*Note ensure planetary gear input diameter matches up to both the motor as well with the coupler output diameter when ordering from Aliexpress

## Connecting Arms
- 6 - 24" long 1" OD X .870 ID X .065 Wall Steel tubing
- 12 - 1/2 X 1/2-20 Economy Panhard Bar Kit with Bung .065, Rod End, Heim Joint https://amzn.to/2FQffak
- 12 - 1/2-3/8 High Misalignment Spacers, Rod End Spacers https://amzn.to/2tm1jlF

## Swing Arms
- 6 - 8" long 1" OD X .870 ID X .065 Wall Steel tubing
- 6 - 3/8"-16 Long Coped Steel Bungs

## Chassis
~30' of 1" OD X .870 ID X .065 Wall Steel tubing welded in place with custom rigging to maintain correct proportions. 
6 - 3/8"-16 Long Coped Steel Bungs
Monitor mount (modified) https://amzn.to/2TmVS0f

## Electronics as built in videos
LG 34" Ultrawide https://amzn.to/2t8YvbC
Thrustmaster T16000M FCS https://amzn.to/30qkHtY
Wind Generator Fan https://amzn.to/36W1um9
Wind Generator PWM Control https://amzn.to/2Ns1anq
Wind Generator 90 Degree angle 3"-> 2" Reducer https://amzn.to/2uN6J9z
Wire Wrap https://amzn.to/2u3jiNu

## AC Servo motor settings
These are my settings on the Servo Driver aasd-15a for use with the Controller output
- pn002 - 002
- pn003 - 001 - servo enable, without this the motor is limp
- pn098 - 80
- pn109 - 002
- pn110 - 050
- pn111 - 300
- pn112 - 150

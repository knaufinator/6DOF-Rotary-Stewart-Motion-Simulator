# 6DOF Rotary Stewart Motion Simulator Platform
Compact yet powerful motion simulator platform utilizing 6 AC servo motors. High presicion planetary gears used to multiply the torque. Custom PCB using a ESP32 microcontroller to process the platform position.

This platform is scalable, and most dimensions are changeable within reason. Certain general design rules will need to be followed, in order for the platform to function correctly. This should not be considered a tutorial, but more of a general guide and conceptual starting point for this platform design



## Disclaimer 
This is a DANGEROUS project, and if absolute care is not taken you will be injured or killed.

<a align="center" href="http://www.youtube.com/watch?feature=player_embedded&v=mN0IrtdKdVY
" target="_blank"><img align="center" src="http://img.youtube.com/vi/mN0IrtdKdVY/0.jpg" 
alt="Motion Sim Example" height="340" width="auto" border="5" /></a>

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
ESP32 Dev board
*power reg
*multiplexer
*connectors 
*3-5v shifters


## Drive
6 - 750w AC servo Motors: https://www.aliexpress.com/item/32844239563.html?spm=a2g0s.9042311.0.0.5c3e4c4dQCTg33
6 - 50:1 Planetary Gears: https://www.aliexpress.com/item/32967571001.html?spm=a2g0s.9042311.0.0.5c3e4c4dQCTg33

## Connecting Arms
6 - 24" long 1" OD X .870 ID X .065 Wall Steel tubing
12 - 1/2 X 1/2-20 Economy Panhard Bar Kit with Bung .065, Rod End, Heim Joint
12 - 1/2-3/8 High Misalignment Spacers, Rod End Spacers

## Swing Arms
6 - 8" long 1" OD X .870 ID X .065 Wall Steel tubing
6 - 3/8"-16 Long Coped Steel Bungs

## Chassis
~30' of 1" OD X .870 ID X .065 Wall Steel tubing welded in place with custom rigging to maintain correct proportions. 
6 - 3/8"-16 Long Coped Steel Bungs


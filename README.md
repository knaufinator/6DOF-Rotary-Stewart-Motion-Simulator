# 6DOF Rotary Stewart Motion Simulator Platform
Compact yet powerful motion simulator platform utilizing 6 AC servo motors,planetary gears and a ESP32 microcontroller.

## Disclaimer 
This is a DANGEROUS project, and if absolute care is not taken you will be injured or killed.

<a align="center" href="http://www.youtube.com/watch?feature=player_embedded&v=mN0IrtdKdVY
" target="_blank"><img align="center" src="http://img.youtube.com/vi/mN0IrtdKdVY/0.jpg" 
alt="Motion Sim Example" height="340" width="auto" border="5" /></a>

## Project
This platform is scalable, and most dimensions are changeable within reason. Certain general design rules will need to be followed, in order for the platform to function correctly. This should not be considered a tutorial, but more of a general guide and conceptual starting point for this platform design

### Controller
This is an ESP32 Arduino project. This interfaces with the PC through software like simtools, after correctly configuring.  You will need the custom MCP23S17 library in order to compile the project.

### MCP23S17
A Custom MCP23S17 library, modified so the outputs of all 6 motors can be set at one time instead of setting them individually, this saves time and allows for more pulses per second. This increase allows for higher movement precision on the rotational arm. 

### Controller Schematic
Schematic of the current Controller PCB

### Controller PCB
Gerber files for ordering current PCB

### Platform Test Application
.Net Application for testing position limits and speed of platform. Allows for manual setting of each DOF / Axis. As well works with XBOX360 controller through the PC USB wireless adapter.





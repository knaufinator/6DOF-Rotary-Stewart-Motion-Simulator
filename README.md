# 6DOF Rotary Stewart Motion Simulator Platform
Compact yet powerful motion simulator platform utilizing 6 AC servo motors. High presicion planetary gears used to multiply the torque. Custom PCB using a ESP32 microcontroller to process the platform position.

This platform is scalable, and most dimensions are changeable within reason. Certain general design rules will need to be followed, in order for the platform to function correctly. This should not be considered a tutorial, but more of a general guide and conceptual starting point for this platform design



## Disclaimer 
This is a DANGEROUS project, and if absolute care is not taken you will be injured or killed.

<a align="center" href="http://www.youtube.com/watch?feature=player_embedded&v=xYQOFhglxkg
" target="_blank"><img align="center" src="http://img.youtube.com/vi/xYQOFhglxkg/0.jpg" 
alt="Motion Sim Example" height="340" width="auto" border="5" /></a>

# Projects Included
These are the components of the project that are included in this repository

## Controller
This is an ESP32 Arduino project. This interfaces with the PC through software like simtools, after correctly configuring.  You will need the custom MCP23S17 library in order to compile the project. This project utilizes both ESP32 cores in order to maximize refresh rates to 1000Hz, or 1ms interval.

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
* [NJK-5002C NPN NO（Normally Open)Hall Effect Sensor Switch](https://amzn.to/2vSzzX8) -


## Base
- Steel plate ½ inch thick 31” diameter
- 6 - Coupler https://amzn.to/2slOiIa

## Drive
* [6 - 750w AC servo Motors](https://www.aliexpress.com/item/32844239563.html)
* [6 - 50:1 Planetary Gears](https://www.aliexpress.com/item/32967571001.html)
*Note ensure planetary gear input diameter matches up to both the motor as well with the coupler output diameter when ordering from Aliexpress

## Connecting Arms

* [12 - 1/2 X 1/2-20 Economy Panhard Bar Kit with Bung .065, Rod End, Heim Joint](https://amzn.to/2FQffak)
* [12 - 1/2-3/8 High Misalignment Spacers, Rod End Spacers](https://amzn.to/2tm1jlF)
* [6 - 24" long 1" OD X .870 ID X .065 Wall Steel tubing]()

## Swing Arms
- 6 - 8" long 1" OD X .870 ID X .065 Wall Steel tubing
- 6 - 3/8"-16 Long Coped Steel Bungs

## Chassis
* [Vesa Monitor mount](https://amzn.to/2TmVS0f)
* [Coped Steel Bungs](https://amzn.to/2TGOcoo)
* [1" OD X .870 ID X .065 Wall Steel tubing](https://amzn.to/3au4FCQ)

## Extras as built in demonstration video
* [LG 34" Ultrawide](https://amzn.to/2t8YvbC)
* [Thrustmaster T16000M FCS](https://amzn.to/30qkHtY)
* [Wind Generator Fan](https://amzn.to/36W1um9)
* [Wind Generator PWM Control](https://amzn.to/2Ns1anq)
* [Wind Generator 90 Degree angle 3"-> 2" Reducer](https://amzn.to/2uN6J9z)
* [Wire Wrap](https://amzn.to/2u3jiNu)

## AC Servo motor settings
These are my settings on the Servo Driver aasd-15a these both enable specific modes as well define the time it should take to accelerate and decelerate the platform before it hits max speed. This is useful for when you want to protect the platform from self destruction due to to fast of movements. Make them to large and the platform will feel slugish.
- pn002 - Control Mode - "002"
- pn003 - Servo enable - "001"
- pn098 - Gear - "80"
- pn109 - Position command deceleration mode- "002"
- pn110 - Position command a filtering time constant - "050"
- pn111 - S-shaped filtering time constant Ta position instruction - "300"
- pn112 - position instruction Ts S-shaped filtering
time constant Ts - "150"

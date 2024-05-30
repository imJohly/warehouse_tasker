# Payload Mechanism and Wiring Diagram for Warehouse Robot

![Wiring_Diagram.jpg](Schematic/Wiring_Diagram.jpg)

## Hardware
There are two parts of hardware: the electrical and the mechanical.

The mechanical components are all 3D printed, and the STEP files are located in the "Step Files" folder under the payload branch.

For the mechanical/3D prints you will need:
- 1x box
- 1x Door_full OR Door_Quick
- 1x Gear
- 2x Pole

For the electrical you will need:
- 1x continuous 9g servo
- 1x Arduino Nano
Recommended:
- 1x resistor
- 3x LED (varying colors)
- 1x button

### Software

The software is located in the "Arduino Code" folder.

It is basic code that activates the servo controlling the door for a set time when a high signal from either the button or the Raspberry Pi is received.

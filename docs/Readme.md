## MPU6000 - Datalogger


A datalogger that tracks and stores 16 hours of wrist motion data in an effort to create algorithms that can detect eating. The device uses an MSP430 for its microcontroller, an MPU6000 for IMU sensing (accelerometer and gyroscope), an Adesto Dataflash chip for memory (Atmel Serial Flash), and an FT232 UART bridge for communication. This was a Masters project at Clemson University to create a datalogger that tracks and stores 16 hours of wrist motion data in an effort to create algorithms that can detect eating.

See [Bite Counter project](http://cecas.clemson.edu/~ahoover/bite-counter/) for more details.

### Folder Layout
- Datasheets
- Reference files used for design decisions and programming
	- Includes datasheets for parts used, an approximate bill of materials and comparison of IMU sensors
- PCB Layout
	- Eagle PCB schematic and board files used for protypting
	- Also contains a 3D rendering of final board, and a final schematic png for reference
- Thesis and Presentation
	- DefensePresentation.pdf - Slides used for Masters defense
    - thesis.pdf - Final thesis presented to the Clemson University Graduate School
- code
	- Contains code to be compiled using IAR Workbench for the MSP430F247
- docs
	- Files for this readme
    
## Final Prototype
A final prototype was built using OSHPark's PCB service, and parts were soldered using a hot skillet. See slides for details.
The device was housed in a plastic body as shown in docs/FinalDev.jpg

![](https://raw.githubusercontent.com/iamsurya/MPU6000-Datalogger/master/docs/FinalDev.jpg)

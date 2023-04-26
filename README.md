# ET7190-Arduino
functions for the ET7190 OBDII chipset written/translated to be used with the arduino platform

This code **IS NOT** a complete working sample, I have only taken the code provided by the manufacturer and modified it to work with the Arduino IDE. In order to use this code you will have to make midifications for your particular code base, you will have to provide declares for some variables, etc. You can easily find these things by trying to compile the code and looking at the errors. If you don't know how to do this then this code might be too advanced for you.

The spiFunctions are a little more mature, while I believe the uartFunctions are pretty much correct, I know that the spiFunctions are correct as I use them regularly in projects.
There are some functions that are not ET7190 specific and are not in the files (bufferToZero is one), and some variables declares are also not in these files. 

Again this is **not** a complete sample project that shows how to use these functions with the ET7190 chipset...

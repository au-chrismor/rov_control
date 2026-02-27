# Electronics Pod Testing

## Introduction

Because this is a sealed assembly, it is essential that the components be thoroughly tested *before* final assembly takes place.  As the thruster motors can't operate out of water for more than a minute or so, heavy 12V gear motors should be connected in place.  These can be operated for much longer periods of time.

## Post-assembly

1.  With the output disconnected, connect the PSU input to 12V DC and adjust the output for 5.1V output.  This allows for a little output "sag" under heavy load, although this is not expected in the current design.  Disconnect power after testing.

2.  Connect the resistive divider to the battery input.  Check the the output does not exceed 5V.

3.  Connect the current sensor in-line with the 12V supply *before* both the PSU and motor controllers.  Connect the power supply pin of the current sensor to 5V.  With power applied, the current sensor should read approximately 2.5V (0mA).

4.  Connect the pressure sensor to the 5V rail and measure the output.  At sea level the output should be close to 0.5V.

5.  Connect the IIC peripherals to the Arduino Mega.  Connect the GND input of the Arduino Mega to the PSU output, but *do not* connect the 5V at this time.  The board can be powered from the host USB for now.  Using the Serial Monitor application, check for errors at startup.

6.  Connect motors to controllers.  Run each motor to verify operation.


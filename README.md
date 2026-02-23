# ROV_Control

This is a set of Arduino sketches for controlling a "SeaPerch-like" simple ROV (Remotely Operated Vessel)

It is almost certainly overkill, but it came about from a logical problem.  Most ROV builders use Ethernet cables (4-pair) to connect to their machines from the surface.  Because I have three independent motors, this left me with no expansion beyond running another multi-pair cable.  I had also considered using relays mounted on the unit to control the motors, but they would cost more than an Arduino and motor drivers.

So I hit on the idea of using RS-485 which is suitable for cable runs in the hundreds of metres and a simple control protocol.

Since now I have considerable processing capabilities on the ROV itself, I thought I would add sensors so I could determine what was happening.  The sensors are:

- HMC5883 Compass;
- MPU650 3-Axis Accelerometer;
- ACS712 Current Sensor;
- Resistive Pressure Sensor (Denso oil pressure sensor);
- Divider network for battery voltage


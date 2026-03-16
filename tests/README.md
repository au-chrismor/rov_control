# Testing tools

The tools in this directory have proven useful while I was developing the controller.  They are general simple with not a lot of functionality in a given tool.

They are generally written to run on a basic Arduino board, but could be adapted to ESP32 without too much effort.

## bts7960test

I wrote this to familiarise myself with the H-Bridge controllers.  It runs a single motor in forward and reverse directions, and demonstrates a soft-start functionality.

## depthtest

One of the challenges is to calibrate the depth sensor.  My solution was to build a cut-down version of the electronics pod with an Arduino Nano, pressure sensor and thermistor.

This also proved useful in testing my potting and sealing techniques.

I then weighted the pod and attached to to a length of coard with marks every 10cm.  I could lower the pod into my swimming pool and record the readings.

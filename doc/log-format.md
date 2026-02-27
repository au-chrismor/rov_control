# LOG FORMAT

Log data is returned in response to a "LOG" command from the console.  The data is a JSON-formatted block as shown below:

{

    "log": {
        "volts": 0.0,
        "amps": 0.0,
        "motor_l": 0,
        "motor_r": 0,
        "motor_v": 0,
        "temperature": 0.0,
        "accel_x": 0.0,
        "accel_y": 0.0,
        "accel_z": 0.0,
        "gyro_x": 0.0,
        "gyro_y": 0.0,
        "gyro_z": 0.0,
        "mag_x": 0.0,
        "mag_y": 0.0,
        "mag_z": 0.0,
        "heading": 0.0,
        "light": "off",
        "pressure": 0,
        "moisture": 0,
        "alarm": "true",
        "alarm_data", 2
    }
}

## Definitions

### accel_x

Longitudinal accelerometer reading in "m/s/s".

### accel_y

Lateral accelerometer reading in "m/s/s".

### accel_z

Vertical accelerometer reading in "m/s/s".

### alarm

"true" or "false".  If set to "true", and alarm state exists within the pod, and the "alarm_data" key will be present.

### alarm_data

Each bit indicates a specific alarm.

0 = Moisture

1 = Low Battery Voltage

### amps

The DC Supply current

### gyro_x

Longitudinal gyroscope output.  Indicates current direction of travel.

### gyro_y

Lateral gyroscope output.  Indicates sideslip.

### gyro_z

Vertical gyroscope output.  Indicates ascent/descent.

### heading

Present compass bearing in degrees.  Relative to true North if the declination is non-zero.

### light

State of the external light:  "on" or "off"

### mag_x

Longitudinal magnetometer reading in uTeslas.

### mag_y

Lateral magnetometer reading in uTeslas.

### mag_z

Vertical magnetometer reading in uTeslas.

### moisture

Moisture sensor ADC output.  Used for leak detection.

### motor_l

Current operating state of the left horizontal motor:

0 = Off
1 = Forward
2 = Reverse

### motor_r

Current operating state of the right horizontal motor:

0 = Off
1 = Forward
2 = Reverse

### motor_v

Current operating state of the vertical motor:

0 = Off

1 = Forward

2 = Reverse

### pressure

External pressure sensor ADC output.  Requires conversion to depth.

### temperature

Current internal temperature.  Measured by IMU module.

### volts

The DC Supply voltage.


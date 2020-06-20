# leptorino torque and force sensor

## Usage

```
sudo chmod 777 /dev/ttyACM0
```

```
rosrun sia20_recognition leptorino
```

## topic

leptorino node sends `/sensor_data`
The topic is published 100 Hz

## Calibration

Using start 100 times sensor data, the bias is calclated.

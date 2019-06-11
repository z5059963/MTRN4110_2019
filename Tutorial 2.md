# MTRN4110 Tutorial 2 - Hardware

# Ultrasound module
![](https://core-electronics.com.au/media/catalog/product/cache/1/image/650x650/fe1bcd18654db18f328c2faaaf3c690a/d/e/device6_1000.jpg)

Ultrasound module is a sonar that can be used to measure the distance from the sensor to an obstacle. The module sends serveral ultrasonic pulses and detect the echo. The time take to recieve the returning signal can be used to calculate the distance to the obstable using the speed of sound.

The model of the ultrasound module used in the lab is HC-SR04. [Datasheet](https://raw.githubusercontent.com/zhihao50/MTRN4110_2019/master/Datasheet/Ultrasound.pdf)

## How to use:
1. Send logic level high signal to trigger pin for at least 10us.
2. The Module automatically sends eight 40 kHz and detect whether there is a pulse signal back.
3. When the bounced back signal is received the ultrasound module will send a pulse on the echo pin with the duration of the pulse equal the time of flight of the ultrasound pulse.
4. Measure the duration of the pulse using Arduino. Distance = high level time * speed of sound / 2. Speed of sound is approximately 340m/s. Note 2 / (340,000mm/s) * 1,000,000us/s = 5.88

# Ineritial measurement unit
## Accelerometers
Accelerometer measures both static (eg gravity) and dynamic acceleration. Most common accelerometers measures 3 axis, x,y and z.  Accelerations can be intergrated once to get the velocity and then again to get the position values, but beware errors will accumulate especially after double intergration. The static accelerations and orientation of the module also need to be taken into account when integrating acceleration.

## Gyroscopes
Gyroscope measures angular velocity. A 3dof gyroscope measures the roll (x), pitch(y) and yaw(z) axis. the gyroscope measure with respect to the current orientation, when moving in 3d space, all three axis need to be intergrated together to get the orientation. 

## Magnetometers
Magnetometer measure the strength and direction of the magnetic field. Earth magnetic field can be used to find the current heading. Magnetometer should be placed as far away from local magnetic inteference (such as ferrous metal or motor) as possible.


## IMU Advanced techniques
Even when there is no movement, the accelerometers and gyroscopes reading will still give an non zero average value due to imperfaction in the sensor, this is know as sensor bias. The bias can be estimated by taking the average value over a period when the sensor is stationary. 

6dof inertial measurement unit combines a 3dof accelerometers with 3dof gyroscope. The accelerometer can be used to estimate the absolute roll and pitch angle by measuring the direction of gravity when stationary. Roll and pitch can be calculated using the fomular:
```
pitch = atan2(ax, sqrt(ay*ay + az*az));
roll = atan2(ay, sqrt(ax*ax + az*az));
```

9dof IMU adds a 3dof magnetometers to estimate the absolute yaw angle. A barometer is added to form a 10dof that can also measure altitude.

## Model
The model of the IMU module used in the lab is MPU-6050. [Product page](https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/) It is a 6dof IMU with integrated digital motion processsing unit that can be used to offload the high frequency IMU processing. The dmp will process IMU reading at 200hz, performing sensor fusion of accelerometer and gyroscope to produce a more accurate result. The dmp will output the result at a slower rate into a 1024 bytes first in first out (fifo) buffer. The DMP output rate is configured in firmware. The library we use is a slightly modified version of [i2cdevlib](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050). The dmp output rate is set at 25Hz.

![](https://core-electronics.com.au/media/catalog/product/cache/1/image/650x650/fe1bcd18654db18f328c2faaaf3c690a/d/e/device13_1000.jpg)

## How to use
1. Include everything starting with external_MPU6050
2. Create a MPU6050 object.
3. Enable the interrupt pin.
4. Call `initialize()` method on the MPU6050 object
5. Call `dmpInitialize()` method on the MPU6050 object, the method return 0 if successful.
6. If dmp is initialized successfully enable dmp by called `setDMPEnabled()` method.
7. clear fifo buffer using `clear_fifo_buffer()`.
8. When there is a packet ready in the fifo buffer, a interrupt will be generated. There are other condition that will generate interrupt.
9. The status code of the interrupt can be checked using `getIntStatus()` method.
10. Size of fifo packet can be checked using `dmpGetFIFOPacketSize()` method.
11. Current number of bytes in the fifo buffer can be checked using `getFIFOCount()` method.
12. A packet can be read using `getFIFOBytes(uint8_t* data,uint8_t length)`
13. Example of using MPU6050 from I2cdevlib can be found [here](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050/examples/MPU6050_DMP6)

# LiDAR
![](https://cdn-learn.adafruit.com/guides/cropped_images/000/001/520/medium640/top.jpg?1515090119)

One method of measuring distance using light is by measuring the time of flight for the light to be reflect back to the sensor. VL6180x is a sensor from STMicroelectronics that combines an IR emitter, rnage sensor and an ambient light sensor. The module used in the lab is the breakout board from Adafruit [link](https://www.adafruit.com/product/3316). 

The header pins are:
| Name | Function |
| --- | --- |
| VIN | Power 3-5V |
| 2v8 | Internal 2.8v regulator output |
| GND | Ground |
| GPIO | Can be configured as an interrupt |
| SHDN | Disable the module when pulled low |
| SCL | I2C clock |
| SDA | I2C data |

VL6180X can be used with Pololu VL6180X library provided to you in external_VL6180X.h and external_VL6180X.cpp. Read the comments in external_VL6180X.cpp to see what each function do. 


# Encoder
![](https://dfimg.dfrobot.com/data/SEN0116/20140709/_DSC0669.jpg?imageView2/1/w/564/h/376)
The model of the encoder module used in the lab is DFRobot Sen0116. [Link](https://www.dfrobot.com/product-823.html) 

The sensor use two IR reflective sensor. The sensor emits IR light, when the light is reflected back, the IR sensor will output high. The wheels we use have encoder teeth build in. The two sensors are positioned so the signal waveform is at a phase difference of 90 degrees. This means one of the sensor will change before the other one. The order of the signal change can be used to figure out the direction of travel.  Experiment with the encoder to find the corresponding logic. One method can be to look at the generated waveforms using an oscilloscope.

## How to use
Pick one of the pins as interrupt pin. When the pin logic level changes, check both pin to determine the direction of travel.

## Tuning
The sensitivity of the optical sensor can be adjusted by the potentiometer near the signal connector. The potentiometer can be adjusted to between 0-30KΩ. Use the oscilloscope, multimeter or Arduino to read the output voltag. Adjust the encoder so that it output high only when the teeth is over the encoder.

# Motor Driver
![](https://dfimg.dfrobot.com/data/DRI0009/53AU4096.jpg?imageView2/1/w/564/h/376)

Motor can draw a level of current exceeding the maximum supply current of Arduino voltage regulator. A dedicated motor driver shield can be used to supply power to the motor. The motor driver also allow switch the direction of the motor by changing the applid voltage polarity on the motor. 

The model of the motor driver shield used in the lab is DFRobot DRI0009. [Link](https://www.dfrobot.com/product-69.html). [More info](https://wiki.dfrobot.com/Arduino_Motor_Shield__L298N___SKU_DRI0009_). The motor driver use a dual full H-bridge chip (L298N) chip. Note: pay attention to the voltage drop due to the H-brdige. See datasheet for L298N and try take measurement in the lab.

The motor shield can controller two motors, each motor is controlled by two pins, one for speed and one for direction. The pin used is in the following table. 

| Pin | Motor | Function | 
| --- | --- | --- |
| 4 | 2 | Direction |
| 5 | 2 | speed |
| 6 | 1 | Speed |
| 7 | 1 | Direction |

The shield can take external power by changing the power source selection pin ![](https://raw.githubusercontent.com/DFRobot/DFRobotMediaWikiImage/master/Image/Arduino_Shield6.png)

Note: Pay attention to the 

## How to use multiple sensors.
Since the sensor have the same default I2C address, to use more the one VL6180X, the sensor I2C address need to be changed at the start of the program. The sensor need to be kept in shutdown mode until its address have changed. See  `void setAddress(uint8_t new_addr)` on how to change the sensor address.

# Display
![](https://dfimg.dfrobot.com//data/DFR0556/DFR0556-family_564x376.jpg?imageView2/1/w/564/h/376)

The model of the display module used in the lab is DFRobot Gravity: I2C LCD1602 Arduino LCD Display. [Link](https://www.dfrobot.com/product-1723.html) 

Communicate with the display is done via I2C port.

The display can be used with the DFRobot display library. `#include "external_DFRobot_LCD.h"` and add `external_DFRobot_LCD.cpp` to the solution. The LCD display class is `DFRobot_LCD`. Use the method `init()` to initialise the display.

The position of the cursor can be set with `setCursor (uint8_t column, uint8_t row)`.

The class have `print` and `write` method simular to Serial.

LCD screen can be cleared with `clear()` method.

# Bluetooth
![](https://core-electronics.com.au/media/catalog/product/cache/1/image/650x650/fe1bcd18654db18f328c2faaaf3c690a/0/1/018-hc-06_1.jpg)

Bluetooth is a wireless communication technology that is designed to work over short distance. For this course we will use one of the mode of Bluetooth called Serial Port Profile (SPP) that can be used just like a serial port.

The model of the Bluetooth module used in the lab is HC-06. [Datasheet](https://www.olimex.com/Products/Components/RF/BLUETOOTH-SERIAL-HC-06/resources/hc06.pdf) 

The bluetooth devices first need to be paired together. This can be done in your devices Bluetooth setting. A pin is required, all Bluetooth module in the lab have name of the format "PIN100X" where 100X is the pin number of that specific device. __Note: apple phone dont appear to work with HC-06__

The bluetooth module simply work as a serial device, RX and TX pin should be connected to the approperate serial port on Arduino. Communication is simply done via Arduino serial library. The HC-06 provided are configured for serial speed of 115200. The bluetooth module will appear as a serial port. The device can be tested using a serial terminal, or on a phone with a Bluetooth serial terminal such as Serial Bluetooth Terminal from Play Store. 
To configure the Bluetooth device. 

## Device Config
The HC-06 provided in lab can be configured when it is not being connected to another Bluetooth device. Make sure the serial terminal is set to __No line ending_. The following commands can be used

| Command | Response | Action |
|---|---|---|
| AT | OK | |
| AT+NAMEzzzz | OKsetname | Set the display name to zzzz |
| AT+PINzzzz | OKsetPIN | Set the pin to zzzz |
| AT+BAUD2 | OK9600 | Set the serial rate to default rate (9600) |
| AT+BAUD8 | OK115200 | Set the serial rate to 115200 |

The version of HC-06 create sells use an different command set. Make sure the serial terminal is set to __Both NL & CR__. The following commands can be used

| Command | Response | Action |
|---|---|---|
| AT | OK | |
| AT+NAME | +NAME:zzzz | Returns the current display name zzzz |
| AT+NAME=zzzz | OK | Set display name to zzzz |
| AT+PSWD | +PIN:"zzzz" | Return  pin number|
| AT+PSWD="zzzz" | OK | Set pin number |
| AT+UART | +UART:speed,0,0 | Return serial speed |
| AT+UART=115200,0,0 | OK | Set serial speed |

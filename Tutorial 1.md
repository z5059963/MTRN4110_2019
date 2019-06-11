# MTRN4110 Tutorial 1 - Arduino
* Official Arduino tutorial can be found [here](https://www.arduino.cc/en/Tutorial/BuiltInExamples)
* Arduino library documentation can be found [here](https://www.arduino.cc/reference/en/).

This course choose to use Arduino platform because it quick and convenient, allowing rapid development of prototype with in the short 10 Weeks course. Most Arduino platforms are based on Atmel AVR microcontroller. There are other Arduino compatible platform based on ARM, ESP8266 and ESP32 microcontroller. 

This tutorial aims to provide a quick refresher on the Arduino libary. Students without prior experience are encouraged to have a look at the official Aruino tutorial linked above. 

# Digital and Analog outputs I/O
AVR microcontroller have a large number of general purpose input output (GPIO) pins that are grouped into ports. Addressing each pins are done by accessing the specific bits of the port register. Arduino functions allow easy linear address of the IO pins using the header pin number. 

Some of the pins have alternative functionality, refer to the arduino pinout ([Uno](https://www.arduino.cc/en/Hacking/PinMapping168), [Mega](https://www.arduino.cc/en/Hacking/PinMapping2560)). 

## Digital
All digital pins can be configured as input with high impedance state, input with internal pull up resister connected to VCC, or output acting as either a source or sink of current. More detailed tutorial on Arduino digital I/O can be found [here](https://www.arduino.cc/en/Tutorial/DigitalPins). The I/O mode for each individual pin can be selected using
```C++
void pinMode(uint8_t pin, uint8_t mode);
```
where ```pin``` is the pin number of the I/O pin, and mode can be 
* ```INPUT // Read mode, default mode.```
* ```OUTPUT // Write mode.```
* ```INPUT_PULLUP // Read with internal pullup resister activated. ```

Digital input can be read using
```C++
int digitalRead(uint8_t pin);
```
Where ```pin``` is the pin number, return ```value``` defined by macro ```LOW``` or ```HIGH```.

Digital output can be performed using
```C++
void digitalWrite(uint8_t pin, uint8_t value);
```
Where ```pin``` is the pin number, and ```value``` defined by macro ```LOW``` or ```HIGH```.

## Analog
Digital pin can be used emulate an analog output by toggling between on and off, a technique known as power width modulation, details can be found [here](https://www.arduino.cc/en/Tutorial/PWM). PWM output write function is
```C++
void analogWrite(int pin, int value)
```
Where ```pin``` is the pin number, and ```value``` is the duty cycle between 0 (always off) and 255 (always on).

Some of the digital I/O pins are also capible of analog inputs, see details [here](https://www.arduino.cc/en/Tutorial/AnalogInputPins). Those pins can be accesed using macro `PIN_AXX` were XX is the analog pin number. Analog voltage can be read using
```C++
int analogRead(uint8_t pin);
```
Where ```pin``` is the pin number. For AVR based arduino the function return a 10 bits number (0-1023) representing the analog input voltage depending on the reference voltage set using 
```C++
void analogReference(uint8_t mode);
```
## Interrupt
Some of the Arduino digital pins are interrupt capable, those pins can be used to monitor an external input without constantly checking the pin. When a trigger condition is meet, the an interrupt is raised and program execution is switched to an interrupt service routine (ISR) to handle the input. More details on interrupt can be found [here](https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/). An interrupt service routine can be used attached to the interrupt pin using the function 
```C++
void attachInterrupt(uint8_t interrupt, void (*ISR)(void), int mode);
```
where interrupt is the interrupt number that can be found by using the macro ```digitalPinToInterrupt(pin)```, ```void (*ISR)(void)``` is a function pointer to the ISR with signature 
```C++
void function_name () 
{
  /// ISR code
}
```

, and mode can be macros ```LOW```, ```CHANGE```, ```RISING```, and ```FALLING```.


Sidenote: Arduino digital I/O functions have high overhead cost, with performance ~20x slower than direct access.

# Clock
Number of microseconds since the microcontroller started can be read using [micros()](https://www.arduino.cc/reference/en/language/functions/time/micros/
). The number will overflow after 70 minutes. Number of milliseconds since the microcontroller started can be read using [(millis()](https://www.arduino.cc/reference/en/language/functions/time/millis/). Those value will overflow after approximately 50 days.
```C++
unsigned long millis(void);
unsigned long micros(void);
```

Program execution can be paused for X milliseconds using the function 
```C++
void delay(unsigned long time ms);
```
Program execution can be paused for X microseconds using the function 
```C++
void delayMicroseconds(unsigned long us);
```

# Serial Communication
Arduino Serial documentation is [here](https://www.arduino.cc/reference/en/language/functions/communication/serial/). Serial communication transmit on the TX pin and recieve on the RX pin, therefor the TXn pin of the Arduino board need to be connected to the RX pin of other board, RXn connect to TX pin. n is serial number 0,1,2... Arduino serial uses dedicated hardware universal asynchronous receiver-transmitter (UART) units. Most Arduino have at least 1 hardware serial that is represented by the global object ```Serial``` also known as serial 0 (TX0, RX0). Ardiono mega have additional hardware serial ```Serial1,Serial2,Serial3```. Arduino also have software serial library that can emulate serial port, see [here](https://www.arduino.cc/en/Reference/SoftwareSerial).

Arduino IDE have a serial terminal windows under tools -> serial monitor.

Hardware serial libary can be added by ```#include <HardwareSerial.h>```. Software serial can be added by ```#include <SoftwareSerial.h>```.

The speed of serial can be set using ```Serial.begin(unsigned long baud);```. Speed of serial need to match on the sending and recieving side.

Serial communication can be stopped and disabled using ```Serial.end();```.

Recieve serial data by using the read method
```C++
virtual int read(void);
Serial.read()
```

Send serial data as raw bytes using the various write function
```C++
Serial.write(val)
Serial.write(str)
Serial.write(buf, len)
```

Formatted print using print or println function. See [here](https://www.arduino.cc/reference/en/language/functions/communication/serial/print/).

# I2C
I2c also known as IIC, two wire, TWI is a serial communication protocal that allow multiple deviced to be connected to the same bus. Individual devices are assigned addresses that can be used to select the device for communication. See device datasheet for the default address and method to change the address when using multiple devices. The two wires are SDA and SCL, connect all SDA together and all SCL together.

Arduino I2C library can be included by ```#include <Wire.h>```.

Wire libary is initialised by calling ```Wire.begin(address)``` where address is optional. If no address is specified, the Arduino act as the master device.

The clock rate of I2C bus can be set using ```WireSetClock(uint32_t clock)```. Typical clock rate is 400000.

A transmission starts by calling```Wire.beginTransmission(address)```to select the slave device.

Data can be sent using 
```C++
Wire.write(value) 
Wire.write(string) 
Wire.write(data, length)
``` 
and recieved using 
```C++
Wire.read();
```

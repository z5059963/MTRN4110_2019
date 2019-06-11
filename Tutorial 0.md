# MTRN4110 Tutorial 0 - Getting Started with Arduino
Author: Subhan Khan.

The Arduino MEGA or MEGA 2560 is designed for projects that require more I/O lines, more sketch memory and more RAM. With 54 digital I/O pins, 16 analog inputs and a larger space for your sketch it is the recommended board for 3D printers and robotics projects. This gives your projects plenty of room and opportunities maintaining the simplicity and effectiveness of the Arduino platform. This document explains how to connect your MEGA or MEGA2560 board to the computer and upload your first sketch. The Arduino Mega2560 board is programmed using the Arduino Software Integrated Development Environment (IDE). Following steps will enable you to start-up your first sketch:

## Step-I
Connect your MEGA or MEGA2560 board with an A B USB cable; sometimes this cable is called a USB printer cable. The USB connection with the PC is necessary to program the board and not just to power it up. The MEGA or MEGA2560 automatically draw power from either the USB or an external power supply. Connect the board to your computer using the USB cable.  The green power LED (labelled PWR) should go on.

## Step-II
If you used the Installer, Windows - from XP up to 10 - will install drivers automatically as soon as you connect your board. If you downloaded and expanded the Zip package or, for some reason, the board wasn't properly recognized, please follow the procedure below.

* Click on the Start Menu and open-up the Control Panel.
* While in the Control Panel, navigate to System and Security. Next, click on System. Once the System window is up, open the Device Manager.
* Look under Ports (COM & LPT). You should see an open port named "Arduino Mega (COMxx)". If there is no COM & LPT section, look under "Other Devices" for "Unknown Device".
* Right click on the "Arduino MEGA or MEGA2560 (COMxx)" port and choose the "Update Driver Software" option.
* Next, choose the "Browse my computer for Driver software" option.
* Finally, navigate to and select the driver file named "arduino.inf", located in the "Drivers" folder of the Arduino Software download (not the "FTDI USB Drivers" sub-directory). If you are using an old version of the IDE (1.0.3 or older), choose the MEGA or MEGA2560 driver file named "Arduino MEGA2560.inf"
* Windows will finish up the driver installation from there.

## Step-III
Run the Arduino IDE and open the LED blink example sketch: **File > Examples >01.Basics > Blink**. 
![alt text](https://raw.githubusercontent.com/drliaowu/MTRN4110_2019/master/image/img1.png "File > Examples >01.Basics > Blink")
You'll need to select the entry in the Tools > Boardmenu that corresponds to your Arduino orGenuino board.
![alt text](https://raw.githubusercontent.com/drliaowu/MTRN4110_2019/master/image/img2.png "Tools > Boardmenu that corresponds to your Arduino orGenuino board")
Select   the   serial   device   of  the   board  from   the  Tools   |   Serial   Port   menu.  This   is   likely   tobeCOM3or higher  (COM1andCOM2are  usually reserved  for hardware serial ports). Tofind   out,  you  can  disconnect   your  board  and  re-open   the  menu;  the   entry  that  disappearsshould be the Arduino board. Reconnect the board and select that serial port.
![alt text](https://raw.githubusercontent.com/drliaowu/MTRN4110_2019/master/image/img3.png "FTools -> Port -> Select your board")

### Basic autonomy of Arduino program

There are two special functions:
```C++
void setup() 
{
  // put your setup code here, runs once 
}

void loop() 
{
  // put your main code here, to run repeatedly
}
```
Put your setup code in `setup()` function and main code in `loop()` that will be repeatedly run.

## Step-IV
Now, simply click the "Upload" button in the environment. Wait a few seconds - you shouldsee the RX and TX leds on the board flashing. If the upload is successful, the message "Doneuploading." will appear in the status bar.
![alt text](https://raw.githubusercontent.com/drliaowu/MTRN4110_2019/master/image/img4.png "Upload code")

A few seconds after the upload finishes, you should see the pin 13 (L) LED on the board startto blink (in orange). If it does, congratulations! You've gotten Arduino MEGA2560 up-and-running. If you have problems, please see thedemonstrator.

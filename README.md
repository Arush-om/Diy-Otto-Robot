# Diy-Otto-Robot
Otto is an interactive robot that anyone can make!

![Screenshot (1056)](https://github.com/Arush-om/Diy-Otto-Robot/assets/134673721/002e6f9f-8f90-4c24-b23f-d763a4711ef0)

If you have hard-time 3d printing stuff and other materials which i have provided in this project please refer the professionals for the help, [JLCPCB](https://jlcpcb.com?from=ayu ) is one of the best company from shenzhen china they provide, PCB manufacturing, PCBA and 3D printing services to people in need, they provide good quality products in all sectors

[JLCPCB](https://jlcpcb.com?from=ayu)


Please use the following link to register an account in [JLCPCB](https://jlcpcb.com?from=ayu )

https://jlcpcb.com?from=ayu 


Pcb Manufacturing

----------

2 layers

4 layers

6 layers

jlcpcb.com/RNA



PCBA Services

[JLCPCB](https://jlcpcb.com?from=ayu ) have 350k+ Components In-stock. You don’t have to worry about parts sourcing, this helps you to save time and hassle, also keeps your costs down.

Moreover, you can pre-order parts and hold the inventory at [JLCPCB](https://jlcpcb.com?from=ayu ), giving you peace-of-mind that you won't run into any last minute part shortages. jlcpcb.com?from=ayu 


3d printing

-------------------

SLA -- MJF --SLM -- FDM -- & SLS. easy order and fast shipping makes [JLCPCB](https://jlcpcb.com?from=ayu ) better companion among other manufactures try out [JLCPCB](https://jlcpcb.com?from=ayu ) 3D Printing servies

[JLCPCB](https://jlcpcb.com?from=ayu ) 3D Printing starts at $1 &Get $54 Coupons for new users

Otto is an interactive robot that anyone can make!, Otto walks, dances, makes sounds and avoids obstacles. Otto was inspired by another robot instructable BoB the BiPed and programmed using code from another open source biped robot called Zow

All The 3D Printable Files Are Included In The Repository Other Variants Can Be Found HERE & Official OTTO STORE

INCLUDE THE OTTO DIY LIBRARY

USE THIS CODE FOR CALIBRATION , THIS CODE FOR DANCE MOVES

![Screenshot (1058)](https://github.com/Arush-om/Diy-Otto-Robot/assets/134673721/ac0241ae-a6bf-4977-99b1-4ad5def95474)
![Screenshot (1059)](https://github.com/Arush-om/Diy-Otto-Robot/assets/134673721/383a48b8-dd13-4ae8-bd7b-25d34838eed7)

Otto's differences are in the assembled size (11cm x 7cm x12cm), cleaner integration of components and expressions. Using off the shelf and 3D printed parts, simple electronics connections (almost no welding required), and basic coding skills, you will be able to build your own cute Otto friend in as little as two hours!

Otto is designed using Autodesk 123D Design software (now thinkercad) you can modify it for customization or further improvements!

List of parts:

Arduino Nano + USB A to Mini-B cable
Arduino Nano Shield I/O Extension Board
HC-SR04 Ultrasound sensor.
Mini servo MG90S x4 (each one comes with 2 pointed screws and one small screw)
5V passive Buzzer.
Female to Female breadboard connectors cable
8.5x8.5mm self-locking Switch.
HC-06 Bluetooth module
MAX7219 LED Matrix module (SDM)
Sound sensor (using LM393) with Analog output
Light sensor (using LM393) with Analog output
YFRobot touch sensor
Rechargeable Battery with separate charging and discharging wires + balance charger
Mini cross screwdriver. (magnetized)
II. 3d printing plastic parts :

3D printed head.
3D printed body.
3D printed leg x2.
3D printed feet.
III. Assemble guide, Software and App:

Otto++ can be programmed by Arduino and Scratch

![Screenshot (1060)](https://github.com/Arush-om/Diy-Otto-Robot/assets/134673721/4d6d2071-034b-4d7b-93f2-f02984c8ce39)

Print Settings

Printer:

Mega I3 (Prusa Mendel I3)

Rafts:

Doesn't Matter

Supports:

Doesn't Matter

Resolution:

0.15mm

Infill:

25%

Notes:

Recommended to use a FDM 3D printer, transparent PLA for the body (to support light sensor) and colorful PLA for others.

Head rounded, with 2 types of Bodies with LED matrix on the inside or revealed through a hole, there are 3 types of detachable hands, that can hold things, like wands. There are 2 types of legs, short (normal size) and tall and the popular eye holder is at the bottom of list.

hence, there are fewer data to encode per second. Compared to lower SF, sending the same amount of data with higher SF needs more transmission time, known as airtime. More airtime means that the modem is up and running longer and consuming more energy.

![Screenshot (1064)](https://github.com/Arush-om/Diy-Otto-Robot/assets/134673721/98ca1b94-f6fb-4a91-a105-0427035e19fd)![Screenshot (1061)](https://github.com/Arush-om/Diy-Otto-Robot/assets/134673721/c905fcfc-64c6-452e-931d-f4ae34ea8f62)
![Screenshot (1062)](https://github.com/Arush-om/Diy-Otto-Robot/assets/134673721/57c73710-4fb6-4b82-8609-75e397616e22)
![Screenshot (1063)](https://github.com/Arush-om/Diy-Otto-Robot/assets/134673721/687ff784-969f-4530-ab91-6eac9d442686)
 

Analog joystick produces two voltages; one corresponding to position with respect to X-axis and another corresponding to the position with respect to Y-axis. The voltages produced depend on the position of the joystick.

For more information about Analog Joystick and how to use it, refer the topic Analog Joystick in the sensors and modules section.

To interface the Analog Joystick with Arduino Uno, we need to use ADC on the microcontroller of the Arduino UNO board

The potentiometers act as voltage dividers. This module produces an output of around 2.5V from X and Y when it is in the resting position. Moving the joystick will cause the output to vary from 0v to 5V depending on its direction. If you connect this module to a microcontroller, you can expect to read a value of around 512 in its resting position (expect small variations due to tiny imprecisions of the springs and mechanism). When you move the joystick you should see the values change from 0 to 1023 depending on its position. The given example values are for a 5V microcontroller or a development board with 10bit ADC resolution like Arduino UNO or nano

The Joystick itself only contains the two potentiometers for each axis and a switch to register the click. One side of the potentiometers is connected to the ground and the other to the VCC. The center pins are connected to the VRx and VRy pins respectively. Similarly, the switch is connected between the GND and the SW pins. And in some modules, there is also an unpopulated space for a pull-up resistor on board for the switch. 

Simply connect the first pin on the left to 3-5V power, the second pin to your data input pin and the rightmost pin to ground. Although it uses a single-wire to send data it is not Dallas One Wire compatible! If you want multiple sensors, each one must have its own data pin.



Power

VCC: power pin. The working voltage for the chip is 1.71-3.6V. Since the module integrates a 3.3V voltage regulator, the power supply can be either 3.3V or 5V. If you are using an Arduino board, you're recommended to use a 5V power supply.

3.3V: the output of the voltage regulator, meaning you can provide the chip with a 3.3V for power here.

GND: common ground for power and logic.BMP280 supports I2C and SPI communication and the module keeps both ports. If you want to connect a simple circuit, you can use the I2C port; to connect multiple sensors, you can use the SPI port free of I2C address collisions.


I2C interface

The I2C interface uses the following pins:

? SCK: serial clock (SCL)

? SDI: data (SDA)

? SDO: The I2C address decides the pin. If SDO connects to GND(0), the address is 0x76, if it connects to VDDIO(1), the address is 0x77. In this module, we have connected it to VDDIO, so the address should be 0x77.

? CSB: Must be connected to VDDIO to select I2C interface.

SPI interface

The SPI interface uses the following pins:

? CSB: chip select, active low and has an integrated pull-up resistor

? SCK: serial clock

? SDI: serial data input; data input/output in 3-wire mode

? SDO: serial data output; hi-Z in 3-wire mode

The sketch starts by defining the connections to the Joystick module. The SW pin is connected to Arduino’s Pin A2 while the VRx and VRy pins are connected to Analog pins A0 and A1.

#define Xaxis_pin A0 // Arduino pin connected to the VRx Pin
#define Yaxis_pin A1 // Arduino pin connected to the VRy Pin
#define SW_pin A2 // Arduino pin connected to the SW Pin
In the setup() function, we initialized the SW pin as an input and keep it HIGH. This is as same as declaring the pin as INPUT_PULLUP. When the switch is pressed, this pin will be pulled to the ground. Thus, we can detect the button press by monitoring the state of this pin. We have also initialized the serial communication so that we can print all the necessary information into the serial monitor.

void setup() {
  pinMode(SW_pin, INPUT);
  digitalWrite(SW_pin, HIGH);
  Serial.begin(9600);
}
In loop() function, we continously read the value of SW pin using digitalRead() function, VRx & VRy pin using analogRead() and display on serial monitor.

void loop() {
  Serial.print("X-axis: ");
  Serial.print(analogRead(Xaxis_pin));
  Serial.print(" : ");
  Serial.print("Y-axis: ");
  Serial.print(analogRead(Yaxis_pin));
  Serial.print(" : ");
  Serial.print("Switch:  ");
  Serial.println(digitalRead(SW_pin));
  delay(200);
}

Otto servo position with arms sketch for Arduino IDE. NOTE: if you have installed Arduino IDE v1.8.5 or later, select ATmega328P oldbootloader. For all clone Nano microboards and Genuine Arduino boards brought before Jan 2022.

Arduino Code
This example sends the state of the pushbutton and two analog outputs serially to the computer. The associated Processing sketch reads the serial data to animate the joystick position.

The sketch is identical to the one above, except that the values printed on the serial monitor are separated by commas. The reason for separating values with commas is to make data transfer easier. The concept here is to send the values as a comma-separated string that we can parse in the Processing IDE to retrieve the values.

Upload the sketch below to your Arduino.

int xValue = 0 ; // read value of the X axis	
int yValue = 0 ; // read value of the Y axis	
int bValue = 0 ; // value of the button reading	

void setup()	
{	
	Serial.begin(9600) ; // Open the serial port
	pinMode(8,INPUT) ; // Configure Pin 2 as input
	digitalWrite(8,HIGH);	
}	

void loop()	
{	
	// Read analog port values A0 and A1	
	xValue = analogRead(A0);	
	yValue = analogRead(A1);	

	// Read the logic value on pin 2	
	bValue = digitalRead(8);	

	// We display our data separated by a comma	
	Serial.print(xValue,DEC);
	Serial.print(",");
	Serial.print(yValue,DEC);
	Serial.print(",");
  
![Screenshot (1076)](https://github.com/Arush-om/Diy-Otto-Robot/assets/134673721/63964b0e-3f94-49dd-9fb3-e023eae105a1)

#include <Servo.h>

#include <Oscillator.h>

#include <EEPROM.h>

#define N_SERVOS 4

#define EEPROM_TRIM false //Activate for calibration with serial

#define TRIM_RR 18#define TRIM_RL 18#define TRIM_YR 26#define TRIM_YL 18

#define PIN_RL 2#define PIN_RR 3#define PIN_YR 4#define PIN_YL 5

#define INTERVALTIME 10.0

Oscillator servo[N_SERVOS];

Then follow the diagram pins numbers and make sure to put them in the right position.

You have at least 4 option to power your Otto:

1. 4xAA alkaline batteries (1.5V each) that connected in series go to Vin pin and Gnd

2. 4xAA rechargeable batteries (1.2V each) that connected in series go to 5V pin and Gnd

3. Just directly from USB cable to your computer or even a power bank

4. External jack connector to use power adapters from 6V up to 12V output

![Screenshot (1079)](https://github.com/Arush-om/Diy-Otto-Robot/assets/134673721/755a4c1a-5766-4846-9dfe-f426c45e53fd)

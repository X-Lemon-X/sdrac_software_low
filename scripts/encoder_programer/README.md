## Introduction
This is a simple arduino program to burn new i2c address to a M6701 magnetic encoder.

## Requirements
1. Arduino IDE
2. M6701 Magnetic Encoder
3. Arduino Microcontroller
4. Vcc=5V conencted to the encoder 
5. I2C connection to the encoder from the microcontroller and some pull up resistors.
6. Follopw datasheet from more detailed instructions [M6701 Datasheet](../../datasheets/MT6701CT-STD.pdf).


## How to Use
1. Load the i2c-encoder-programer.ino file into your Arduino IDE.
2. Verify and compile the code to ensure there are no errors.
3. Upload the compiled code to your microcontroller wait 10 second so the main prgramm stops (you dont want to conenct it half way through).
5. Open the serial monitor in the Arduino IDE.
4. Connect your I2C device to the appropriate pins on your microcontroller (I2C pins and VCC and 5V).
6. Restart the microcontroller.
7. Follow the instructions in the serial monitor to burn the new i2c address to the device.
This will mostly require you to wait for a few second then dosconnect i2c device from VCC wait for 30 seconds and connect it again, then restart the microcontroller to check if the new address is burned.


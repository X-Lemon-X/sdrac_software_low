
#include <Wire.h>

#define i2c_address  0x06
#define i2c_register 0x28
#define i2c_new_address 0x46

#define i2c_programing_key_reg 0x09
#define i2c_programing_key 0xB3

#define i2c_programing_command_reg 0x0A
#define i2c_programing_command 0x05

void serial_scaner(){

 int nDevices;
  byte error, address;
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n"); 

}

void setup() {
  Wire.begin(); // Initialize I2C communication
  Serial.begin(9600); // Initialize serial communication for debugging
  
  delay(5000); // Wait 5 for the serial monitor to open
  Serial.println("I2C programer");
  ///############################################
  /// program the i2s address
  
  Wire.beginTransmission(i2c_address);
  Wire.write(i2c_register);
  Wire.endTransmission();

  Wire.requestFrom(i2c_address, 1); // Request 1 byte from the device
  if (!Wire.available()) {
    Serial.println("Failed to read data from base i2c address");
    Serial.println("Scanning for i2c devices");
    Serial.println("If you see the i2c address 0x46, the i2c address is already changed");
    serial_scaner();
    return;
  }
  Serial.print("I2c encdoer connected address: 0x");
  Serial.println(i2c_address, HEX);


  byte address_reg_data = Wire.read();
  Serial.print("Data read from address 0x");
  Serial.print(i2c_address, HEX);
  Serial.print(", register 0x");
  Serial.print(i2c_register, HEX);
  Serial.print(": 0b");
  Serial.print(address_reg_data, BIN);
  Serial.print(": 0x");
  Serial.println(address_reg_data, HEX);

  // set bit 4 to 1 to change the i2c address
  address_reg_data = address_reg_data | 0x08;
  Serial.print("Data to write to address 0x");
  Serial.print(i2c_address, HEX);
  Serial.print(", register 0x");
  Serial.print(i2c_register, HEX);
  Serial.print(": 0b");
  Serial.print(address_reg_data, BIN);
  Serial.print(": 0x");
  Serial.println(address_reg_data, HEX);

  //############################################
  // write the new i2c address

  byte data[2];

  Wire.beginTransmission(i2c_address);
  data[0] = 0x28;
  data[1] = 0x0C;
  Wire.write(data, 2);
  Wire.endTransmission();


  Wire.beginTransmission(i2c_new_address);
  Wire.write(i2c_register);
  Wire.endTransmission();
  Wire.requestFrom(i2c_new_address, 1); // Request 1 byte from the device
  if (!Wire.available()) {
    Serial.println("Failed to change address");
    return;
  }
  Serial.println("I2C address changed");

  Serial.println("I2c encdoer connected address: 0x46");
  Serial.println("Burning settings in EEPROM");
  Wire.beginTransmission(i2c_new_address);
  data[0] = i2c_programing_key_reg;
  data[1] = i2c_programing_key;
  Wire.write(data, 2);
  Wire.endTransmission();

  Wire.beginTransmission(i2c_new_address);
  data[0] = i2c_programing_command_reg;
  data[1] = i2c_programing_command;
  Wire.write(data, 2);
  Wire.endTransmission();
  Serial.println("Wait for 1 second");
  delay(1000);
  Serial.println("EEPROM burned");
  Serial.println("Disconnect the MT6701 from VCC.");
  Serial.println("Connect the MT6701 to VCC. and restart the arduino");

}



void loop() {

}

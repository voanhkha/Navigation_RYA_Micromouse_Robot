//VO ANH KHA NAVIGATION ROBOT 
// MODULE GYROSCOPE L3G4200D
// SCL->PA_6; SDA->PA_7
#include <Wire.h>

//CONFIGURATION FOR GYROSCOPE
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
int L3G4200D_Address = 105; //I2C address of the L3G4200D
int scale_gyro = 250;
int z;
long start_time;
long elapsed;
float z_float; 
float total_angle = 0;
float last_angle = 0;
float gyro_offset = 0;

void setup() // run once before void loop
{
  Serial.begin(9600);
  //-----------------------------------------------
  //Configure Gyro 
  Wire.begin();
  setupL3G4200D(scale_gyro);
  delay(1000); // wait 1000 ms = 1 s
  CalibGyro();
}

//--------------------------------------------------------------
// MAIN LOOP
void loop() //main code here, run repeatedly
{
  getGyroValues();  // This will update x, y, and z with new values
  z_float = z*0.00875 - gyro_offset;
  elapsed = millis() - start_time;
  start_time = millis();
  total_angle = total_angle + z_float*elapsed/1000;
  Serial.println(total_angle);
  delay(10); 
}
// END OF LOOP
//--------------------------------------------------------------
// -----------------------------Gyro voids----------------------
void CalibGyro(){
  int i = 0;
  float z_total = 0;
  for (i = 0; i < 200; i++)  {
    getGyroValues();
    z_total = z_total + z*0.00875;
    delay(10);
  }
  gyro_offset = z_total / 200;
}

void getGyroValues(){
  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  z = ((zMSB << 8) | zLSB);
  if (z>32767) {
    z = z - 65536;  
  }
}

int setupL3G4200D(int scale){
  //From  Jim Lindblom of Sparkfun's code

    // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:
  if(scale == 250){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }
  else if(scale == 500){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }
  else{
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }
  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}

void writeRegister(int deviceAddress, byte address, byte val) {
  Wire.beginTransmission(deviceAddress); // start transmission to device 
  Wire.write(address);       // send register address
  Wire.write(val);         // send value to write
  Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){
  int v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); // register to read
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, 1); // read a byte
  while(!Wire.available()) {
    // waiting
  }
  v = Wire.read();
  return v;
}



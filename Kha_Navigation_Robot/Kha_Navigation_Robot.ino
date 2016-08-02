//VO ANH KHA NAVIGATION ROBOT 
#include <Wire.h>

// i2c port
TwoWire i2c1(1);

// CONFIGURATION FOR COMPASS
// address
#define hmc5883_addr_read  0x3d
#define hmc5883_addr_write 0x3c
#define hmc5883_addr_rw    0x1e
// regs
#define hmc5883_reg_config_a 0
#define hmc5883_reg_config_b 1
#define hmc5883_reg_mode     2
#define hmc5883_reg_x_msb    3
#define hmc5883_reg_x_lsb    4
#define hmc5883_reg_z_msb    5
#define hmc5883_reg_z_lsb    6
#define hmc5883_reg_y_msb    7
#define hmc5883_reg_y_msb    8
#define hmc5883_reg_status   9
#define hmc5883_reg_id_a     10
#define hmc5883_reg_id_b     11
#define hmc5883_reg_id_c     12
// reg configure A
#define hmc5883_avg_1 1<<5
#define hmc5883_avg_2 2<<5
#define hmc5883_avg_4 3<<5
#define hmc5883_avg_8 4<<5
#define hmc5883_hz_0_75 0<<2
#define hmc5883_hz_1_5  1<<2
#define hmc5883_hz_3    2<<2
#define hmc5883_hz_7_5  3<<2
#define hmc5883_hz_15   4<<2
#define hmc5883_hz_30   5<<2
#define hmc5883_hz_75   6<<2
#define hmc5883_ms_normal   0
#define hmc5883_ms_positive 1
#define hmc5883_ms_negative 2
// reg configure B
#define hmc5883_gain_0_88 0<<4
#define hmc5883_gain_1_33 1<<4
#define hmc5883_gain_1_9  2<<4
#define hmc5883_gain_2_5  3<<4
#define hmc5883_gain_4    4<<4
#define hmc5883_gain_4_7  5<<4
#define hmc5883_gain_5_6  6<<4
#define hmc5883_gain_8_1  7<<4
// reg configure mode
#define hmc5883_md_continuous 0 
#define hmc5883_md_single     1
///////////////////////////////////////
//CONFIGURATION FOR GYROSCOPE
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
int L3G4200D_Address = 105; //I2C address of the L3G4200D
int scale_gyro = 250;
int x;
int y;
int z;
long start_time;
long elapsed;
float z_float; 
float total_angle = 0;
float last_angle = 0;
float gyro_offset = 0;

/////////////////////////////////////////
//Pin define
#define LEFT_MOTOR PB_5
#define RIGHT_MOTOR PB_7
#define LEFTEST_IR PE_1
#define RIGHTEST_IR PE_0
#define LEFT_IR PF_1
#define RIGHT_IR PF_2
#define CENTER_IR PA_2
#define BOOST_EN_PIN PB_2

const int strobe = 29;
const int clock = 28;
const int data = 5;

int LeftestIR, LeftIR, CenterIR, RightIR, RightestIR;
int state;
char buf[100];
float angle_new = 0, angle_old = 0, angle_target = 180;
int compass_err = 10;
volatile int LeftEncCount = 0;
volatile int RightEncCount = 0;
volatile int LeftEncFlag = 0;
volatile int RightEncFlag = 0;
volatile int LastLeftEncCount = 0;
volatile int LastRightEncCount = 0;
volatile int flag;
int PPR = 10000;
int LeftSpeed;
int RightSpeed;
int goStraightSpeed = 200;
int WheelSpeed =  130;
#define LeftBaseSpeed 78
#define RightBaseSpeed 82
#define RightMaxSpeed 120 // max speed of the robot
#define LeftMaxSpeed 120 // max speed of the robot
int posi_error, last_posi_error, tuning, integral;
int Kp = 2;
int Kd = 20;
int Ki = 0; 

volatile int next_target = 1;
volatile int current_target = 1;

float SpinMatrix[4][4] = {
  {
    0,  45, 135, 0  }
  ,
  {
    45,  0,  -15, 0  }
  ,
  {
    15, 95,  0, 0  }
  ,
  {
    0,  0,  0, 0  }
};

int SpinDegree = 0;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
boolean stringCorrect = false;

void setup() // run once before void loop
{
  //usart debug
  Serial.begin(9600);

  //configure hmc5883l for COMPASS
  i2c1.begin(); // for Compass

  i2c1.beginTransmission(hmc5883_addr_rw); 
  i2c1.write( hmc5883_reg_config_a ); 
  i2c1.write( hmc5883_avg_8 | hmc5883_hz_15 | hmc5883_ms_normal); 
  i2c1.endTransmission(  );

  i2c1.beginTransmission(hmc5883_addr_rw); 
  i2c1.write( hmc5883_reg_config_b ); 
  i2c1.write( hmc5883_gain_1_33 ); 
  i2c1.endTransmission();

  i2c1.beginTransmission(hmc5883_addr_rw); 
  i2c1.write( hmc5883_reg_mode );  
  i2c1.write( hmc5883_md_continuous); 
  i2c1.endTransmission();
  //-----------------------------------------------
  //Configure Gyro 
  Wire.begin();
  setupL3G4200D(scale_gyro);
  //-----------------------------------------------
  // pin config
  delay(1000); // wait 1000 ms = 1 s
  pinMode(BOOST_EN_PIN, OUTPUT); 
  pinMode(LEFT_MOTOR, OUTPUT); 
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(LEFTEST_IR, INPUT); 
  //pinMode(LEFT_IR, INPUT); 
  pinMode(RIGHTEST_IR, INPUT); 
  // pinMode(RIGHT_IR, INPUT); 
  pinMode(CENTER_IR, INPUT); 
  //pinMode(BLUE_LED, OUTPUT); 

  digitalWrite(BOOST_EN_PIN, HIGH);

  //Interrupt for IR
  pinMode(LEFT_IR, INPUT_PULLUP);
  pinMode(RIGHT_IR, INPUT_PULLUP);
  //attachInterrupt(LEFTEST_IR, LeftIR_ISR, RISING); // Interrupt is fired whenever button is pressed
  //attachInterrupt(RIGHTEST_IR, RightIR_ISR, RISING); 
  pinMode(PF_0, INPUT_PULLUP);
  pinMode(PF_4, INPUT_PULLUP);
  //------------------------------------------
  // Interrupt pins for encoders
  pinMode(PC_5, INPUT_PULLUP);
  pinMode(PD_6, INPUT_PULLUP);
  attachInterrupt(PC_5, RightEncISR, RISING);
  attachInterrupt(PD_6, LeftEncISR, RISING); 
  //--------------------------------------------
  // Config 7-segment LEDs
  pinMode(strobe, OUTPUT);
  pinMode(clock, OUTPUT);
  pinMode(data, OUTPUT);
  //digitalWrite(strobe, HIGH);
  sendCommand(0x8f);  // activate
  sendCommand(0x40); // set auto increment mode
  reset();
  Display(current_target, 6);
  Display(next_target, 8);
  Display(10,7);

  //-----------------------------------------------
  inputString.reserve(200);
  CalibGyro();
  GoStraight_PWM();
}

//--------------------------------------------------------------
// MAIN LOOP
void loop() //main code here, run repeatedly
{
  getIR();
  if(CenterIR==1 && LeftIR==1 && RightIR==1 && LeftestIR==1 && RightestIR==1){
    posi_error = 0;
    last_posi_error = 0;
    Stop();
    WaitForNextTarget();
    SpinToNextTarget();
    delay(3000);
    MoveNoLine();
    posi_error = 0;
    last_posi_error = 0;
  }

  getIR();
  integral += posi_error;
  tuning = Kp*posi_error + Kd*(posi_error-last_posi_error)+Ki*integral;
  last_posi_error = posi_error;
  int RightMotorSpeed = RightBaseSpeed + tuning;
  int LeftMotorSpeed = LeftBaseSpeed - tuning;

  if (RightMotorSpeed > RightMaxSpeed ) RightMotorSpeed = RightMaxSpeed; // prevent the motor from going beyond max speed
  if (LeftMotorSpeed > LeftMaxSpeed ) LeftMotorSpeed = LeftMaxSpeed; // prevent the motor from going beyond max speed
  if (RightMotorSpeed < 0) RightMotorSpeed = 0; // keep the motor speed positive
  if (LeftMotorSpeed < 0) LeftMotorSpeed = 0; // keep the motor speed positive
  analogWrite(LEFT_MOTOR,LeftMotorSpeed);
  analogWrite(RIGHT_MOTOR,RightMotorSpeed);

  //  angle_new=get_angle;
  //  Serial.print(" angle=");
  //  Serial.print((int) angle_new);
  //  Serial.println("");
  LeftSpeed = getLeftEncIncre();
  RightSpeed = getRightEncIncre();
  Serial.print("LeftSpeed=");
  Serial.print((int) LeftSpeed);
  Serial.println("");
  Serial.print("RightSpeed=");
  Serial.print((int) RightSpeed);
  Serial.println("");

}
// END OF LOOP
//--------------------------------------------------------------
// -----------------------------Gyro voids----------------------
void CalibGyro(){
  int i = 0;
  float z_total = 0;
  for (i = 0; i < 50; i++)  {
    getGyroValues();
    z_total = z_total + z*0.00875;
    delay(10);
  }
  gyro_offset = z_total / 50;
}

void getGyroValues(){
  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  z = ((zMSB << 8) | zLSB);
  if (z>32767) {z = z - 65536; }
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
//--------------------------------------------------------------
// --------------------Wheel controling voids----------------
void MoveNoLine(){
  long start_time_1;
  long elapsed_1;
Stop();
delay(500);
CalibGyro();
  //CalibGyro();
  int flag_detect_line = 0;
  total_angle = 0;
  start_time = millis();
  start_time_1 = millis();
  //  analogWrite(LEFT_MOTOR, LeftBaseSpeed);
  //  analogWrite(RIGHT_MOTOR, RightBaseSpeed);
  while(flag_detect_line == 0){
    delay(10);
    getIR();
    elapsed_1 = millis() - start_time_1;
    if (elapsed_1 > 3000){
    if (LeftestIR==0){
      Stop();
      delay(100);
      while(RightestIR==1){
        getIR();
        SpinStepLeft();
        SpinStepRight();
        delay(20);
      }
      SpinLeftToLine();
      current_target = next_target;
      Display(current_target, 6);
      break;
    }
    if (RightestIR==0){
       Stop();
       delay(100);
      while(LeftestIR==1){
        getIR();
        SpinStepLeft();
        SpinStepRight();
        delay(20);
      }
      SpinRightToLine();
      current_target = next_target;
      Display(current_target, 6);
      break;
    }
  }
    getGyroValues();  // This will update x, y, and z with new values
    z_float = z*0.00875 - gyro_offset;
    elapsed = millis() - start_time;
    start_time = millis();
    total_angle = total_angle + z_float*elapsed/1000;
    posi_error = -total_angle;
    tuning = Kp*posi_error + Kd*(posi_error-last_posi_error)+Ki*integral;
    last_posi_error = posi_error;
    int RightMotorSpeed = RightBaseSpeed + tuning;
    int LeftMotorSpeed = LeftBaseSpeed - tuning;
    if (RightMotorSpeed > RightMaxSpeed ) RightMotorSpeed = RightMaxSpeed; // prevent the motor from going beyond max speed
    if (LeftMotorSpeed > LeftMaxSpeed ) LeftMotorSpeed = LeftMaxSpeed; // prevent the motor from going beyond max speed
    if (RightMotorSpeed < 0) RightMotorSpeed = 0; // keep the motor speed positive
    if (LeftMotorSpeed < 0) LeftMotorSpeed = 0; // keep the motor speed positive
    analogWrite(LEFT_MOTOR,LeftMotorSpeed);
    analogWrite(RIGHT_MOTOR,RightMotorSpeed);
    Display_Angle((int) total_angle);
  }
}


void SpinLeftToLine(){
  Stop();
  int spin_flag = 0;
  int time = millis();
  while(spin_flag == 0){
    SpinStepLeft();
    delay(20);
    getIR(); 
    if(LeftestIR==1 && LeftIR==0 && CenterIR==0 && RightIR==0 && RightestIR==1){
      spin_flag=1;
    }
    if((millis()-time)>3000){
      spin_flag=1;
    }
  }
}

void SpinRightToLine(){
  Stop();
  int spin_flag = 0;
  int time = millis();
  while(spin_flag == 0){
    SpinStepRight();
    delay(20);
    getIR();
    if(LeftestIR==1 && LeftIR==0 && CenterIR==0 && RightIR==0 && RightestIR==1){
      spin_flag=1;
    }
    if((millis()-time)>3000){
      spin_flag=1;
    }
  }
}

void Spin(float angle){
  float z_gyro ;
  int Speed;
  long start_time = millis();
  long elapsed = 0;
  float spin_angle = 0;
  int i;
  Stop();
  delay(500);
  CalibGyro();

  if (angle>0){ //quay banh phai
    //analogWrite(RIGHT_MOTOR, RightWheelSpeed);
    while(abs(spin_angle) < abs(angle)){    
      SpinStepRight();
      delay(15);
      getGyroValues();  // This will update x, y, and z with new values
      z_gyro = z*0.00875 - gyro_offset;
      elapsed = millis() - start_time;
      start_time = millis();
      spin_angle = spin_angle + z_gyro*elapsed/1000;
      Display_Angle((int)spin_angle);
    }

    analogWrite(RIGHT_MOTOR, 0);
    for (i = 0; i < 20; i++)  {
      delay(20);
      getGyroValues();
      z_gyro = z*0.00875 - gyro_offset;
      elapsed = millis() - start_time;
      start_time = millis();
      spin_angle = spin_angle + z_gyro*elapsed/1000;
      Display_Angle((int)spin_angle);
    }

    while(spin_angle > angle){
      SpinStepLeft();
      getGyroValues();
      z_gyro = z*0.00875 - gyro_offset;
      elapsed = millis() - start_time;
      start_time = millis();
      spin_angle = spin_angle + z_gyro*elapsed/1000;
      Display_Angle((int)spin_angle);
      delay(15);
    }

  }

  else{ // quay banh trai
    //Speed = LeftBaseSpeed+20;
    //analogWrite(LEFT_MOTOR, Speed);
    while(abs(spin_angle) < abs(angle)){
      SpinStepLeft();
      delay(15);
      getGyroValues();
      z_gyro = z*0.00875 - gyro_offset;
      elapsed = millis() - start_time;
      start_time = millis();
      spin_angle = spin_angle + z_gyro*elapsed/1000;
      Display_Angle((int)spin_angle);
    }
    analogWrite(LEFT_MOTOR, 0);
    for (i = 0; i < 20; i++)  {
      delay(15);
      getGyroValues();
      z_gyro = z*0.00875 - gyro_offset;
      elapsed = millis() - start_time;
      start_time = millis();
      spin_angle = spin_angle + z_gyro*elapsed/1000;
      Display_Angle((int)spin_angle);
    }

    while(spin_angle < angle){
      SpinStepRight();
      getGyroValues();
      z_gyro = z*0.00875 - gyro_offset;
      elapsed = millis() - start_time;
      start_time = millis();
      spin_angle = spin_angle + z_gyro*elapsed/1000;
      Display_Angle((int)spin_angle);
      delay(15);
    }
  }
}

void GoStraight_PWM()
{ 
  analogWrite(LEFT_MOTOR, LeftBaseSpeed);
  analogWrite(RIGHT_MOTOR, RightBaseSpeed);
}

void Stop()
{
  analogWrite(RIGHT_MOTOR, 0);
  analogWrite(LEFT_MOTOR, 0);
}

//--------------------------------------------------------------
//--------------------------IR Voids---------------------
void RightestIR_ISR(){
  detachInterrupt(LEFTEST_IR); // Interrupt is fired whenever button is pressed
  detachInterrupt(RIGHTEST_IR); 
  while(LeftestIR==1){
    getIR();
  }
  analogWrite(LEFT_MOTOR, 0);
  analogWrite(RIGHT_MOTOR, RightBaseSpeed);
  while(RightestIR==0){
    getIR();
  }
  flag = 1;
}

void LeftestIR_ISR()
{ 
  detachInterrupt(LEFTEST_IR); // Interrupt is fired whenever button is pressed
  detachInterrupt(RIGHTEST_IR); 
  while(RightestIR==1){
    getIR();
  }
  analogWrite(RIGHT_MOTOR, 0);
  analogWrite(LEFT_MOTOR, LeftBaseSpeed);
  while(LeftestIR==0){
    getIR();
  }
  flag = 1;
}


void getIR()
{
  LeftestIR=digitalRead(LEFTEST_IR);
  LeftIR=digitalRead(LEFT_IR);
  CenterIR=digitalRead(CENTER_IR);
  RightIR=digitalRead(RIGHT_IR);
  RightestIR=digitalRead(RIGHTEST_IR);
  posi_error = LeftestIR*(-20)+LeftIR*(-8)+RightIR*8+RightestIR*20;
  //posi_error = LeftIR*(-10)+RightIR*10;
}

float get_angle()
{
  i2c1.beginTransmission(hmc5883_addr_rw);
  i2c1.write(hmc5883_reg_x_msb); 
  i2c1.endTransmission(false);

  int rxmax=6;  
  int rxc = 0;
  int rxdata[6];

  i2c1.requestFrom(hmc5883_addr_rw, rxmax,true);
  while(Wire.available() && rxc<rxmax)
  {
    rxdata[rxc]=i2c1.read();
    rxc++;
  }
  int16_t sx,sy,sz; //triple axis data
  sx = rxdata[1] | rxdata[0] <<8; 
  //sz = rxdata[3] | rxdata[2] <<8;  
  sy = rxdata[5] | rxdata[4] <<8;
  float angle= atan2((float)sy,(float)sx) * (180 / 3.14159265) + 180; 
  return angle;
}

void RightEncISR()
{
  RightEncCount++;
  if(RightEncCount == -32768) RightEncFlag = 1;//integer overflow from 32767 to -32768
}

void LeftEncISR()
{
  LeftEncCount++;
  if(LeftEncCount == -32768) LeftEncFlag = 1;
}

int getLeftEncIncre()
{ 
  int incre;
  if (LeftEncFlag==1)
  {
    incre = 65535 - LeftEncCount + LastLeftEncCount; 
    LeftEncFlag=0;
  }
  else
  {
    incre=LeftEncCount - LastLeftEncCount;
  }
  LastLeftEncCount = LeftEncCount;
  return incre;
}

int getRightEncIncre()
{
  int incre;
  if (RightEncFlag==1)
  {
    incre = 65535 - RightEncCount + LastRightEncCount; 
    RightEncFlag=0;
  }
  else
  {
    incre=RightEncCount - LastRightEncCount;
  }
  LastRightEncCount = RightEncCount;
  return incre;
}

//--------------------------------------------------
//--------- 7seg LED-------------------------------

void sendCommand(uint8_t val)
{
  digitalWrite(strobe, LOW);
  delayMicroseconds(1);
  uint8_t i;
  for (i = 0; i < 8; i++)  {
    digitalWrite(data, !!(val & (1 << i))); 
    digitalWrite(clock, LOW);
    delayMicroseconds(1);
    digitalWrite(clock, HIGH);
    delayMicroseconds(1);
  }
  //KhashiftOut(data, clock, LSBFIRST, value);
  digitalWrite(strobe, HIGH);
  delayMicroseconds(1);
}

void reset()
{
  uint8_t value = 0xc0;
  digitalWrite(strobe, LOW);
  delayMicroseconds(1);
  uint8_t i;
  for (i = 0; i < 8; i++)  {
    digitalWrite(data, !!(value & (1 << i))); 
    digitalWrite(clock, LOW);
    delayMicroseconds(1);
    digitalWrite(clock, HIGH);
    delayMicroseconds(1);
  }
  for(uint8_t i = 0; i < 16; i++){
    for(uint8_t j = 0; j<8; j++)
    {
      digitalWrite(data, LOW); 
      digitalWrite(clock, LOW);
      delayMicroseconds(1);
      digitalWrite(clock, HIGH);
      delayMicroseconds(1);
    }
  }
  //KhashiftOut(data, clock, LSBFIRST, value);
  digitalWrite(strobe, HIGH);
  delayMicroseconds(1);
}

uint8_t Convert7seg(int value){
  switch (value) {
  case 0:
    return 0x3F;
    break;
  case 1:
    return 0x06;
    break;
  case 2:
    return 0x5B;
    break;
  case 3:
    return 0x4F;
    break;
  case 4:
    return 0x66;
    break;
  case 5:
    return 0x6D;
    break;
  case 6:
    return 0x7D;
    break;
  case 7:
    return 0x07;
    break;
  case 8:
    return 0x7F;
    break;
  case 9:
    return 0x67;
    break;
  default: 
    return 0x00;
    // if nothing else matches, do the default
    // default is optional
  }

}

void Display_Angle(int digits){
  if (digits < 0){
    digits = - digits;
    Display(10,1);
  }
  else Display(11,1);
  int hundred = digits/100;
  int tens = (digits/10)%10;
  int units = digits%10;
  Display(hundred,2);
  Display(tens,3);
  Display(units,4);
}

void Display_Angle_2(int digits){
  if (digits < 0){
    digits = - digits;
    Display(10,5);
  }
  else Display(11,5);
  int hundred = digits/100;
  int tens = (digits/10)%10;
  int units = digits%10;
  Display(hundred,6);
  Display(tens,7);
  Display(units,8);
}

void Display(int number, int position){
  int position_code;
  int number_code;
  switch (position) {
  case 1:
    position_code = 0xc0;
    break;
  case 2:
    position_code = 0xc2;
    break;
  case 3:
    position_code = 0xc4;
    break;
  case 4:
    position_code = 0xc6;
    break;
  case 5:
    position_code = 0xc8;
    break;
  case 6:
    position_code = 0xca;
    break;
  case 7:
    position_code = 0xcc;
    break;
  case 8:
    position_code = 0xce;
    break;
  default: 
    position_code = 0xce;
  }

  switch (number) {
  case 0:
    number_code = 0x3F;
    break;
  case 1:
    number_code = 0x06;
    break;
  case 2:
    number_code = 0x5B;
    break;
  case 3:
    number_code = 0x4F;
    break;
  case 4:
    number_code = 0x66;
    break;
  case 5:
    number_code = 0x6D;
    break;
  case 6:
    number_code = 0x7D;
    break;
  case 7:
    number_code = 0x07;
    break;
  case 8:
    number_code = 0x7F;
    break;
  case 9:
    number_code = 0x67;
    break;
  case 10: // ki hieu "-"
    number_code = 0x40;
    break;
  case 11: // khong ki hieu
    number_code = 0x00;
    break; 
  default: 
    number_code = 0xFF;
    // if nothing else matches, do the default
    // default is optional
  }

  digitalWrite(strobe, LOW);
  delayMicroseconds(1);
  uint8_t i;
  for (i = 0; i < 8; i++)  {
    digitalWrite(data, !!(position_code & (1 << i))); 
    digitalWrite(clock, LOW);
    delayMicroseconds(1);
    digitalWrite(clock, HIGH);
    delayMicroseconds(1);
  }
  for (i = 0; i < 8; i++)  {
    digitalWrite(data, !!(number_code & (1 << i))); 
    digitalWrite(clock, LOW);
    delayMicroseconds(1);
    digitalWrite(clock, HIGH);
    delayMicroseconds(1);
  }
  //KhashiftOut(data, clock, LSBFIRST, value);
  digitalWrite(strobe, HIGH);
  delayMicroseconds(1);
}

//--------------------------------------------------------
// General control voids
void WaitForSpinDegree(){
  int wait_flag = 0 ;
  const int buttonPin = PF_0; //push button attached to this pin
  const int buttonPin_2 = PF_4; //push button attached to this pin
  int buttonState = LOW; //this variable tracks the state of the button, low if not pressed, high if pressed
  int buttonState_2 = LOW;
  long DebounceTime = 0;  // the last time the output pin was toggled
  long debounceDelay = 50;    // the debounce time; increase if the output flickers

  while(wait_flag == 0){

    // Check button 1
    buttonState = digitalRead(buttonPin);
    if (buttonState == LOW){
      delay(50);
      buttonState = digitalRead(buttonPin);
      if (buttonState == LOW){
        while(buttonState == LOW){
          buttonState = digitalRead(buttonPin);
        }
        SpinDegree = SpinDegree + 5;
        if (SpinDegree > 180) SpinDegree = -180;
        Display_Angle_2(SpinDegree);
      }
    }

    // Check button 2
    buttonState_2 = digitalRead(buttonPin_2);
    if (buttonState_2 == LOW){
      delay(50);
      buttonState_2 = digitalRead(buttonPin_2);
      if (buttonState_2 == LOW){
        while(buttonState_2 == LOW){
          buttonState_2 = digitalRead(buttonPin_2);
        }
        wait_flag = 1;
      }
    }

  }
}

void WaitForNextTarget(){
  current_target = next_target;
  Display(current_target, 6);
  int wait_flag = 0 ;
  while(wait_flag == 0){
    
      while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();

    // add it to the inputString:
    inputString += inChar;

    //count=count+1;
    //if (inputString.startsWith("BCAST:000D6F000C21591D,05=Hello", 0)){stringCorrect = true;}

    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
    
    if (stringComplete==true) {
      //Serial.print(inputString.substring(26,28));
      if(inputString.substring(26,27)=="1"){
        next_target = 1;
        wait_flag = 1;
      }
      if(inputString.substring(26,27)=="2"){
         next_target = 2;
         wait_flag = 1;
      }
      if(inputString.substring(26,27)=="3"){
         next_target = 3;
         wait_flag = 1;
      }
      if(inputString.substring(26,27)=="4"){
         next_target = 4;
         wait_flag = 1;
      }
      
      inputString = "";
      stringComplete = false;  
      Display(next_target, 8);
    }
  }
}

void SpinToNextTarget(){
  float angle_to_rotate = SpinMatrix[current_target-1][next_target-1];
  Spin(angle_to_rotate);
}

void SpinStepRight(){
  analogWrite(LEFT_MOTOR, 0);
  analogWrite(RIGHT_MOTOR, 250);
  delay(5);
  analogWrite(RIGHT_MOTOR, 0);
}

void SpinStepLeft(){
  analogWrite(RIGHT_MOTOR, 0);
  analogWrite(LEFT_MOTOR, 220);
  delay(5);
  analogWrite(LEFT_MOTOR, 0);
}


void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();

    // add it to the inputString:
    inputString += inChar;

    //count=count+1;
    //if (inputString.startsWith("BCAST:000D6F000C21591D,05=Hello", 0)){stringCorrect = true;}

    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}



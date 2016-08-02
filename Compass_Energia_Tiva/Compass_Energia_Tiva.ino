//VO ANH KHA NAVIGATION ROBOT 
// MODULE COMPASS HMC5883
// SCL -> PA_6 ; SDA -> PA_7
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

void setup() // run once before void loop
{
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
}
//--------------------------------------------------------------
// MAIN LOOP
void loop() //main code here, run repeatedly
{
float compassangle = get_angle();
  Serial.println(compassangle);
delay(20);
}
// END OF LOOP

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




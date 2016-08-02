//VO ANH KHA NAVIGATION ROBOT 
// LED&KEY MODULE TM1638

const int strobe = 29; //PE_3
const int clock = 28; //PE_2
const int data = 5;//PE_4

void setup() // run once before void loop
{
  Serial.begin(9600);
  // Config 7-segment LEDs
  pinMode(strobe, OUTPUT);
  pinMode(clock, OUTPUT);
  pinMode(data, OUTPUT);
  sendCommand(0x8f);  // activate
  sendCommand(0x40); // set auto increment mode
  reset_LEDKEY();
  Display(1, 1);
  Display(2, 2);
  Display(3, 3);
  Display(4, 4);
  Display_Angle_2(-678);
}

//--------------------------------------------------------------
// MAIN LOOP
void loop() //main code here, run repeatedly
{

}
// END OF LOOP

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

void reset_LEDKEY()
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


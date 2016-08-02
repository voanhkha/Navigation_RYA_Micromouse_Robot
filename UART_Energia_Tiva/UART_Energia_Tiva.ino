//VO ANH KHA NAVIGATION ROBOT 
// MODULE UART
// UART1 (TXD,RXD) of Tiva displays what it received

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup() // run once before void loop
{
  Serial.begin(9600);
 inputString.reserve(200);
}

// MAIN LOOP
void loop() //main code here, run repeatedly
{
  delay(50);
   WaitForNextTarget();
   if (stringComplete==true) {
    Serial.println(inputString);
    stringComplete=false;
  }
}
// END OF LOOP

//--------------------------------------------------------
void WaitForNextTarget(){
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
    wait_flag = 1;
  }
}
}


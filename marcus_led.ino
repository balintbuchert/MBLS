#include <FastLED.h>

// How many leds in your strip?
#define NUM_LEDS 90

#define SENSOR_PIN      A0
#define CORRECTION_PIN  A7

// For led chips like Neopixels, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
#define DATA_PIN 5
//#define CLOCK_PIN 13
#define BRIGHTNESS  30
#define HELLO_LED 13

#define RUNNINGTIME        15000   // the game runnig time /milli secounds 

#define SYSTEM_CYCLE_TIME  250    // the main loop delay 

#define YELLOW_TIME        3000   
#define ORANGE_TIME        2000   
#define RED_TIME           1000   
#define BLACK_TIME         5000   

#define STOP_FADE_IN_OUT   100

#define DISABLED_DELAY     10000 // pause to start need constnat input to start.

#define SENSOR_MIN_LIMIT   2
 


// Define the array of leds
CRGB leds[NUM_LEDS];
int level = 0; // led level;
int max_hight = 0;
int peek_reset = 20; // sec to reset the max peek. 

int peek_reset_counter = 0;

bool valid_level = false;
int mi = 0;

CRGBPalette256 currentPalette; // the colour palet of the led strip.

void setup() { 
      Serial.begin(9600);
      Serial.println("start");
      // Uncomment/edit one of the following lines for your leds arrangement.
      
      FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
      pinMode(HELLO_LED, OUTPUT); // live hello wolrd led
      FastLED.setBrightness( BRIGHTNESS );
      
      palette();
       
}

//uint32_t color_section_1 = 0xFF0000;
//const char* color_section_1 = "CRGB::Red";
const palette()
{
    
    //fill_solid( currentPalette, 16, CRGB::Black);
    // and set every fourth one to white.
    // seciton 1 RED 0-14
    
    currentPalette[0] = CRGB::Red; 
    currentPalette[1] = CRGB::Red; 
    currentPalette[2] = CRGB::Red; 
    currentPalette[3] = CRGB::Red; 
    currentPalette[4] = CRGB::Red; 
    currentPalette[5] = CRGB::Red; 
    currentPalette[6] = CRGB::Red; 
    currentPalette[7] = CRGB::Red; 
    currentPalette[8] = CRGB::Red; 
    currentPalette[9] = CRGB::Red; 
    currentPalette[10] = CRGB::Red; 
    currentPalette[11] = CRGB::Red; 
    currentPalette[12] = CRGB::Red; 
    currentPalette[13] = CRGB::Red; 
    currentPalette[14] = CRGB::Red; 
     
    // seciton 2 Orange 15-29

   
       
    currentPalette[15] = CRGB::Orange;
    currentPalette[16] = CRGB::Orange;
    currentPalette[17] = CRGB::Orange;
    currentPalette[18] = CRGB::Orange;
    currentPalette[19] = CRGB::Orange;
    currentPalette[20] = CRGB::Orange;
    currentPalette[21] = CRGB::Orange;
    currentPalette[22] = CRGB::Orange;
    currentPalette[23] = CRGB::Orange;
    currentPalette[24] = CRGB::Orange;
    currentPalette[25] = CRGB::Orange;
    currentPalette[26] = CRGB::Orange;
    currentPalette[27] = CRGB::Orange;
    currentPalette[28] = CRGB::Orange;
    currentPalette[29] = CRGB::Orange;

    // seciton 3 YELLOW 30-44

      
    currentPalette[30] = CRGB::Yellow;
    currentPalette[31] = CRGB::Yellow;
    currentPalette[32] = CRGB::Yellow;
    currentPalette[33] = CRGB::Yellow;
    currentPalette[34] = CRGB::Yellow;
    currentPalette[35] = CRGB::Yellow;
    currentPalette[36] = CRGB::Yellow;
    currentPalette[37] = CRGB::Yellow;
    currentPalette[38] = CRGB::Yellow;
    currentPalette[39] = CRGB::Yellow;
    currentPalette[40] = CRGB::Yellow;
    currentPalette[41] = CRGB::Yellow;
    currentPalette[42] = CRGB::Yellow;
    currentPalette[43] = CRGB::Yellow;
    currentPalette[44] = CRGB::Yellow;

    // seciton 4 GREEN 45-59
  
    currentPalette[45] = CRGB::Green;
    currentPalette[46] = CRGB::Green;
    currentPalette[47] = CRGB::Green;
    currentPalette[48] = CRGB::Green;
    currentPalette[49] = CRGB::Green;
    currentPalette[50] = CRGB::Green;
    currentPalette[51] = CRGB::Green;
    currentPalette[52] = CRGB::Green;
    currentPalette[53] = CRGB::Green;
    currentPalette[54] = CRGB::Green;
    currentPalette[55] = CRGB::Green;
    currentPalette[56] = CRGB::Green;
    currentPalette[57] = CRGB::Green;
    currentPalette[58] = CRGB::Green;
    currentPalette[59] = CRGB::Green;
        
    // seciton 5 Cyan 60-74
  
    currentPalette[60] = CRGB::Cyan;
    currentPalette[61] = CRGB::Cyan;
    currentPalette[62] = CRGB::Cyan;
    currentPalette[63] = CRGB::Cyan;
    currentPalette[64] = CRGB::Cyan;
    currentPalette[65] = CRGB::Cyan;
    currentPalette[66] = CRGB::Cyan;
    currentPalette[67] = CRGB::Cyan;
    currentPalette[68] = CRGB::Cyan;
    currentPalette[69] = CRGB::Cyan;
    currentPalette[70] = CRGB::Cyan;
    currentPalette[71] = CRGB::Cyan;
    currentPalette[72] = CRGB::Cyan;
    currentPalette[73] = CRGB::Cyan;
    currentPalette[74] = CRGB::Cyan;
    
    // seciton 6 White 75-89
    
    currentPalette[75] = CRGB::White;
    currentPalette[76] = CRGB::White;
    currentPalette[77] = CRGB::White;
    currentPalette[78] = CRGB::White;
    currentPalette[79] = CRGB::White;
    currentPalette[80] = CRGB::White;
    currentPalette[81] = CRGB::White;
    currentPalette[82] = CRGB::White;
    currentPalette[83] = CRGB::White;
    currentPalette[84] = CRGB::White;
    currentPalette[85] = CRGB::White;
    currentPalette[86] = CRGB::White;
    currentPalette[87] = CRGB::White;
    currentPalette[88] = CRGB::White;
    currentPalette[89] = CRGB::White;
    
}


int increaseReset() {
    peek_reset_counter= peek_reset_counter+1 ;
}


void dim( int hight){
  int i = 0;
  int l = hight; // led hight
  int m = NUM_LEDS;//6;
  int mh = max_hight;

   palette();// recall color palette

  /*
  for (i = 0 ; i << l ; i++){
     leds[i-1] = currentPalette[i-1];
     //leds[i-1] %= 192;
  }
  //if (false)
  delay(100);
  for (i = l+1 ; i << m+1 ; i++){
     leds[i-1] = CRGB::Black;

 
  }
  */

   //if(peek_reset_counter >> peek_reset){
     
  //FastLED.show(); 

  //if (level >= m-15)        
    //flash();
  //else{   
    for (i = 0 ; i <= l ;i++){
      leds[i-1] = currentPalette[i-1];
     
      }
    
    for (i = l+1 ; i <= m ;i++){
      leds[i-1] = CRGB::Black; ; 
   
    }
 // }// end else
  if(level >= mh){
    max_hight = level;
    peek_reset_counter = 0;
   }
  
    if(peek_reset_counter > peek_reset){
        max_hight = level; 
        peek_reset_counter = 0;
    }
    
 
    Serial.print("peek_level: ");
    Serial.println(peek_reset_counter);
    Serial.println(peek_reset);
 
    leds[max_hight-1] = currentPalette[max_hight-1];

  
  //FastLED.show();
 
  }


/**  hello world led */
int ledState = LOW;  

void helloWolrd(){
   if(ledState == LOW)
     digitalWrite(HELLO_LED, ledState);
   else{
    ledState == HIGH;
    digitalWrite(HELLO_LED, ledState);
   } 
  }



/** flash led bar*/
bool flash_state = true;
int f = 0;

void flash(){
  f = 0;
  if(flash_state){
    while(f <= NUM_LEDS){     
      leds[f-1] = CRGB::White;
      f++;    
    }
    flash_state = false;
  } // end if
 else{
  f = 0;
    while(f <= NUM_LEDS){
      leds[f-1] = CRGB::Black;
      f++;
  }
  flash_state = true;
 }// end else 
  
}


// stop function;

bool stopEnabled = false;
int stopCounter = 0;
int s = 0;

void stopFunction(){
  
   Serial.println("STOP");
   
   //s = 0;  // from down 
   s = NUM_LEDS ;// from up 
   while(s >= 0){ 
   //while(s <= NUM_LEDS){     // down
      leds[s-1] = CRGB::Yellow;
      //s++; //down
      s--;   // up
      delay(STOP_FADE_IN_OUT);
      FastLED.show();   
    }
  
  delay(YELLOW_TIME);

  //s = 0; 
  s = NUM_LEDS; // from up 
  while(s >= 0){ 
  //while(s <= NUM_LEDS){         
      leds[s-1] = CRGB::Orange;
      s--;
      //s++; 
      delay(STOP_FADE_IN_OUT);
      FastLED.show(); 
    }
  
  delay(ORANGE_TIME);

  //s = 0;
  s = NUM_LEDS; // from up  
  //while(s <= NUM_LEDS){  
  while(s >= 0){    
      leds[s-1] = CRGB::Red;
      //s++;
      s--; 
      delay(STOP_FADE_IN_OUT);
      FastLED.show();   
    }

  delay(RED_TIME);
  
  //s = 0;
  s = NUM_LEDS; // from up   
  while(s >= 0){
  // while(s <= NUM_LEDS){         
      leds[s-1] = CRGB::Black;
      s--;
      //s++; 
      delay(STOP_FADE_IN_OUT);
      FastLED.show(); 
    }

  //s = 0; 
  s = NUM_LEDS;
  delay(BLACK_TIME);
}

/** */
void loop() { 
    
  // Turn the LED on, then pause

  Serial.println("live");
  //helloWolrd();
  delay(250);
  //increaseReset();

  peek_reset_counter+=1;
  int sensorCorectionValue = analogRead(CORRECTION_PIN);       // A0 
  int sensorValue = analogRead(SENSOR_PIN);           // A7 swop for test perpose 


  if(sensorCorectionValue == 0)
    level = (sensorValue/11.33);//*(sensorCorectionValue*0.1);
  else
    level = (sensorValue/11.33)*(sensorCorectionValue*0.01);


  if (stopEnabled){
     // reset 
     stopCounter = 0;
     stopFunction();
     stopEnabled = false;

    int sensor_tmp = 0;
    while ( sensor_tmp <= SENSOR_MIN_LIMIT){
      delay(1000);
      sensor_tmp = analogRead(SENSOR_PIN);
     
    }
    delay(DISABLED_DELAY);
     
  }
  else{
    dim(level);
    stopCounter++; // add one to the play time
    Serial.print("STOP:  ");
     Serial.println(stopCounter);
    if (stopCounter > (RUNNINGTIME/ SYSTEM_CYCLE_TIME))
      stopEnabled = true;
  }

  FastLED.show();
   
}

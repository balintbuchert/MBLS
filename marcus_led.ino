#include <FastLED.h>

// How many leds in your strip?
#define NUM_LEDS 6

// For led chips like Neopixels, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
#define DATA_PIN 5
//#define CLOCK_PIN 13
#define BRIGHTNESS  64
#define HELLO_LED 13

// Define the array of leds
CRGB leds[NUM_LEDS];
int level = 0; // led level;
int max_hight = 0;
int peek_reset = 20; // sec to reset the max peek. 
int peek_reset_counter = 0;

int mi = 0;
void setup() { 
      Serial.begin(9600);
      Serial.println("start");
      // Uncomment/edit one of the following lines for your leds arrangement.
      
      FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
      pinMode(HELLO_LED, OUTPUT); // live hello wolrd led

       FastLED.setBrightness( BRIGHTNESS );
       //leds[0] = CRGB( 255, 0, 0); 
       //leds[1] = CRGB::Orange;
       //leds[2] = CRGB::Yellow;
       //leds[3] = CRGB::Green;
       //leds[4] = CRGB::Cyan;
       //leds[5] = CRGB::White;
       palette();
       
}

CRGBPalette16 currentPalette;

void palette()
{
    // 'black out' all 16 palette entries...
    //fill_solid( currentPalette, 16, CRGB::Black);
    // and set every fourth one to white.
    currentPalette[0] = CRGB::Red; 
    currentPalette[1] = CRGB::Orange;
    currentPalette[2] = CRGB::Yellow;
    currentPalette[3] = CRGB::Green;
    currentPalette[4] = CRGB::Cyan;
    currentPalette[5] = CRGB::White;
    
}


int increaseReset() {
    peek_reset_counter= peek_reset_counter+1 ;
}


void dim( int hight){
  int i = 0;
  int l = hight; // led hight
  int m = 6;
  int mh = max_hight;

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

  if (level >= 6)        
    flash();
  else{   
    for (i = 0 ; i <= l ;i++){
      leds[i-1] = currentPalette[i-1];
      //FastLED.show(); 
      //delay(100);
      }
    //FastLED.show(); 
    for (i = l+1 ; i <= 6 ;i++){
      leds[i-1] = CRGB::Black; ; 
    //delay(100);
    //FastLED.show();
    }
  }
  if(level >= mh){
    max_hight = level;
    peek_reset_counter = 0;
   }
  
    if(peek_reset_counter > peek_reset){
        max_hight = level; 
        //Serial.print("anyad?");  
        peek_reset_counter = 0;
    }
    
 
    Serial.print("peek_level: ");
    Serial.println(peek_reset_counter);
    Serial.println(peek_reset);
 
    leds[max_hight-1] = CRGB::Red;

  
  FastLED.show();
 
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
    while(f <= 6){     
      leds[f-1] = CRGB::White;
      f++;    
    }
    flash_state = false;
    //FastLED.show();
  } // end if
 else{
  f = 0;
    while(f <= 5){
      leds[f-1] = CRGB::Black;
      f++;
  }
  flash_state = true;
  //FastLED.show();
 }// end else 
  
}



/** */
void loop() { 
    
  // Turn the LED on, then pause


  Serial.println("live");
  helloWolrd();
  delay(500);
  //increaseReset();

  peek_reset_counter+=1;
  int sensorValue = analogRead(A0);
  // print out the value you read:
  Serial.println(sensorValue);

  level = sensorValue/168;
  ///dim(level);
  Serial.print("LEVEL:");
  Serial.println(level);
  //if (level >= 6)        
    //flash();
  //else
     dim(level);

  Serial.print("LEVEL_MAX: ");
  Serial.println(max_hight);


  FastLED.show();
    
  
  
  /*int u = 0;
  
  while(u <= 6){
  dim(u);
  FastLED.show();
  Serial.println(u);
  delay(500);
  u++;
  }
*/
 
}

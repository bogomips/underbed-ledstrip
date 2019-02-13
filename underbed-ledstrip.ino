
#include <Adafruit_DotStar.h>
#include <timer.h>
#include <SPI.h>         // COMMENT OUT THIS LINE FOR GEMMA OR TRINKET
#include <ESP8266WiFi.h>
#include <WiFiUDP.h>

#define PIR_pin D1    //Digital input for PIR sensor
#define NUMPIXELS 144 // Number of LEDs in strip

// Network SSID
const char* ssid = "your ssid";
const char* password = " your wifi password";

uint32_t current_color;
WiFiUDP Udp;
unsigned int localUdpPort = 4210;
char incomingPacket[255];
//char replyPacket[] = "Hi there! Got the message :-)";
byte working_mode = 0;
auto timer = timer_create_default();
int PIR_state = LOW;             // by default, no motion detected
int PIR_lock = 0;
int light_on_timer = 15; 

// Here's how to control the LEDs from any two pins:
//#define DATAPIN    D1
//#define CLOCKPIN   D2
//Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

// Hardware SPI is a little faster, but must be wired to specific pins
// (Arduino Uno = pin 11 for data, 13 for clock, other boards are different).
// FOR wemos D1 mini lite we use D5 (blu wire) ad D7 (green)
Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, DOTSTAR_BGR);

//Static IP address configuration, change it for your needs
IPAddress staticIP(192, 168, 1, 232); //ESP static ip
IPAddress gateway(192, 168, 1, 1);   //IP Address of your WiFi Router (Gateway)
IPAddress subnet(255, 255, 255, 0);  //Subnet mask
IPAddress dns(8, 8, 8, 8);  //DNS

void connectWiFi() {

  byte ledStatus = LOW;
  
  WiFi.mode( WIFI_STA );
  WiFi.config(staticIP, gateway, subnet, dns);

    // Connect WiFi
  WiFi.hostname("bed_strip");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite( BUILTIN_LED, ledStatus ); // Write LED high/low.
    ledStatus = ( ledStatus == HIGH ) ? LOW : HIGH;
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println( WiFi.localIP() );
 
  // Print the IP address
  Serial.print("IP address: ");
  Serial.print(WiFi.localIP());
  Udp.begin(localUdpPort);
  
}

/////// wrapping functions for adafruid lib
bool night_mode_led_off (void *) {
  whole_strip(0x000000,false);
  PIR_lock = 0;
  Serial.printf("night_mode_led_off, pir enable\n");
  return true;
}

void whole_strip(uint32_t color, bool save_last_color) {

   for (int i=0; i < 144; i++) {
    strip.setPixelColor(i, color); // 'On' pixel at head
    //strip.setPixelColor(tail, 0);     // 'Off' pixel at tail
   }
   
   strip.show();                     // Refresh strip

   if (save_last_color)
    current_color = color;
}

/////// end wrapping functions

/////////start rainbow functions

void rainbowCycle(int SpeedDelay) {
  byte *c;
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< NUMPIXELS; i++) {
      c=Wheel(((i * 256 / NUMPIXELS) + j) & 255);
      strip.setPixelColor(i, *c, *(c+1), *(c+2));
    }
    strip.show();
    delay(SpeedDelay);

    
  }
}

byte * Wheel(byte WheelPos) {
  static byte c[3];
  
  if(WheelPos < 85) {
   c[0]=WheelPos * 3;
   c[1]=255 - WheelPos * 3;
   c[2]=0;
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   c[0]=255 - WheelPos * 3;
   c[1]=0;
   c[2]=WheelPos * 3;
  } else {
   WheelPos -= 170;
   c[0]=0;
   c[1]=WheelPos * 3;
   c[2]=255 - WheelPos * 3;
  }

  return c;
}
/////////end rainbox functions


///////////////////// crossfade, move as separate file
void crossFade(uint32_t color) {

  int oR = current_color >> 16;
  int oG = (current_color & 0x00ff00) >> 8;
  int oB = (current_color & 0x0000ff);
  
  int nR = color >> 16;
  int nG = (color & 0x00ff00) >> 8;
  int nB = (color & 0x0000ff);
  
  int tR = oR;
  int tG = oG;
  int tB = oB;
  
  int stepR = calculateStep(oR, nR);
  int stepG = calculateStep(oG, nG); 
  int stepB = calculateStep(oB, nB);
  
  for (int i = 0; i <= 600; i++) {
   
    tR = calculateVal(stepR, tR, i);
    tG = calculateVal(stepG, tG, i);
    tB = calculateVal(stepB, tB, i);

    for (int y=0; y < 144; y++) {
      strip.setPixelColor(y, tR, tG ,tB);
    }
    strip.show(); 

    //delay(1);

  }

  current_color = color;
  
}

int calculateStep(int prevValue, int endValue) {
  int step = endValue - prevValue; // What's the overall gap?
  if (step) {                      // If its non-zero, 
    step = 600/step;              //   divide by 1020
  } 
  return step;
}

int calculateVal(int step, int val, int i) {

  if ((step) && i % step == 0) { // If step is non-zero and its time to change a value,
    if (step > 0) {              //   increment the value if step is positive...
      val += 1;           
    } 
    else if (step < 0) {         //   ...or decrement it if step is negative
      val -= 1;
    } 
  }
  // Defensive driving: make sure val stays in the range 0-255
  if (val > 255) {
    val = 255;
  } 
  else if (val < 0) {
    val = 0;
  }
  return val;
}

// end of crossfade

void setup() {

 Serial.begin(115200);
  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000L)
    clock_prescale_set(clock_div_1); // Enable 16 MHz on Trinket
  #endif

  current_color = 0x000000;
  strip.begin(); // Initialize pins for output
  strip.show();  // Turn all LEDs off ASAP

  connectWiFi();

}


void loop() {

  timer.tick();
   
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0;
    }
    
    Serial.printf("UDP packet contents: %s\n", incomingPacket);
    uint32_t udp_color = (int)strtol(incomingPacket, NULL, 0);

    
    if (strcmp(incomingPacket,"0x010000") == 0) { 
      working_mode = 0; // Direct
    }

    else if (strcmp(incomingPacket,"0x000100") == 0) { 
      working_mode = 1; // Crossfade
    }
    else if (strcmp(incomingPacket,"0x000001") == 0) { 
      working_mode = 2; // Crossfade
    }
    else if (strcmp(incomingPacket,"0x010101") == 0) { 
      working_mode = 3; // Crossfade
    }

    Serial.printf("Working Mode: %d\n", working_mode);
    if (working_mode == 0) {
      whole_strip(udp_color,true);
    }
    else if (working_mode == 1) {
      crossFade(udp_color);
    }
    else if (working_mode == 2) {
      whole_strip(0x000000,false);
    }
    else if (working_mode == 3) {
      whole_strip(0x000000,false);
    }

    // send back a reply, to the IP address and port we got the packet from
    //Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    //Udp.write(replyPacket);
    //Udp.endPacket();

  }

  if (working_mode == 2) {

    //Serial.printf("PIR: %d\n", digitalRead(PIR_pin));
    if (PIR_lock == 0) {
      if (digitalRead(PIR_pin) == HIGH) {
        PIR_lock = 1;
        Serial.printf("Motion detected\n");
        whole_strip(current_color,false);
        timer.in(light_on_timer * 1000, night_mode_led_off);  
      }
    }
    
  }
  else if (working_mode == 3) {
    rainbowCycle(10);
    //BouncingBalls(0xff,0,0, 3);
    //meteorRain(0xff,0xff,0xff,10, 64, true, 30);
    //NewKITT(0xff, 0, 0, 8, 10, 50);
  }
  
}




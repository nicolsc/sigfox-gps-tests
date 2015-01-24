#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "Akeru.h"
#include <stdlib.h>

// L LED is connected to pin 13, give it a name:
int led = 13;

SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

#define GPSECHO  true

boolean usingInterrupt = false;

// the setup routine runs once when you press reset:
void setup() {
  
  // initialize the digital pin of L led as an output
  pinMode(led, OUTPUT);  
  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Serial.println("Starting...");
  
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  
  // Wait 1 second for the modem to warm
  delay(1000);
  mySerial.println(PMTK_Q_RELEASE);
  
  // Init modem
  Akeru.begin();
}

uint32_t timer = millis();
void loop() {
  
  Serial.println("Loop !");
  
  // Wait for availability of the modem
  while (!Akeru.isReady()) {
    Serial.println("Modem not ready");
    delay(1000);
  }
  
  char c = GPS.read();
  if (c) Serial.print(c);
  
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    
    timer = millis(); // reset the timer
    
    // LED is on
    digitalWrite(led, HIGH);
//  
//    Serial.print("\nTime: ");
//    Serial.print(GPS.hour, DEC); Serial.print(':');
//    Serial.print(GPS.minute, DEC); Serial.print(':');
//    Serial.print(GPS.seconds, DEC); Serial.print('.');
//    Serial.println(GPS.milliseconds);
//    Serial.print("Date: ");
//    Serial.print(GPS.day, DEC); Serial.print('/');
//    Serial.print(GPS.month, DEC); Serial.print("/20");
//    Serial.println(GPS.year, DEC);
//    Serial.print("Fix: "); Serial.print((int)GPS.fix);
//    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
//    
//    if (GPS.fix) {
//      Serial.print("Location: ");
//      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
//      Serial.print(", "); 
//      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
//      Serial.print("Location (in degrees, works with Google Maps): ");
//      Serial.print(GPS.latitudeDegrees, 4);
//      Serial.print(", "); 
//      Serial.println(GPS.longitudeDegrees, 4);
//      
//      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
//      Serial.print("Angle: "); Serial.println(GPS.angle);
//      Serial.print("Altitude: "); Serial.println(GPS.altitude);
//      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
//    }
    
    char lat[5];
    dtostrf(GPS.lat, 1, 3, lat);
    char lon[5];
    dtostrf(GPS.lon, 1, 3, lon);
    char alt[5];
    dtostrf(GPS.altitude, 1, 3, alt);
    
    // Send in the mighty internet!
    char *data = "";
    
    strcat(data, lat);
    strcat(data, ";");
    strcat(data, lon);
    strcat(data, ";");
    strcat(data, alt);
    
    Serial.println("Sending data ...");
    Serial.println(data);
    
    Akeru.send(&data, sizeof(data));
    
    // LED is off
    digitalWrite(led, LOW);
    
    // Wait for 10 minutes. LULZ
    for (int second = 0; second < 600; second++) {
      //delay(1000);
    }
  }
}


char * floatToString(char * outstr, double val, byte precision, byte widthp){
 char temp[16]; //increase this if you need more digits than 15
 byte i;

 temp[0]='\0';
 outstr[0]='\0';

 if(val < 0.0){
   strcpy(outstr,"-\0");  //print "-" sign
   val *= -1;
 }

 if( precision == 0) {
   strcat(outstr, ltoa(round(val),temp,10));  //prints the int part
 }
 else {
   unsigned long frac, mult = 1;
   byte padding = precision-1;
   
   while (precision--)
     mult *= 10;

   val += 0.5/(float)mult;      // compute rounding factor
   
   strcat(outstr, ltoa(floor(val),temp,10));  //prints the integer part without rounding
   strcat(outstr, ".\0"); // print the decimal point

   frac = (val - floor(val)) * mult;

   unsigned long frac1 = frac;

   while(frac1 /= 10) 
     padding--;

   while(padding--) 
     strcat(outstr,"0\0");    // print padding zeros

   strcat(outstr,ltoa(frac,temp,10));  // print fraction part
 }

 // generate width space padding 
 if ((widthp != 0)&&(widthp >= strlen(outstr))){
   byte J=0;
   J = widthp - strlen(outstr);

   for (i=0; i< J; i++) {
     temp[i] = ' ';
   }

   temp[i++] = '\0';
   strcat(temp,outstr);
   strcpy(outstr,temp);
 }

 return outstr;
}

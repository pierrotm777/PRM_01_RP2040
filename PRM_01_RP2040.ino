/*
 * This code is inspired by the Aruna code
 * https://github.com/Archfx/DIY-PRM_01/blob/master/prm01.ino
*/

#include <Misclib.h>
#include <Wire.h>
#include <TinyGPS++.h>    // https://github.com/mikalhart/TinyGPSPlus
#include <SoftwareSerial.h>
#include <Adafruit_BMP280.h>  // https://github.com/adafruit/Adafruit_BMP280_Library
#include <RunningAverage.h>   // https://github.com/RobTillaart/RunningAverage

#include <SerialCommand.h>	//https://github.com/kroimon/Arduino-SerialCommand/tree/master
SerialCommand sCmd;     // The demo SerialCommand object

#include <FastLED.h> //https://github.com/FastLED/FastLED
// How many leds in your strip?
#define NUM_LEDS 1
#define DATA_PIN 16
#define BRIGHTNESS 10

// Define the array of leds
CRGB leds[NUM_LEDS];

struct STREAM_DATA {
  float battVoltage = 12.75;
  float altitude = 10;
  float climb = 0;
  float roll = 1;
  float pitch = 2;
  float yaw = 3;
  uint8_t gps_sats = 5;
  float gps_lon = -0.6;//aérodrome
  float gps_lat = 44.73333;//aérodrome
  float home_lon = -0.5672574;//Villenave d'Ornon
  float home_lat = 44.7799813;//Villenave d'Ornon
  uint16_t gps_speed = 100;
  boolean gps_fix = false;
  uint32_t home_distance = 220;
};

STREAM_DATA streamData;

#define BATT_PIN        26 //pin 26
unsigned long tmrVolt = 0;
float voltCoef = 4.31;     // 1/(r2/(r1+r2)) R1 and R2 are a resistors devider

TinyGPSPlus gps;
// GPS Module (Serial)
#define GPS_RX_PIN      12
#define GPS_TX_PIN      13
#define GPS_BAUD        9600         // GPS module must be configurate to this speed,
// The serial connection to the GPS device
SoftwareSerial ss(GPS_RX_PIN, GPS_TX_PIN);
bool GpsFix = false;
bool GpsAvailable = false;
bool homeLatLonOk = false;

Adafruit_BMP280 bmp(&Wire1);  // BMP280 connected using I2C
//RunningAverage AltAverage(10);
//RunningAverage VspeedAverage(10);
bool BMP280Available = false;
//unsigned long temps;  //sert à enregistrer le temps écoulé depuis la dernière mise à jour du compteur

/*
* Change based on your city's barometric pressure
* http://www.meteociel.fr/observations-meteo/pression.php
*/
float pressureAtStartAltitude = 1020;  //  Standard pressure at sea level in millibar for Bordeaux
float startAltitude = 0;
float vSpeed = 0;

boolean packetSet = false;

/* Air Speed Sensor */
#include <ms4525do.h>	https://github.com/bolderflight/ms4525do
//I2C library 0x28H
bfs::Ms4525do pres;
bool MS4525DOAvailable = false;
float pressionMS4525;
/* Air Speed Sensor */

/* Accéléromètre 3 axes et un gyroscope 3 axes */
#include <Adafruit_MPU6050.h>	//https://github.com/adafruit/Adafruit_MPU6050
Adafruit_MPU6050 mpu;
bool MPU6050Available = false;

#define PRINT_BUF_SIZE   100
static char PrintBuf[PRINT_BUF_SIZE + 1];

#define PRINTF(fmt, ...)    do{if(Serial){snPRINT_P(PrintBuf, PRINT_BUF_SIZE, PSTR(fmt) ,##__VA_ARGS__);Serial.print(PrintBuf);}}while(0)
#define PRINT_P(FlashStr)   do{if(Serial){Serial.print(FlashStr);}}while(0)
  

#define PROJECT_NAME    RadioLink Telemetry For RP2040-Zero
#define FW_VERSION      0
#define FW_REVISION     5

#define RADIOLINK_VER_REV     PRJ_VER_REV(PROJECT_NAME, FW_VERSION, FW_REVISION) /* ProjectName VVersion.Revision */

#define DBG_PRINT(...)           Serial.print(__VA_ARGS__)
#define DBG_PRINTLN(...)         Serial.println(__VA_ARGS__)

bool printGPS = false;
bool printMPU6050 = false;
bool printBMP280 = false;
bool printVOLT = false;
bool printPITOT = false;
bool checkLED = false;
bool SetupIsDone = false;

unsigned statusBMP280;

void setup() {

  Serial.begin(115200);
  ss.begin(GPS_BAUD);  // Init GPS
  
  //FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);  /* GRB = Inverted Red and Green*/
  FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);  /* RGB = NORMAL */
  FastLED.setBrightness( BRIGHTNESS );
  leds[0] = CRGB::Red;
  FastLED.show();
  FastLED.delay(1000);
  
  // Setup callbacks for SerialCommand command
  sCmd.addCommand("H",  displayHelp);   // Display Help & Version
  sCmd.addCommand("C",  setupCheck);    // Check Components
  sCmd.addCommand("B",  OnOffBMP280);   // Display bmp280
  sCmd.addCommand("G",  OnOffGPS);      // Display gps
  sCmd.addCommand("M",  OnOffMPU6050);  // Display mpu6050
  sCmd.addCommand("V",  OnOffVOLT);     // Display Voltage
  sCmd.addCommand("P",  OnOffPITOT);    // Disqplay Pitot
  sCmd.addCommand("L",  OnOffLED);      // Display Led

  sCmd.setDefaultHandler(unrecognized);             // Handler for command that isn't matched  (says "What?")

  // battery voltage pin init
  pinMode(BATT_PIN, INPUT);
  
  // I2C1 port for I2C sensors
  Wire1.setSDA(10);
  Wire1.setSCL(11);
  Wire1.setClock(400000);
  Wire1.begin();
  //checkI2C1();

  // RadioLink Telemetry I2C0 port
/*
  --- GND \
  --- +Vx | oXs Board
  --- SDA |   J9
  --- SCL /
*/
  Wire.setSDA(8);
  Wire.setSCL(9);
  Wire.setClock(400000);
  Wire.begin(4);
  Wire.onRequest(onRequest);

//  AltAverage.clear();
//  VspeedAverage.clear();
//  temps=millis();
  statusBMP280 = bmp.begin();// 0x76 or 0x77
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  startAltitude = bmp.readAltitude(pressureAtStartAltitude);
                  
  delay(1000);
  
  if (ss.available())
  {
    GpsAvailable = true;
  }
  else
  {
    GpsAvailable = false;
  }

  MS4525DO_Init();
  MPU6050_Init();
        
}

void setupCheck()
{
  PRINT_P("BMP280 ");// Use the I2C1 port
  if (statusBMP280) 
  {  
    // Default settings from datasheet.
    PRINT_P("OK...\r\n");
    startAltitude = bmp.readAltitude(pressureAtStartAltitude);
  } 
  else 
  {
    PRINT_P("Not found...\r\n");
  }
    
  if (ss.available())
  {
    PRINT_P("GPS OK...\r\n");
  }
  else
  {
    PRINT_P("GPS Not found...\r\n");
  }
  
  PRINT_P(F("MS4525DO "));
  if (MS4525DOAvailable== true)
    PRINT_P(F("OK...\r\n"));
  else
    PRINT_P(F("Not found...\r\n"));  

  PRINT_P(F("MPU6050 "));
  if (MPU6050Available == true)
    PRINT_P(F("OK..."));
  else
    PRINT_P(F("Not found..."));

  PRINT_P("\r\n\r\nReady\r\n");  
}

void loop() {

  /* Check Serial Cmd */
   sCmd.readSerial();     // We don't do much, just process serial commands
  /* Check Serial Cmd */
  
  if (millis() - tmrVolt >= 800)  
  {
    float sum = 0.0;
    analogReadResolution(16);
    for (int i = 0; i < 10; i++) {
      sum += (analogRead(BATT_PIN) * 3.3 /65534.0); //65534 ==> 16 bits, 4096 ==> 12 bits, 1024 ==> 10 bits
    }
    sum = sum / 10.0 * voltCoef;  /* sum = sum / 10 * (1/(r2/(r1+r2))); */
    if (sum > 1)
      streamData.battVoltage = sum;    
    else
      streamData.battVoltage = 0.0;

    if (printVOLT == true)
    {
      PRINT_P("Voltage raw: ");
      Serial.print(analogRead(BATT_PIN));
      PRINT_P(", ");
      Serial.print(streamData.battVoltage,2);
      PRINT_P("V, k=");
      Serial.print(voltCoef);
      PRINT_P("\r\n");       
    }
    tmrVolt = millis();
  }

//  AltAverage.addValue(bmp.readAltitude(pressureAtStartAltitude));
//  VspeedAverage.addValue((AltAverage.getAverage() - startAltitude) * 10.0f);  // calculate vertical speed
  streamData.altitude = bmp.readAltitude(pressureAtStartAltitude) - startAltitude;
  streamData.climb = streamData.altitude * 10.0f;//VspeedAverage.getAverage();
  if (printBMP280 == true)
  {
//    PRINT_P("Altitude=");PRINT_P(AltAverage.getAverage() - startAltitude);PRINT_P("\t");
//    PRINT_P("VSpeed=");PRINT_P(VspeedAverage.getAverage());PRINT_P("\r\n");

    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.print("°C\t");
    
    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure()/100);
    Serial.print(" Pa\t");
    
    Serial.print(F("Relative altitude = "));
    Serial.print(streamData.altitude);
    Serial.print(" m\t");

    Serial.print(F("Verticale Speed = "));
    Serial.print(streamData.climb);
    Serial.println(" m/s");
    Serial.println();

    delay(500);    
  }
  if (GpsAvailable == true)
  {
    //Serial.print(gps.location.lng(), 6);Serial.print(F(","));Serial.println(gps.location.lat(), 6);
    if (printGPS == true) displayGPS();
    streamData.gps_sats = gps.satellites.value();
    if ((homeLatLonOk == false) && gps.location.isValid())
    {
      streamData.home_lat = gps.location.lat();
      streamData.home_lon = gps.location.lng();
      homeLatLonOk = true;
    }
    streamData.gps_lat = gps.location.lat();
    streamData.gps_lon = gps.location.lng();
//    streamData.altitude = gps.altitude.meters();// bmp280 return relative altitude
    streamData.gps_speed = gps.speed.kmph();
    smartDelay(500);   
  }
  else
  {
    Serial.println("Gps not available");delay(1000);
  }

  if (checkLED == false)
  {
    if (GpsAvailable == true)
    {
      if (gps.satellites.value() >= 4)
      {
        /* For colors, see http://fastled.io/docs/3.1/pixeltypes_8h_source.html */
        /* Blink LED Management */
        leds[0] = CRGB::Green;
        FastLED.show();
        FastLED.delay(500);
        // Now turn the LED off, then pause
        leds[0] = CRGB::Black;
        FastLED.show();
        FastLED.delay(500);      
      }
      else
      {
        /* Blink LED Management */
        leds[0] = CRGB::Orange;
        FastLED.show();
        FastLED.delay(500);
        // Now turn the LED off, then pause
        leds[0] = CRGB::Black;
        FastLED.show();
        FastLED.delay(500);          
      }
  
    }
    else
    {
      /* Blink LED Management */
      leds[0] = CRGB::Red;
      FastLED.show();
      FastLED.delay(500);
      // Now turn the LED off, then pause
      leds[0] = CRGB::Black;
      FastLED.show();
      FastLED.delay(500);      
    }
  }
  else // checkLED
  {
      leds[0] = CRGB::Red;
      FastLED.show();
      FastLED.delay(500);
      leds[0] = CRGB::Orange;
      FastLED.show();
      FastLED.delay(500);
      leds[0] = CRGB::Green;
      FastLED.show();
      FastLED.delay(500);          
  }

  uint16_t AirSpeed1 = MS4525();

  MPU6050_Loop();
  
}//loop()

#define d2r (M_PI / 180.0)
float calc_dist(float lat1, float long1, float lat2, float long2) {
  double dlong = (long2 - long1) * d2r;
  double dlat = (lat2 - lat1) * d2r;
  double a = pow(sin(dlat / 2.0), 2) + cos(lat1 * d2r) * cos(lat2 * d2r) * pow(sin(dlong / 2.0), 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float d = 6367 * c * 1000; // in meters
  if (d >= 0) {
    return d;
  } else {
    return -d;
  }
}

void set1() {

  int  alt = streamData.altitude / 10;//V5
  //int  alt = AltAverage.getAverage()*10;
  byte  altHi = highByte(alt )  ;
  byte  altLo =  lowByte(alt ) ;

  int  yaw = streamData.yaw * 100;
  byte yawHi = highByte(yaw );
  byte yawLo = lowByte(yaw );


  //int  speed2 = streamData.gps_speed * 1000;//V5
  int  speed2 = gps.speed.kmph() * 10;//V5
  //int  speed2 = pressionMS4525 * 10;
  byte speedHi = highByte(speed2);
  byte speedLo = lowByte(speed2);


  int  roll = streamData.roll * 100;
  byte rollHi = highByte(roll);
  byte rollLo = lowByte(roll);


  int  pitch = streamData.pitch * 100;
  byte pitchHi = highByte(pitch);
  byte pitchLo = lowByte(pitch);

  int distance = calc_dist(streamData.home_lat, streamData.home_lon, gps.location.lat(), gps.location.lng()) * 10;
  byte distanceHi = highByte(distance);
  byte distanceLo = lowByte(distance);

  byte buffer[16] = {0x89, 0xAB, 
                      streamData.gps_sats, 
                      altHi, altLo, 
                      yawHi, yawLo, 
                      speedHi, speedLo, 
                      rollHi , rollLo, 
                      pitchHi, pitchLo, 
                      distanceHi, distanceLo, 
                      0x00};
  Wire.write(buffer, 16);
}

void set2() {

  int  rise = streamData.climb * 10;
  byte  riseHi = highByte(rise);
  byte  riseLo =  lowByte(rise);

  uint16_t voltes = streamData.battVoltage * 100.0;//15000; // 15V
  byte voltesHi = highByte(voltes);
  byte voltesLo = lowByte(voltes);

  union u32_tag  {
    byte  b[4];
    uint32_t ui32;
  } latit, longt;
  longt.ui32 = gps.location.lng() * 10000000;//V5
  latit.ui32 = gps.location.lat() * 10000000;//V5;
  byte sat = streamData.gps_sats;// satelites count
  //byte buffer[16] = {0x89, 0xCD, streamData.gps_sats, riseHi, riseLo, voltesHi, voltesLo,/**/ 0x0, 0x0, 0x0 , 0x0, /**/0x0, 0x0, 0x0, 0x0,/**/ 0x00};
  byte buffer[16] = {0x89, 0xCD,
                      sat, 
                      riseHi, riseLo, 
                      voltesHi, voltesLo, 
                      longt.b[3], longt.b[2], longt.b[1] , longt.b[0],
                      latit.b[3], latit.b[2], latit.b[1] , latit.b[0], 
                      0x00};  
  Wire.write(buffer, 16);
}


void onRequest() 
{
  if (packetSet) 
  {
    packetSet = false;
    set1();
  } 
  else 
  {
    packetSet = true;
    set2();
  }
}


void displayGPS()
{  
  PRINT_P("Location: "); 
  if (gps.location.isValid())
  {
    GpsFix = true;
      Serial.print(gps.location.lat(),6);
      Serial.print(F(","));
      Serial.print(gps.location.lng(),6);
  }
  else
  {
    GpsFix = false;
    PRINT_P("INVALID");
  }

  PRINT_P(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    PRINT_P(gps.date.month());
    PRINT_P(F("/"));
    PRINT_P(gps.date.day());
    PRINT_P(F("/"));
    PRINT_P(gps.date.year());
  }
  else
  {
    PRINT_P(F("INVALID"));
  }

  PRINT_P(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) PRINT_P(F("0"));
    PRINT_P(gps.time.hour());
    PRINT_P(F(":"));
    if (gps.time.minute() < 10) PRINT_P(F("0"));
    PRINT_P(gps.time.minute());
    PRINT_P(F(":"));
    if (gps.time.second() < 10) PRINT_P(F("0"));
    PRINT_P(gps.time.second());
    PRINT_P(F("."));
    if (gps.time.centisecond() < 10) PRINT_P(F("0"));
    PRINT_P(gps.time.centisecond());
  }
  else
  {
    PRINT_P(F("INVALID"));
  }
  PRINT_P("\r\n");    

}

void MS4525DO_Init() {
  pres.Config(&Wire1, 0x28, 1.0f, -1.0f);
  if (pres.Begin())
    MS4525DOAvailable = true;
  else
    MS4525DOAvailable = false;
}

float MS4525() {
  /* Read the sensor */
  if (pres.Read()) {
    pressionMS4525 = pres.pres_pa();//*3.6 for km/h
    if (printPITOT == true)
    {
      /* Display the data */
      Serial.print(pressionMS4525, 2);
      Serial.print("km/h\t");
      Serial.print(pres.die_temp_c());
      Serial.println("°C\r\n");  
    }
  }
  return pressionMS4525;
}

void MPU6050_Init() 
{
  if (mpu.begin(0x68, &Wire1))
    MPU6050Available = true;
  else
    MPU6050Available = false;
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  PRINT_P("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    PRINT_P("+-2G\r\n");
    break;
  case MPU6050_RANGE_4_G:
    PRINT_P("+-4G\r\n");
    break;
  case MPU6050_RANGE_8_G:
    PRINT_P("+-8G\r\n");
    break;
  case MPU6050_RANGE_16_G:
    PRINT_P("+-16G\r\n");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  PRINT_P("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    PRINT_P("+- 250 deg/s\r\n");
    break;
  case MPU6050_RANGE_500_DEG:
    PRINT_P("+- 500 deg/s\r\n");
    break;
  case MPU6050_RANGE_1000_DEG:
    PRINT_P("+- 1000 deg/s\r\n");
    break;
  case MPU6050_RANGE_2000_DEG:
    PRINT_P("+- 2000 deg/s\r\n");
    break;
  }
}

void MPU6050_Loop()
{
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  streamData.roll = g.gyro.x;
  streamData.pitch = g.gyro.y;
  streamData.yaw = g.gyro.z;
  
  /* Print out the values */
  if (printMPU6050 == true)
  {
    PRINT_P("Acceleration X: ");
    PRINT_P(a.acceleration.x);
    PRINT_P(", Y: ");
    PRINT_P(a.acceleration.y);
    PRINT_P(", Z: ");
    PRINT_P(a.acceleration.z);
    PRINT_P(" m/s^2\r\n");

    PRINT_P("Rotation X: ");
    PRINT_P(g.gyro.x);
    PRINT_P(", Y: ");
    PRINT_P(g.gyro.y);
    PRINT_P(", Z: ");
    PRINT_P(g.gyro.z);
    PRINT_P(" rad/s\r\n");

    PRINT_P("Temperature: ");
    PRINT_P(temp.temperature);
    PRINT_P(" degC\r\n");

    delay(500);    
  }

}

void OnOffGPS() {
   printGPS = !printGPS;
   if(printGPS==true) 
    PRINT_P("GPS display is ON:\r\n"); 
   else 
    PRINT_P("GPS display is OFF:\r\n");
}

void OnOffMPU6050() {
   printMPU6050 = !printMPU6050;
   if(printMPU6050==true) 
    PRINT_P("MPU6050 display is ON:\r\n"); 
   else 
    PRINT_P("MPU6050 display is OFF:\r\n");
}

void OnOffBMP280() {
   printBMP280 = !printBMP280;
   if(printBMP280==true) 
    PRINT_P("BMP280 display is ON:\r\n"); 
   else 
    PRINT_P("BMP280 display is OFF:\r\n");
}

void OnOffVOLT() {
   printVOLT = !printVOLT;
   if(printVOLT==true) 
    PRINT_P("VOLTAGE display is ON:\r\n"); 
   else 
    PRINT_P("VOLTAGE display is OFF:\r\n");
}

void OnOffLED() {
   checkLED = !checkLED;
   if(checkLED==true) 
    PRINT_P("LED ckeck is ON:\r\n"); 
   else 
    PRINT_P("LED check is OFF:\r\n");
}

void OnOffPITOT() {
   printPITOT = !printPITOT;
   if(printPITOT==true) 
    PRINT_P("PITOT display is ON:\r\n"); 
   else 
    PRINT_P("PITOT display is OFF:\r\n");
}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  PRINT_P("\r\nWhat?\r\n");
}

void displayHelp(void)
{ 
  DBG_PRINTLN(F(RADIOLINK_VER_REV));
  DBG_PRINTLN(F("\r\nHit Command + Enter")); 
  
  PRINT_P("H Help List\r\n");
  PRINT_P("C Check components\r\n");
  PRINT_P("B BMP280 debug\r\n");
  PRINT_P("G GPS debug\r\n");
  PRINT_P("M MPU6050 debug\r\n");
  PRINT_P("V Voltage debug\r\n");
  PRINT_P("P Pitot debug\r\n");
  PRINT_P("L Led debug\r\n\r\n");
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

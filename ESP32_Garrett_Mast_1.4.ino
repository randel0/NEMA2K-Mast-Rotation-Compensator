#include <Arduino.h>

#define ESP32_CAN_TX_PIN GPIO_NUM_32//The library defines as default Tx pin to GPIO 16 and Rx pint to GPIO 4. You can change these with defines:
#define ESP32_CAN_RX_PIN GPIO_NUM_34
#define USE_N2K_CAN 7  // for use with ESP32

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <NMEA2000_esp32.h>

#include <N2kMessages.h>

#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <vector>
#include <numeric>
#include <movingAvg.h>

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);       // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

movingAvg honeywellSensor(10);                // define the moving average object

using namespace std;

class RotationSensor {
  public:
    static int newValue;
    static int oldValue;
};

class WindSensor {
  public:
    static double windSpeedKnots;
    static int windAngleDegrees;
};

// double WindSpeedMs;
// double WindAngle;
// int WindAngleDegrees;
// double WindSpeedKnots;

typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg); 
} tNMEA2000Handler;

void WindSpeed(const tN2kMsg &N2kMsg);

tNMEA2000Handler NMEA2000Handlers[]={
  {130306L,&WindSpeed},
  {0,0}
};


// Initialize static variables for RotationSensor Class
int RotationSensor::newValue{0};
int RotationSensor::oldValue{0};

double WindSensor::windSpeedKnots{0.0};
int WindSensor::windAngleDegrees{0};

// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM={130306L,0};   // This is the PGN for Wind

void setup() {
  // Set Product information
  NMEA2000.SetProductInformation("00000002",                // Manufacturer's Model serial code
                                 100,                       // Manufacturer's product code
                                 "Mast Rotation Compensator",     // Manufacturer's Model ID
                                 "1.1.0.22 (2021-10-04)",  // Manufacturer's Software version code
                                 "1.1.0.0 (2021-10-04)"     // Manufacturer's Model version
                                 );
  // Set device information
  NMEA2000.SetDeviceInformation(1, // Unique number. Use e.g. Serial number.
                                130, // Device function=Atmospheric. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                85, // Device class=External Environment. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                               );

  
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly,23);    // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly
  NMEA2000.EnableForward(false);
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.Open();

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3D for 128x64
      honeywellSensor.begin();    //Instantiates the moving average object


  ads.begin();
  OLEDdataSplash(); 
  }


void loop() {
  
  NMEA2000.ParseMessages();

  int mastRotate = readAnalogRotationValue();
  int windInput = readWindAngleInput();      //windinput value should be 0-359 as function of analog input

  int anglesum = windInput + mastRotate;                    //adds windinput and mastrotate

  int rotateout;

  if (anglesum<0) {                             //ensure sum is 0-359
    rotateout = anglesum + 360;
  }              
  else if (anglesum>359) {   
    rotateout = anglesum - 360;               
  }
  else {
    rotateout = anglesum;               
  }

   SendN2kWind(rotateout);

  // Output Debug TO OLED
   
   // int newValue = RotationSensor::newValue;
   // OLEDdataWriteDebug(newValue, windInput, mastRotate, rotateout);
   OLEDdataWind(mastRotate, rotateout); 
}

double ReadWindAngle(int rotateout) {
  return DegToRad(rotateout); // 
}


double ReadWindSpeed() {
  // return 10.3; // Read here the wind speed e.g. from analog input - output currently 20 - ReadWindSpeed x 1.94384 = Output in Knots - This is where the real windspeed needs to go
  return WindSensor::windSpeedKnots;
}

#define WindUpdatePeriod 1000

int readWindAngleInput() {
  return  WindSensor::windAngleDegrees;
}

int readAnalogRotationValue(){

  // Define Constants
  const int lowset = 120;
  const int highset = 1024;
  
  int adc1 = (ads.readADC_SingleEnded(1)>>4);      //reads bitshifted value of Honeywell position
  int newValue = honeywellSensor.reading(adc1);    // calculate the moving average
  int oldValue = RotationSensor::oldValue;
  
  if (newValue < highset){                 //writes value to oldsensor if below highset threshold
    oldValue = newValue;
  }

  // Update values for new and old values (for the next loop iteration)
  RotationSensor::newValue = newValue;
  RotationSensor::oldValue = oldValue;

  int mastRotate = map(oldValue, lowset, highset, -50, 50);    //maps 10 bit number to degrees of rotation
  return mastRotate;
}

void WindSpeed(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    double windSpeedMetersSeconds;
    double windAngle;
    tN2kWindReference WindReference;

    if (ParseN2kWindSpeed(N2kMsg,SID, windSpeedMetersSeconds, windAngle, WindReference) ) {
      double windSpeedKnots = windSpeedMetersSeconds * 1.94384; //maybe * .01?
      int windAngleDegrees = windAngle * 57.2958; //maybe * .0001? 

      // Update Static Object Values for Wind Velocity and Angle
      WindSensor::windSpeedKnots = windSpeedKnots;
      WindSensor::windAngleDegrees = windAngleDegrees;
      
     }
}

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {   //NMEA 2000 message handler
  int iHandler;
  // Find handler
  for (iHandler=0; NMEA2000Handlers[iHandler].PGN!=0 && !(N2kMsg.PGN==NMEA2000Handlers[iHandler].PGN); iHandler++);
  if (NMEA2000Handlers[iHandler].PGN!=0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg); 
  }
}


void OLEDdataWrite(int mastrotate) {                    // Puts the data on the OLED Screen
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Mast");
  display.setCursor(0, 24);
  display.println("Rotation");
  display.setCursor(0, 48);
  display.println(mastrotate);
  display.display(); 
}

void OLEDdataWriteDebug(int newValue, int windinput, int mastrotate, int rotateout) {                    // Puts the data on the OLED Screen
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(windinput);
  display.setCursor(30, 0);
  display.println("windinput");
  display.setCursor(0, 12);
  display.println(mastrotate);
  display.setCursor(30, 12);
  display.println("mastrotate");
  display.setCursor(0, 24);
  display.println(rotateout);
  display.setCursor(30, 24);
  display.println("rotateout");
  display.setCursor(0, 36);
  display.println(newValue);
  display.setCursor(30, 36);
  display.println("newValue");
//  display.setCursor(0, 48);
//  display.println(mean);
//  display.setCursor(30, 48);
//  display.println("bufferMean");
  display.display(); 
}

void OLEDdataWind(int mastrotate, int rotateout) {                           // Splash OLED Screen
  double windSpeedKnots = WindSensor::windSpeedKnots;
  int windAngleDegrees = WindSensor::windAngleDegrees;
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(windSpeedKnots);
  display.setCursor(30, 0);
  display.println("WindSpeed");
  display.setCursor(0, 12);
  display.println(windAngleDegrees);
  display.setCursor(30, 12);
  display.println("WindAngle");
  display.setCursor(0, 24);
  display.println(rotateout);
  display.setCursor(30, 24);
  display.println("Corrected WindAngle");
  display.setCursor(0, 36);
  display.println(mastrotate);
  display.setCursor(30, 36);
  display.println("Mast Rotation");
  display.display();
}

void OLEDdataSplash() {                    // Splash OLED Screen
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Mast Rotation");
  display.setCursor(0, 12);
  display.println("Compensator");
  display.setCursor(0, 24);
  display.println("1.4A");
  display.display();
  delay(1000);                // waits 
}

void SendN2kWind(int rotateout) {
  static unsigned long WindUpdated=millis();
  tN2kMsg N2kMsg;

  if ( WindUpdated+WindUpdatePeriod<millis() ) {
    SetN2kWindSpeed(N2kMsg, 1, ReadWindSpeed(), ReadWindAngle(rotateout),N2kWind_Apprent);  // Typo in N2kWindApprent is intentional
    WindUpdated=millis();
    NMEA2000.SendMsg(N2kMsg);
  }
}

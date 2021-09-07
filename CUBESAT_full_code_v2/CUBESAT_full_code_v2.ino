//ArduCam Setup

#include <Wire.h>
#include <ArduCAM.h>
#include "memorysaver.h"
#if !(defined (OV5640_MINI_5MP_PLUS)||defined (OV5642_MINI_5MP_PLUS))
#error Please select the hardware platform and camera module in the ../libraries/ArduCAM/memorysaver.h file
#endif
#define   FRAMES_NUM    0x00
const int CS = 3;
#define SD_CS 5
bool is_header = false;
int total_time = 0;
#if defined (OV5640_MINI_5MP_PLUS)
  ArduCAM myCAM(OV5640, CS);
#else
  ArduCAM myCAM(OV5642, CS);
#endif
uint8_t read_fifo_burst(ArduCAM myCAM);

//Temperature Sensor Setup

#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 0
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);


//ENV Shield Setup
#include <Arduino_MKRENV.h>

//RTC Setup
#include "RTClib.h"
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};


//SD card breakout setup
#include <SPI.h>
#include <SD.h>
File myFile;




//Grove Particle Sensor Setup
#include <Seeed_HM330X.h>

#ifdef  ARDUINO_SAMD_VARIANT_COMPLIANCE
    #define SERIAL_OUTPUT SerialUSB
#else
    #define SERIAL_OUTPUT Serial
#endif

HM330X sensor;
uint8_t buf[30];


const char* str[] = {"sensor num: ", "PM1.0 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM2.5 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM10 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM1.0 concentration(Atmospheric environment,unit:ug/m3): ",
                     "PM2.5 concentration(Atmospheric environment,unit:ug/m3): ",
                     "PM10 concentration(Atmospheric environment,unit:ug/m3): ",
                    };

HM330XErrorCode print_result(const char* str, uint16_t value) {
    if (NULL == str) {
        return ERROR_PARAM;
    }
    SERIAL_OUTPUT.print(str);
    SERIAL_OUTPUT.println(value);
    return NO_ERROR;
}

HM330XErrorCode parse_result(uint8_t* data) {
    uint16_t value = 0;
    if (NULL == data) {
        return ERROR_PARAM;
    }
    for (int i = 1; i < 8; i++) {
        value = (uint16_t) data[i * 2] << 8 | data[i * 2 + 1];
        print_result(str[i - 1], value);

    }

    return NO_ERROR;
}
String airQuality;


//Sound Detector
String soundLevel;






void setup() 
{

  
  Serial.begin(115200);
  while (!Serial);
  
    //ENV Shield
    if (!ENV.begin()) {
    Serial.println("Failed to initialize!");
    while (1);
  }


  //Temperature Setup
   sensors.begin(); 


  //RTC Setup
    if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }
  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

//SD Setup
if (!SD.begin(0)) {
    Serial.println("sd initialization failed!");
    while (1);
  }
  Serial.println("sd initialization done.");


  //ArduCam Setup
  
  uint8_t vid, pid;
uint8_t temp;
#if defined(__SAM3X8E__)
  Wire1.begin();
#else
  Wire.begin();
#endif
Serial.println(F("ArduCAM Start!"));

pinMode(CS, OUTPUT);
digitalWrite(CS, HIGH);

SPI.begin();
myCAM.write_reg(0x07, 0x80);
delay(100);
myCAM.write_reg(0x07, 0x00);
delay(100); 
  
while(1){
  myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
  temp = myCAM.read_reg(ARDUCHIP_TEST1);
  if(temp != 0x55)
  {
    Serial.println(F("SPI interface Error!"));
    delay(1000);continue;
  }else{
    Serial.println(F("SPI interface OK."));break;
  }
}
#if defined (OV5640_MINI_5MP_PLUS)
while(1){
  //Check if the camera module type is OV5640
  myCAM.rdSensorReg16_8(OV5640_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg16_8(OV5640_CHIPID_LOW, &pid);
  if ((vid != 0x56) || (pid != 0x40)){
    Serial.println(F("Can't find OV5640 module!"));
    delay(1000); continue;
  }else{
    Serial.println(F("OV5640 detected."));break;      
  }
}
#else
while(1){
  //Check if the camera module type is OV5642
  myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
  if ((vid != 0x56) || (pid != 0x42)){
    Serial.println(F("Can't find OV5642 module!"));
    delay(1000);continue;
  }else{
    Serial.println(F("OV5642 detected."));break;      
  }
}
#endif
//Initialize SD Card
while(!SD.begin(SD_CS))
{
  Serial.println(F("SD Card Error!"));delay(1000);
}
Serial.println(F("SD Card detected."));


//Change to JPEG capture mode and initialize the OV5640 module
myCAM.set_format(JPEG);
myCAM.InitCAM();
myCAM.set_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
myCAM.clear_fifo_flag();
myCAM.write_reg(ARDUCHIP_FRAMES, FRAMES_NUM);

myFile = SD.open("info.txt", FILE_WRITE);
  
  if (myFile)
  {
   myFile.print("");
  myFile.print(',');
  myFile.print("DS18B20 Temperature: C ; F");
   myFile.print(',');
  myFile.print("ENV Shield Temperature = C ");
   myFile.print(',');
  myFile.print("Humidity    =  % ");
   myFile.print(',');
  myFile.print("Pressure    = kPa ");
   myFile.print(',');
  myFile.print("Illuminance = lx ");
   myFile.print(',');
  myFile.print("UV index    = ");
   myFile.print(',');
  myFile.print("Sound Status: ");
   myFile.print(',');
  myFile.print("Air Quality: ug/m3 ");
   myFile.print(',');
  myFile.print("PM1.0 Concentration: ug/m3");
   myFile.print(',');
  myFile.print("PM2.5 Concentration: ug/m3");
   myFile.print(',');
  myFile.println("PM10 Concentration: ug/m3");

    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

 
void loop() 
{
  //Real Time Clock

   if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    
  } 

    DateTime now = rtc.now();

    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();

    
  //Temperature Sensor
  sensors.requestTemperatures();
  Serial.print("DS18B20 Temperature: ");
  Serial.print(sensors.getTempCByIndex(0)); 
  Serial.print(" C, ");
  Serial.print(sensors.getTempFByIndex(0));
  Serial.println(" F");
    
  //ENV Shield
  float temperature = ENV.readTemperature();
  float humidity    = ENV.readHumidity();
  float pressure    = ENV.readPressure();
  float illuminance = ENV.readIlluminance();
  Serial.print("ENV Shield Temperature = ");
  Serial.print(temperature);
  Serial.println(" C");
  Serial.print("Humidity    = ");
  Serial.print(humidity);
  Serial.println(" %");
  Serial.print("Pressure    = ");
  Serial.print(pressure);
  Serial.println(" kPa");
  Serial.print("Illuminance = ");
  Serial.print(illuminance);
  Serial.println(" lx");

  
 //UV Sensor
  float sensorVoltage; 
  float sensorValue;
  float uVindex;
  sensorValue = analogRead(A4);
  sensorVoltage = sensorValue/1024*5;
  uVindex = sensorVoltage/0.1;
 // Serial.print("UV sensor reading = ");
 // Serial.print(sensorValue);
 // Serial.println("");
 // Serial.print("UV sensor voltage = ");
 // Serial.print(sensorVoltage);
 // Serial.println(" V");
  Serial.print("UV index    = ");
  Serial.println(uVindex);


  //Sound detector 
   int value;
  value = analogRead(A5);
  Serial.print("Sound Status: ");
  if(value <= 100){
    soundLevel = "Quiet";
  }else if( (value > 100) && (value <= 160) ) {
    soundLevel = "Moderate";
  }else if( (value > 160) && ( value <= 220) ) {
    soundLevel = "Loud";
  }else if(value > 220) {
    soundLevel = "Very Loud";
  }
  Serial.println(soundLevel);

  //Grove Particle Sensor
      if (sensor.read_sensor_value(buf, 29)) {
        SERIAL_OUTPUT.println("HM330X read result failed!!!");
    }
   if ((buf[7]) < 12.0) {
    airQuality = "Good";
   }else if ((buf[7]) < 35.4) {
    airQuality = "Moderate";
   }else if ((buf[7]) < 55.4) {
    airQuality = "Unhealthy for Sensitive Groups";
   }else if ((buf[7]) < 150.4) {
    airQuality = "Unhealthy";
   }else if ((buf[7]) < 250.4) {
    airQuality = "Very Unhealthy";
   }else {
    airQuality = "Hazardous";
   }
    
   Serial.print("Air Quality: ");
   Serial.println(airQuality);
   Serial.print("PM1.0 Concentration: ");
   Serial.print(buf[5]);
   Serial.println(" ug/m3");
   Serial.print("PM2.5 Concentration: ");
   Serial.print(buf[7]);
   Serial.println(" ug/m3");
   Serial.print("PM10 Concentration: ");
   Serial.print(buf[9]);
   Serial.println(" ug/m3");

   //SD Card

   
myFile = SD.open("info.txt", FILE_WRITE);
  
  if (myFile) {
    
    Serial.println("Writing to SD card");
        
    myFile.print(now.year(), DEC);
    myFile.print('/');
    myFile.print(now.month(), DEC);
    myFile.print('/');
    myFile.print(now.day(), DEC);
    myFile.print(" (");
    myFile.print(daysOfTheWeek[now.dayOfTheWeek()]);
    myFile.print(") ");
    myFile.print(now.hour(), DEC);
    myFile.print(':');
    myFile.print(now.minute(), DEC);
    myFile.print(':');
    myFile.print(now.second(), DEC);
    myFile.print(',');
    //  myFile.print("DS18B20 Temperature: ");
  myFile.print(sensors.getTempCByIndex(0)); 
    myFile.print(" ; ");
  myFile.print(sensors.getTempFByIndex(0));
  myFile.print(',');
 // myFile.print("ENV Shield Temperature = ");
  myFile.print(temperature);
  myFile.print(',');
  //myFile.print("Humidity    = ");
  myFile.print(humidity);
  myFile.print(',');
  //myFile.print("Pressure    = ");
  myFile.print(pressure);
  myFile.print(',');
  //myFile.print("Illuminance = ");
  myFile.print(illuminance);
  myFile.print(',');
 //  myFile.print("UV index    = ");
    myFile.print(uVindex);
    myFile.print(',');
 // myFile.print("Sound Status: ");
  myFile.print(soundLevel);
  myFile.print(',');
  // myFile.print("Air Quality: ");
   myFile.print(airQuality);
   myFile.print(',');
   //myFile.print("PM1.0 Concentration: ");
   myFile.print(buf[5]);
   myFile.print(',');
  // myFile.print("PM2.5 Concentration: ");
   myFile.print(buf[7]);
   myFile.print(',');
// myFile.print("PM10 Concentration: ");
   myFile.println(buf[9]);

    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

  
    //ArduCam

    myCAM.flush_fifo();
myCAM.clear_fifo_flag();
#if defined (OV5640_MINI_5MP_PLUS)
  myCAM.OV5640_set_JPEG_size(OV5640_320x240);delay(1000);
#else
  myCAM.OV5642_set_JPEG_size(OV5642_320x240);delay(1000);
#endif
//Start capture
myCAM.start_capture();
Serial.println(F("start capture."));
total_time = millis();
while ( !myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)); 
Serial.println(F("CAM Capture Done."));
total_time = millis() - total_time;
Serial.print(F("capture total_time used (in miliseconds):"));
Serial.println(total_time, DEC);
total_time = millis();
read_fifo_burst(myCAM);
total_time = millis() - total_time;
Serial.print(F("save capture total_time used (in miliseconds):"));
Serial.println(total_time, DEC);
//Clear the capture done flag
myCAM.clear_fifo_flag();

Serial.println();
//Line for organization
Serial.println("------------------------------------------------------------------------");
delay(20000);
}
uint8_t read_fifo_burst(ArduCAM myCAM)
{
uint8_t temp = 0, temp_last = 0;
uint32_t length = 0;
static int i = 0;
static int k = 0;
char str[8];
File outFile;
byte buf[256]; 
length = myCAM.read_fifo_length();
Serial.print(F("The fifo length is :"));
Serial.println(length, DEC);
if (length >= MAX_FIFO_SIZE) //8M
{
  Serial.println("Over size.");
  return 0;
}
if (length == 0 ) //0 kb
{
  Serial.println(F("Size is 0."));
  return 0;
} 
myCAM.CS_LOW();
myCAM.set_fifo_burst();//Set fifo burst mode
i = 0;
while ( length-- )
{
  temp_last = temp;
  temp =  SPI.transfer(0x00);
  //Read JPEG data from FIFO
  if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
  {
    buf[i++] = temp;  //save the last  0XD9     
    //Write the remain bytes in the buffer
    myCAM.CS_HIGH();
    outFile.write(buf, i);    
    //Close the file
    outFile.close();
    Serial.println(F("OK"));
    is_header = false;
    myCAM.CS_LOW();
    myCAM.set_fifo_burst();
    i = 0;
  }  
  if (is_header == true)
  { 
    //Write image data to buffer if not full
    if (i < 256)
     buf[i++] = temp;
    else
    {
      //Write 256 bytes image data to file
      myCAM.CS_HIGH();
      outFile.write(buf, 256);
      i = 0;
      buf[i++] = temp;
      myCAM.CS_LOW();
      myCAM.set_fifo_burst();
    }        
  }
  else if ((temp == 0xD8) & (temp_last == 0xFF))
  {
    is_header = true;
    myCAM.CS_HIGH();
    //Create a avi file
    k = k + 1;
    itoa(k, str, 10);
    strcat(str, ".jpg");
    //Open the new file
    outFile = SD.open(str, O_WRITE | O_CREAT | O_TRUNC);
    if (! outFile)
    {
      Serial.println(F("File open failed"));
      while (1);
    }
    myCAM.CS_LOW();
    myCAM.set_fifo_burst();   
    buf[i++] = temp_last;
    buf[i++] = temp;   
  }
}
  myCAM.CS_HIGH();
  return 1;
}

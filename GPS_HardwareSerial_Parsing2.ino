// Test code for Ultimate GPS Using Hardware Serial (e.g. GPS Flora or FeatherWing)
//
// This code shows how to listen to the GPS module via polling. Best used with
// Feathers or Flora where you have hardware Serial and no interrupt
//
// Tested and works great with the Adafruit GPS FeatherWing
// ------> https://www.adafruit.com/products/3133
// or Flora GPS
// ------> https://www.adafruit.com/products/1059
// but also works with the shield, breakout
// ------> https://www.adafruit.com/products/1272
// ------> https://www.adafruit.com/products/746
//
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

#include <Adafruit_GPS.h>
#include <SD.h>
#include <math.h>


#include "Wire.h" // This library allows you to communicate with I2C devices.

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data

// what's the name of the hardware serial port?
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();
const int chipSelect = 4;
//double lat1 = 22.9846;
//double long1 = -72.7319;
double lat1 = 0.000;
double long1 = 0.000;




void setup()
{
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

//  connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  //Serial.begin(115200);
  //Serial.println("Adafruit GPS library basic parsing test!");
  
  if (!SD.begin(chipSelect)) {
    //Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  //Serial.println("card initialized.");

  // ------ line on intialization
  
  File dataFile = SD.open("GPS2.csv", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println("----------");
    dataFile.println("Hour, Minute, Second ,Day,Month,Year, LAT, LONG, SPEED, Distance");
    dataFile.close();
}
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // Ask for firmware version
  //GPSSerial.println(PMTK_Q_RELEASE);
}

  double distance_in_km(double  lat1, double lon1, double lat2, double lon2) {
    
    
        //for latitude1
        double w1 = (lat1/100);
        int d1 =  w1;
        double s1 = w1 - d1;
        double o1 = (s1 / 60) * 100;
        double sum_lat =  d1+o1;
      //for longitude2
        double wlo1 = (lon1 / 100);
        int dlo1=  wlo1;
        double slo1 = wlo1-dlo1;
        double olo1 = (slo1 / 60) * 100;
        double sum_lon = dlo1 + olo1;
    
    //for latitude
        double w = (lat2/100);
        int d =  w;
        double s = w - d;
        double o = (s / 60) * 100;
        double sum1 =  d+o;
      //for longitude
        double wl = (lon2 / 100);
        int dl =  wl;
        double sl = wl-dl;
        double ol = (sl / 60) * 100;
        double sum2 = dl + ol;
      
        double lat_d1 = sum_lat * (PI /180);
        double lat_d2 = sum1 * (PI/180);
        double lon_d1 = -sum_lon * (PI /180);
        double lon_d2 = -sum2 * (PI/180);
        double dlon = lon_d2 -lon_d1;
        double dlat = lat_d2-lat_d1;
        double dlat_h = dlat/2;
        double dlon_h = dlon/2;
        double  a = (sin(dlat_h) *sin(dlat_h)) +(cos(lat_d1) * cos(lat_d2) *(sin(dlon_h)*sin(dlon_h)));
        double c = 2 * asin(sqrt(a));
        double r = 6371;
        double dIS = c* r;
        return double (dIS);
        

      }
void loop() // run over and over again
{
  String dataString = " ";
  
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) ;//Serial.print(c)
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
 
  // approximately every 2 seconds or so, print out the current stats 2000
  if (millis() - timer > 1000)  {
    timer = millis(); // reset the timer
    //Serial.print("\nTime: ");
    if (GPS.hour < 10)// { Serial.print('0'); }
    //Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10)// { Serial.print('0'); }
    //Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) //{ Serial.print('0'); }
    //Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      //Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      //Serial.print("0");
    }
    //Serial.println(GPS.milliseconds);
    //Serial.print("Date: ");
    //Serial.print(GPS.day, DEC); Serial.print('/');
    //Serial.print(GPS.month, DEC); Serial.print("/20");
    //Serial.println(GPS.year, DEC);
    //Serial.print("Fix: "); Serial.print((int)GPS.fix);
    //Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      //Serial.print("Location: ");
      //Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      //Serial.print(", ");
     // Serial.print(-GPS.longitude, 4); Serial.println(GPS.lon);
     // Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      //Serial.print("Angle: "); Serial.println(GPS.angle);
      //Serial.print("Altitude: "); Serial.println(GPS.altitude);
      //Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    
    }
    double lat2 = GPS.latitude;
    double long2 = GPS.longitude;
    double Distance = (distance_in_km(lat1, long1 ,lat2, long2));
    lat1 = lat2;
    long1 = long2;
    //Serial.println(Distance,10);
    //Serial.println(lat1);
    //Serial.println(long1);
    //Serial.println(lat2);
    //Serial.println(long2);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  
  float acc_x  = (float)(accelerometer_x)/16324.0;
  float acc_y  = (float)(accelerometer_y)/16324.0;
  float acc_z  = (float)(accelerometer_z)/16324.0;
  //Serial.println(acc_x);
  //Serial.println(acc_y);
  //Serial.println(acc_z);


  dataString += "";
  dataString += String(GPS.hour, DEC);
  dataString += ",";
  dataString += String(GPS.minute,DEC);
  dataString += ",";
  dataString += String(GPS.seconds,DEC);
  dataString += ",";
  dataString += String(GPS.day, DEC);
  dataString += ",";
  dataString += String(GPS.month, DEC);
  dataString += ",";
  dataString += String(GPS.year, DEC);
  dataString += ",";
  dataString += String(GPS.latitude,4);
  dataString += ",";
  dataString += String(GPS.longitude,4);
  dataString += ",";
  dataString += String((GPS.speed)*1.852);
  dataString += ",";
  dataString += String(Distance,6);
  dataString += ",";
  dataString += String(acc_x,6);
  dataString += ",";
  dataString += String(acc_y,6);
  dataString += ",";
  dataString += String(acc_z,6);


 
  
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("GPS2.csv", FILE_WRITE);
  //starttime = millis();

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    // Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    //Serial.println("error opening datalog.txt");
  }
  }
}

/*------------------------------------------------------------------------------

  LIDARLite Arduino Library
  GetDistanceI2c

  This example shows how to initialize, configure, and read distance from a
  LIDAR-Lite connected over the I2C interface.

  Connections:
  LIDAR-Lite 5 Vdc (red) to Arduino 5v
  LIDAR-Lite I2C SCL (green) to Arduino SCL
  LIDAR-Lite I2C SDA (blue) to Arduino SDA
  LIDAR-Lite Ground (black) to Arduino GND
  
  (Capacitor recommended to mitigate inrush current when device is enabled)
  680uF capacitor (+) to Arduino 5v
  680uF capacitor (-) to Arduino GND

  See the Operation Manual for wiring diagrams and more information:
  http://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf

------------------------------------------------------------------------------*/
#include <Wire.h>
#include <LIDARLite.h>
#include "PX4Flow.h"
#define SCALER 1
LIDARLite myLidarLite;
PX4Flow sensor = PX4Flow(); 
int count=0;
int count1=0;
float flow_x;
float flow_y;
long timespan;
float gddistance;
float gddistance1;
int quality;
float px=0;
float py=0;
long times=0;
float flowArray[8];
float movingSum=0;
float pixel_x;
  float pixel_y;
  float velocity_x;
  float velocity_y;

void setup(){
  Wire.begin();      
  Serial.begin(115200); // Initialize serial connection to display distance readings
  myLidarLite.begin(4, true); // Set configuration to default and I2C to 400 kHz

  /*
    configure(int configuration, char lidarliteAddress)

    Selects one of several preset configurations.

    Parameters
    ----------------------------------------------------------------------------
    configuration:  Default 0.
      0: Default mode, balanced performance.
      1: Short range, high speed. Uses 0x1d maximum acquisition count.
      2: Default range, higher speed short range. Turns on quick termination
          detection for faster measurements at short range (with decreased
          accuracy)
      3: Maximum range. Uses 0xff maximum acquisition count.
      4: High sensitivity detection. Overrides default valid measurement detection
          algorithm, and uses a threshold value for high sensitivity and noise.
      5: Low sensitivity detection. Overrides default valid measurement detection
          algorithm, and uses a threshold value for low sensitivity and noise.
    lidarliteAddress: Default 0x62. Fill in new address here if changed. See
      operating manual for instructions.
  */
  myLidarLite.configure(4); // Change this number to try out alternate configurations
}

void loop(){
 //times=millis();
 ReadPx4flow();
 //delay(200);
 ReadLidarLite();
 distanceMeasure();
 delay(100);
}
float movingAverage(float Sum,float Array[8],float newflow){
  Sum=Sum+newflow-flowArray[1];
  for (int i=0;i<=6;i++){
    Array[i]=Array[i+1];
  }
  Array[7]=newflow;
  movingSum=Sum;
  *flowArray=*Array;
  return movingSum/8; 
}
void distanceMeasure(){
  
  if (quality > 100)
    {
      count=count+1;
      // Update flow rate with gyro rate
      pixel_x = flow_x;  // mrad
      //pixel_y = flow_y; // mrad
      if (count<=8){
        flowArray[count-1]=flow_y;
        movingSum=movingSum+flow_y;
        pixel_y=movingSum*SCALER/count;
      }
      else{
        pixel_y=movingAverage(movingSum,flowArray,flow_y)*SCALER;
      }
      
      // Scale based on ground distance and compute speed
      // (flow/1000) * (ground_distance/1000) / (timespan/1000000)
      velocity_x = pixel_x * gddistance1;     // m/s
      velocity_y = pixel_y * gddistance1 ;     // m/s 
      
      // Integrate velocity to get pose estimate
      px = px + velocity_x/1000000;
      py = py + velocity_y/1000000;
          
      // Output some data
      
    }
  //Serial.print(millis());Serial.print(","); Serial.print("\t"); 
      Serial.print(py);Serial.print("\t");
      Serial.print(px);Serial.print("\t"); 
      Serial.print(pixel_y);Serial.print("\t");   
      Serial.print(flow_y);Serial.print("\t"); 
      Serial.print(SCALER);Serial.print("\t"); 
       Serial.print(quality);Serial.print("\t");
      Serial.println(gddistance1);
      //Serial.println(flowArray);
    /*  for (int t=0;t<=7;t++){
         Serial.print(flowArray[t]);Serial.print("\t");
      }
      Serial.println();
      Serial.println(movingSum);*/
}
void ReadPx4flow(){
  sensor.update_integral();
  flow_x = sensor.pixel_flow_x_integral() / 10.0f;      // mrad
  flow_y = sensor.pixel_flow_y_integral() / 10.0f;      // mrad  
  timespan = sensor.integration_timespan();               // microseconds
  gddistance1 = sensor.ground_distance_integral();    // mm
  quality = sensor.quality_integral();
  //Serial.print(flow_x); Serial.print("\t");
  //Serial.println(timespan);
  //Serial.print("PX4flow GD:  ");
  //Serial.print("PX4flow GD:  ");
  //Serial.print("PX4flow GD:  ");
  //Serial.print(sensor.flow_comp_m_x());Serial.print(",");
  //Serial.print(sensor.flow_comp_m_y());Serial.print(",");
  //Serial.print(sensor.ground_distance()/10);
  //Serial.print("\t");
   //delay(100);

}
void ReadLidarLite(){
   /*
    distance(bool biasCorrection, char lidarliteAddress)

    Take a distance measurement and read the result.

    Parameters
    ----------------------------------------------------------------------------
    biasCorrection: Default true. Take aquisition with receiver bias
      correction. If set to false measurements will be faster. Receiver bias
      correction must be performed periodically. (e.g. 1 out of every 100
      readings).
    lidarliteAddress: Default 0x62. Fill in new address here if changed. See
      operating manual for instructions.
  */

  // Take a measurement with receiver bias correction and print to serial terminal
  //Serial.print("LidarLite GD:  ");
  if (count1==0){
    //Serial.println(myLidarLite.distance());
    gddistance=(myLidarLite.distance()*10);
    count1=count1+1;
  }
  else{
    count1=count1+1;
    //Serial.println(myLidarLite.distance(false));
    gddistance=(myLidarLite.distance()*10);
  }
  if (count1==100){
    count1=0;
  }
  

  // Take 99 measurements without receiver bias correction and print to serial terminal
 /* for(int i = 0; i < 99; i++)
  {
    
  }*/
}


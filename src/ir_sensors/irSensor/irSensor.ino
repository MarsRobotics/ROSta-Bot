/* 
 * rosserial IR Ranger Example  
 * 
 * NOW DEPENDS ON the SharpIR library:
 * http://playground.arduino.cc/Main/SharpIR 
 */

#include <ros.h>
#include <std_msgs/Int32.h>
#include <SharpIR.h>

ros::NodeHandle  nh;

std_msgs::Int32 irReading;

// topic name is range_data
ros::Publisher irSensor( "range_data", &irReading);

int pin = 0;
int val = 0;
// pin, # of values to average, %change required for new value,
SharpIR sharp(pin,5,95,20150);

void setup()
{
  //Serial.begin(9600);
  nh.initNode();
  nh.advertise(irSensor);
}

void loop()
{
  // publish the range value 
  irReading.data = sharp.distance();
  irSensor.publish(&irReading);
  //Serial.println(val);
  nh.spinOnce();
  delay(5000);
}


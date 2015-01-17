/* 
 * rosserial IR Ranger Example  
 * 
 */

#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle  nh;

std_msgs::Int32 irReading;

// topic name is range_data
ros::Publisher irSensor( "range_data", &irReading);

int pin = 0;
int val = 0;

void setup()
{
  //Serial.begin(9600);
  nh.initNode();
  nh.advertise(irSensor);
}

void loop()
{
  // publish the range value 
  val = analogRead(pin);
  irReading.data = val;
  irSensor.publish(&irReading);
  //Serial.println(val);
  nh.spinOnce();
  delay(5000);
}


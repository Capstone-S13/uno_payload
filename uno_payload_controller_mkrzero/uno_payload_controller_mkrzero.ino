/*
 * rosserial Subscriber Example using TCP on Arduino Shield (Wiznet W5100 based)
 * Blinks an LED on callback
 */
#include <SPI.h>
#include <Ethernet.h>

#define ROSSERIAL_ARDUINO_TCP

#include <Servo.h>
#include <ros.h>
#include <std_msgs/Uint32.h>

#define servo_pin 4

Servo myservo;
int pos = 0;

const int PUSHER_OUT = 0;
const int PUSHER_IN = 1;
const int PUSHER_IDLE = 2;

ros::NodeHandle  nh;

// Shield settings
byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0x28, 0xCE };
IPAddress ip(192, 168, 0, 177);

// Server settings
IPAddress server(192, 168, 0, 10);
uint16_t serverPort = 11411;

std_msgs::UInt32 status_msg;
ros::Publisher pub_status( "/uno_payload/status", &status_msg);

void messageCb(const std_msgs::UInt32& cmd_msg){
  digitalWrite(LED_BUILTIN, HIGH);
  if (status_msg.data == cmd_msg.data){
    // already in correct state
    delay(15);
  }
  else if (cmd_msg.data == PUSHER_OUT){
    for (pos = 0; pos <= 180; pos += 3){
    myservo.write(pos);
    delay(5);
    }
  }

  else if (cmd_msg.data == PUSHER_IN){
    for (pos = 180; pos >= 0; pos -= 3){
    myservo.write(pos);
    delay(15);
    }
  }
  status_msg.data = cmd_msg.data;
  for (int i = 0; i<=3; i +=1){
    pub_status.publish(&status_msg);
  }
  digitalWrite(LED_BUILTIN, LOW);
}

ros::Subscriber<std_msgs::UInt32> sub_cmd("/uno_payload/cmd", messageCb );

void setup()
{
  Ethernet.begin(mac, ip);
  // give the Ethernet shield a second to initialize:
  delay(1000);

  myservo.attach(servo_pin);
  pinMode(LED_BUILTIN, OUTPUT);
  myservo.write(0);
  delay(500);
  
  nh.getHardware()->setConnection(server, serverPort);
  
  nh.initNode();
  nh.subscribe(sub_cmd);
  nh.advertise(pub_status);

  status_msg.data = PUSHER_IDLE;
  pub_status.publish(&status_msg);
}

void loop()
{
  nh.spinOnce();

  delay(50);
}

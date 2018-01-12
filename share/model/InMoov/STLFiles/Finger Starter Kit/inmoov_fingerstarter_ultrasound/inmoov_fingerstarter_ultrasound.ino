//Little demo of Inmoov finger starter of Gael Langevin http://www.thingiverse.com/thing:67709
//using an ultrasound sensor. Youtube demo http://www.youtube.com/watch?v=ewc-a3LLIs0
//by @hugobiwan / labfab.fr

//You should use ping library for sr04 ultrasound sensor- first unzip in arduino/libraries.
//infos and download here : http://code.google.com/p/arduino-new-ping/

#include <NewPing.h>

// pins for ultrasonic sensors

#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.

//here we will use pin 6 as 5V (see below) and a GND pin anywhere on the arduino.

#define MAX_DISTANCE 30 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
int distance;

//servo library
#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
 
int val;    // variable to read the value from the analog pin 
 
void setup() 
{ 
  myservo.attach(7);  // attaches the servo on pin 9 to the servo object 
  Serial.begin(9600);
  //we use the pin 6 as a 5V pin (just to make it easyer without breadboard).
  pinMode(6,OUTPUT);
  digitalWrite(6,HIGH);
} 
 
void loop() 
{ 

  
  //ping
  
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  Serial.print("Ping: ");
  distance= uS / US_ROUNDTRIP_CM;
  Serial.print(uS / US_ROUNDTRIP_CM); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");
  
  //action of the finger
  if(distance>0 && distance <MAX_DISTANCE)
  {
  kikoo();
  }
  
} 
//action of the finger : the last point is without tension (90 degrees).
void kikoo()
{
  myservo.write(0);
  delay(1500);
  myservo.write(179);
  delay(1500);
  myservo.write(90);                  
  distance=0;
}
  

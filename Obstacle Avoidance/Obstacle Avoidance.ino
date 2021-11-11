// 
// Robot Control Program
// Amy Everitt
// 10/09/2021
//
#include <Wire.h> // support library for talking to PCF8574 chip

// define hardware addresses for easy reference
#define LM_PWMA   6           // Left Motor Speed pin (ENA)
#define LM_AIN2   A0          // Motor-L forward (IN2).
#define LM_AIN1   A1          // Motor-L backward (IN1)
#define RM_PWMB   5           // Right Motor Speed pin (ENB)
#define RM_BIN1   A2          // Motor-R forward (IN3)
#define RM_BIN2   A3          // Motor-R backward (IN4)
#define ULTRA_ECHO   2        // Ultrasound echo pin
#define ULTRA_TRIG   3        // Ultrasound trigger pin

// Define address of PCF8574 i2c io expander
#define PCF8574_ADDR  0x20

#define SPD_FAST 60
#define SPD_NORMAL 40
#define SPD_SLOW 30
#define SPD_STOP 0

// define distances for which to modify behaviour
#define DIST_CLOSE 20
#define DIST_AVOID 10

// define global variables
int g_distance = 0;
int g_speed = SPD_NORMAL;

// define internal functions
void forward();
void backward();
void right();
void left();
void stop();
byte read_ir_collision();
int ultra_distance_test();
void PCF8574Write(byte data);
byte PCF8574Read();

// Main program
void setup() {
  // Start serial interface for debug messages and send initial welcome message
  Serial.begin(115200);  
  Serial.println("Robot Control Program V2.0");

  // Start the wire library
  Wire.begin();

  // Configure ultrasound detector
  pinMode(ULTRA_ECHO, INPUT);    // Define the ultrasonic echo input pin
  pinMode(ULTRA_TRIG, OUTPUT);   // Define the ultrasonic trigger input pin 

  // Configure left and right motor controllers
  pinMode(LM_PWMA,OUTPUT);                     
  pinMode(LM_AIN2,OUTPUT);      
  pinMode(LM_AIN1,OUTPUT);
  pinMode(RM_PWMB,OUTPUT);       
  pinMode(RM_BIN1,OUTPUT);     
  pinMode(RM_BIN2,OUTPUT); 

  // Start motors and then bring to a stop until first distance measured
  analogWrite(LM_PWMA,g_speed);
  analogWrite(RM_PWMB,g_speed);
  stop();    
}

void loop() {

  forward();
  
  // retrieve distance to obstacle
  g_distance = ultra_distance_test();  

  // display distance on serial port
  Serial.print("US Distance = ");            
  Serial.print(g_distance);       
  Serial.println("cm"); 

  // check status of ir collision detection sensors
  byte ir_status = read_ir_collision();

  // display ir detection sensor status
  Serial.print("IR Status = ");
  Serial.println(ir_status, HEX);


// if obstacle within close distance, slow down first
    if (g_distance < DIST_CLOSE)      
    {
      Serial.println("Slowing down ..."); 
    
      g_speed = SPD_SLOW;
      forward();    
    }
 
      if (ir_status == 0x7F)
    {
      Serial.println("Left IR sensor on");
      right();
      delay(500);
      stop();
    }
  
    else if (ir_status == 0xBF)
    {
      Serial.println("Right IR sensor on");
      left();
      delay(750);
      stop();
    }
  
    else if (ir_status == 0x3F)
    {
      Serial.println("Both IR sensors on");
      reverse();
      delay(500);
      right();
      delay(500);
      stop();
    }
  
  
   
   //nothing to worry about so carry on
  else
  {
    g_speed = SPD_NORMAL;
    forward();    
  }

   //move in appropriate way for 250ms
  delay(250);
}

//Supplementary functions

//move robot forward
void forward()
{
  analogWrite(LM_PWMA,g_speed);
  analogWrite(RM_PWMB,g_speed);
  digitalWrite(LM_AIN1,LOW);
  digitalWrite(LM_AIN2,HIGH);
  digitalWrite(RM_BIN1,LOW);  
  digitalWrite(RM_BIN2,HIGH); 
}

// reverse robot slowly
void reverse()
{
  analogWrite(LM_PWMA,SPD_SLOW);
  analogWrite(RM_PWMB,SPD_SLOW);
  digitalWrite(LM_AIN1,HIGH);
  digitalWrite(LM_AIN2,LOW);
  digitalWrite(RM_BIN1,HIGH); 
  digitalWrite(RM_BIN2,LOW);  
}

// turn robot right slowly
void right()
{
  analogWrite(LM_PWMA,SPD_SLOW);
  analogWrite(RM_PWMB,SPD_SLOW);
  digitalWrite(LM_AIN1,LOW);
  digitalWrite(LM_AIN2,HIGH);
  digitalWrite(RM_BIN1,HIGH); 
  digitalWrite(RM_BIN2,LOW);  
}

// turn robot left slowly
void left()
{
  analogWrite(LM_PWMA,SPD_SLOW);
  analogWrite(RM_PWMB,SPD_SLOW);
  digitalWrite(LM_AIN1,HIGH);
  digitalWrite(LM_AIN2,LOW);
  digitalWrite(RM_BIN1,LOW); 
  digitalWrite(RM_BIN2,HIGH);  
}

// stop robot
void stop()
{
  analogWrite(LM_PWMA,SPD_STOP);
  analogWrite(RM_PWMB,SPD_STOP);
  digitalWrite(LM_AIN1,LOW);
  digitalWrite(LM_AIN2,LOW);
  digitalWrite(RM_BIN1,LOW); 
  digitalWrite(RM_BIN2,LOW);  
}

// check status of IR Collision Detection Sensors
byte read_ir_collision()
{
  PCF8574Write(0xC0 | PCF8574Read());   //set Pin High
  return PCF8574Read() | 0x3F;          //read Pin
}

// measure distance to obstacles using ultrasound
// code copied from sample code library
// ultrasonic range ranging 2cm to 400cm
int ultra_distance_test()          
{
  digitalWrite(ULTRA_TRIG, LOW);   // set trig pin low 2μs
  delayMicroseconds(2);
  digitalWrite(ULTRA_TRIG, HIGH);  // set trig pin 10μs , at last 10us 
  delayMicroseconds(10);
  digitalWrite(ULTRA_TRIG, LOW);    // set trig pin low
  float Fdistance = pulseIn(ULTRA_ECHO, HIGH);  // Read echo pin high level time(us)
  Fdistance= Fdistance/58;       //Y m=（X s*344）/2
  // X s=（ 2*Y m）/344 ==》X s=0.0058*Y m ==》cm = us /58       
  return (int)Fdistance;
}  

// retrieve information from onboard IR collision detection sensors
// code copied from sample code library
void PCF8574Write(byte data)
{
  Wire.beginTransmission(PCF8574_ADDR);
  Wire.write(data);
  Wire.endTransmission(); 
}

byte PCF8574Read()
{
  int data = -1;
  Wire.requestFrom(PCF8574_ADDR, 1);
  if(Wire.available()) {
    data = Wire.read();
  }
  return data;
}

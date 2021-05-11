//Arduino UNO and ultrasonic sensor obstacle avoidance 

//motor
#define Lpwm_pin  5     //pin of controlling speed---- ENA of motor driver board
#define Rpwm_pin  6    //pin of controlling speed---- ENB of motor driver board
int pinLB = 2;             //pin of controlling turning---- IN1 of motor driver board
int pinLF = 4;             //pin of controlling turning---- IN2 of motor driver board
int pinRB = 7;            //pin of controlling turning---- IN3 of motor driver board
int pinRF = 11;            //pin of controlling turning---- IN4 of motor driver board


//ir sensor
#include <Servo.h>
volatile int left_IR_Sensor;
volatile int right_IR_Sensor;
int pos = 0; 
Servo myservo; 

//ultrasound 

long a;

float checkdistance() {
  digitalWrite(A1, LOW);
  delayMicroseconds(2);
  digitalWrite(A1, HIGH);
  delayMicroseconds(10);
  digitalWrite(A1, LOW);
  float distance = pulseIn(A0, HIGH) / 58.00;
  //delay(10);
  return distance;
}


//--------------------------SETUP-------------------------------

void setup()
{
  //motor
  pinMode(2,OUTPUT); // /pin 2
  pinMode(4,OUTPUT); // pin 4
  pinMode(7,OUTPUT); // pin 7
  pinMode(11,OUTPUT);  // pin 11
  pinMode(5,OUTPUT);  // pin 5 (PWM) 
  pinMode(6,OUTPUT);  // pin6(PWM) 
  Serial.begin(115200);

  //ir sensor
  myservo.attach(A2);
  pinMode(9, INPUT);
  pinMode(10, INPUT);

  //ultrasound
   pinMode(A1, OUTPUT);
   pinMode(A0, INPUT);
   delay(1000);
}


//--------------------------LOOP-------------------------------
void loop()
{
  

//------------------ultrasound-------------------
a=checkdistance();
   Serial.print(a);
   Serial.println("cm");


if (a < 100) {
 stopp();
  delay(2000);

Set_Speed(200); //set speed changes pwm must be above 150
 turnR();
   delay(300);

}//if
  
  else {
 
Set_Speed(200);
  advance();
  delay(2000);
  
  }//else
      
  }//loop
  






//-------------------------functions--------------------------------

void Set_Speed(unsigned char pwm) //function of setting speed
{
  analogWrite(Lpwm_pin,pwm);
  analogWrite(Rpwm_pin,pwm);
}

void advance()    //  going forward
    {
     Serial.println("advancing");
     digitalWrite(7,LOW);  // making motor move towards right rear
     digitalWrite(11,HIGH);
     digitalWrite(2,LOW);  // making motor move towards left rear
     digitalWrite(4,HIGH); 
   
    }
void turnR()        //turning right(dual wheel)
    {
     Serial.println("turning right");
     digitalWrite(7,LOW);  //making motor move towards right rear
     digitalWrite(11,HIGH);
     digitalWrite(2,HIGH);
     digitalWrite(4,LOW);  //making motor move towards left front
  
    }
void turnL()         //turning left(dual wheel)
    {
     Serial.println("turning left");
     digitalWrite(7,HIGH);
     digitalWrite(11,LOW );   //making motor move towards right front
     digitalWrite(2,LOW);   //making motor move towards left rear
     digitalWrite(4,HIGH);
    
    }    
void stopp()        //stop
    {
     Serial.println("stoping");
     digitalWrite(7,HIGH);
     digitalWrite(11,HIGH);
     digitalWrite(2,HIGH);
     digitalWrite(4,HIGH);
    
    }
void back()         //back up
    {
     Serial.println("backing up");
     digitalWrite(7,HIGH);  //making motor move towards right rear     
     digitalWrite(11,LOW);
     digitalWrite(2,HIGH);  //making motor move towards left rear
     digitalWrite(4,LOW);
      
    }



  

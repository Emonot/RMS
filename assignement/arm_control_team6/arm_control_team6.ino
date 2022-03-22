#include <Servo.h>

//defining pin for the servo motor
#define SERVO1 9
#define SERVO2 10
#define SERVO3 11
#define SERVO4 12

//defining pin for the joysticks
#define JOY1 14
#define JOY2 15
#define JOY3 16
#define JOY4 17

//declare pin for the button
#define INPUT1 4

//declare all of our motor
Servo motor1; //base motor to turn the arm around the Z axis
Servo motor2; //first arm to control a theta angle
Servo motor3; //control the second joint
Servo motor4; //control the grab

int btnMode; // declare the button for the manual or automatic mode

//declare our potentiometer
int joy1;
int joy2;
int joy3;
int joy4;

void setup() {
  //put pwm option for each of those pin
  Serial.begin(9600);
  
  motor1.attach(SERVO1);
  motor2.attach(SERVO2);
  motor3.attach(SERVO3);
  motor4.attach(SERVO4);
  
  //initialise the button as input
  pinMode(INPUT1, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  btnMode = digitalRead(INPUT1);

  delay(300);
    joy1 = analogRead(JOY1);            // reads the value of the potentiometer (value between 0 and 1023)
    joy1 = map(joy1, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    joy2 = analogRead(JOY2);            // reads the value of the potentiometer (value between 0 and 1023)
    joy2 = map(joy2, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    joy3 = analogRead(JOY3);            // reads the value of the potentiometer (value between 0 and 1023)
    joy3 = map(joy3, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    joy4 = analogRead(JOY4);            // reads the value of the potentiometer (value between 0 and 1023)
    joy4 = map(joy4, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    
    motor1.write(joy1);                  // sets the servo position according to the scaled value
    motor2.write(joy2);
    motor3.write(joy3);
    motor4.write(joy4);
    
    //Serial.print("joy 1 :");
    //Serial.println(joy1);
    Serial.print("joy 2 :");
    Serial.println(joy2);/*
    Serial.print("joy 3 :");
    Serial.println(joy3);
    Serial.print("joy 4 :");
    Serial.println(joy4);
    */
    
    delay(10); //wait for the servo to reach the position    
  if(btnMode == LOW)
  {
    //launch automatic mode
    int anything = 0;
    motor1.write(anything);                  // WE HAVE TO FIND A SOLUTION FOR THE CONTINUOUS SERVO
    delay(1);
    motor2.write(anything);
    delay(1);
    motor3.write(anything);
    delay(1);
    motor4.write(anything);

    delay(1);

    motor1.write(anything);                  // WE HAVE TO FIND A SOLUTION FOR THE CONTINUOUS SERVO
    delay(1);
    motor2.write(anything);
    delay(1);
    motor3.write(anything);
    delay(1);
    motor4.write(anything);
    delay(1);
  }
  
  else
  {
    //launch manual mode
    delay(300);
    joy1 = analogRead(JOY1);            // reads the value of the potentiometer (value between 0 and 1023)
    joy1 = map(joy1, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    joy2 = analogRead(JOY2);            // reads the value of the potentiometer (value between 0 and 1023)
    joy2 = map(joy2, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    joy3 = analogRead(JOY3);            // reads the value of the potentiometer (value between 0 and 1023)
    joy3 = map(joy3, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    joy4 = analogRead(JOY4);            // reads the value of the potentiometer (value between 0 and 1023)
    joy4 = map(joy4, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    
    motor1.write(joy1);                  // sets the servo position according to the scaled value
    motor2.write(joy2);
    motor3.write(joy3);
    motor4.write(joy4);
    
    //Serial.print("joy 1 :");
    //Serial.println(joy1);
    Serial.print("joy 2 :");
    Serial.println(joy2);/*
    Serial.print("joy 3 :");
    Serial.println(joy3);
    Serial.print("joy 4 :");
    Serial.println(joy4);
    */
    
    delay(10); //wait for the servo to reach the position                           
  }
}

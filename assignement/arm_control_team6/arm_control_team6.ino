#include <Servo.h>
#define _USE_MATH_DEFINES // for C
#include <math.h>

//definning pin for stepper motor
#define STEPPER_A1
#define STEPPER_A2
#define STEPPER_B1
#define STEPPER_B2

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

//Declare goals for end effector
float gx;
float gy;
float gz;
float gfi;

//Declare joint angle goals
float g_theta1;
float g_theta2;
float g_theta3;
float g_theta4;

//Declare current joint angles
float theta1;
float theta2;
float theta3;
float theta4;
float theta234;

//Declare lengths of arm limbs
float L2;
float L3;
float L4;

//Other variables
bool goal_reached;
float p1;
float p2;
float c3;



void setup() {
  //put pwm option for each of those pin
  Serial.begin(9600);

  //Initialise limb lengths (THESE WILL NEED CORRECTION / CALIBRATION)
  L2 = 10;
  L3 = 5;
  L4 = 5;
  goal_reached = false;

  //Initialise goal positions
  gx = 15;
  gy = 0;
  gz = 0;
  gfi = 0;
  
  // Initialise joint positions of arm
  g_theta1 = 90;
  g_theta2 = 90;
  g_theta3 = 0;
  g_theta4 = 90;

  theta1 = 90;
  theta2 = 90;
  theta3 = 0;
  theta4 = 90;
  
  motor1.attach(SERVO1);
  motor2.attach(SERVO2);
  motor3.attach(SERVO3);
  motor4.attach(SERVO4);
  
  //initialise the button as input
  pinMode(INPUT1, INPUT);

  while (goal_reached == false){
    calc_motion();
    move_joints();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  check_button();
/*  btnMode = digitalRead(INPUT1);

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
  if(auto_switch == true)
  {
    //launch automatic mode
    if (goal_reached == false){
      calc_motion();
      move_joints();     
    }
    else{
      // Need a way to read input location and effector angle (x,y,z,fi)
      calc_joint_angle_goals();
    }
  }
  
  else
  {
    //launch manual mode
    if (goal_reached == true){
      read_joy();
      set_goals();
    }
    else {
      
    }
    
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

void read_joy(){
  joy1 = analogRead(JOY1);            // reads the value of the potentiometer (value between 0 and 1023)
  joy1 = map(joy1, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  joy2 = analogRead(JOY2);            // reads the value of the potentiometer (value between 0 and 1023)
  joy2 = map(joy2, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  joy3 = analogRead(JOY3);            // reads the value of the potentiometer (value between 0 and 1023)
  joy3 = map(joy3, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  joy4 = analogRead(JOY4);            // reads the value of the potentiometer (value between 0 and 1023)
  joy4 = map(joy4, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
}

void set_goals(){
  
}

void check_button(){
  buttonState = digitalRead(BUTT_1);
  
  // Check to see if the button has changed to HIGH from LOW
  if ((buttonState == HIGH) && (lastButt == LOW)) {
    if (auto_switch == false){
      auto_switch = true;
    }
    else {
      auto_switch = false;
    }
  }
  // Record buttonState for next cycle
  lastButt = buttonState;
}

void calc_joint_angle_goals (){
  g_theta1 = atan2(gy,gx);
  
  float theta234 = gfi;
  
  float p1 = ((gx*cos(g_theta1)) + (gy*sin(g_theta1))) - (L4*cos(theta234));
  float p2 = gz - (L4*sin(theta234));
  float c3 = (((p1*p1) + (p2*p2) - (L2*L2) - (L3*L3))/(2*L2*L3));
  
  g_theta3 = atan2(sqrt(1 - (c3*c3)) , c3);
  g_theta2 = atan2((((L3* c3) + L2)*p2) - (L3*sin(g_theta3)*p1), (((L3*c3) + L2)*p1) + (L3*sin(g_theta3)*p2));
  g_theta4 = theta234 - g_theta2 - g_theta3;

  // Convert to degrees
  g_theta1 = (g_theta1*180)/M_PI;
  g_theta2 = (g_theta2*180)/M_PI;
  g_theta3 = (g_theta3*180)/M_PI;
  g_theta4 = (g_theta4*180)/M_PI;
  
  if ((theta1 != g_theta1) && (theta2 != g_theta2) && (theta3 != g_theta3) && (theta4 != g_theta4)){
    goal_reached = false;
  }
}

// NO SAFETIES / MIN or MAX
// theta1 is a continuous servo motor
// checks radians and degrees
void calc_motion(){

  // If difference in angle to goal is less than 1 degree then go to the angle
  if (abs(theta1 - g_theta1) < 1){
    theta1 = g_theta1;
  }
  // If greater than 1 degree then move 1 degree towards the goal angle
  else if (theta1 <= g_theta1){
    theta1++;
  }
  else if (theta1 >= g_theta1){
    theta1--;
  }

  // If difference in angle to goal is less than 1 degree then go to the angle
  if (abs(theta2 - g_theta2) < 1){
    theta2 = g_theta2;
  }
  // If greater than 1 degree then move 1 degree towards the goal angle
  else if (theta2 <= g_theta2){
    theta2++;
  }
  else if (theta2 >= g_theta2){
    theta2--;
  }

  // If difference in angle to goal is less than 1 degree then go to the angle
  if (abs(theta3 - g_theta3) < 1){
    theta3 = g_theta3;
  }
  // If greater than 1 degree then move 1 degree towards the goal angle
  else if (theta3 <= g_theta3){
    theta3++;
  }
  else if (theta1 >= g_theta1){
    theta3--;
  }

  // If difference in angle to goal is less than 1 degree then go to the angle
  if (abs(theta4 - g_theta4) < 1){
    theta4 = g_theta4;
  }
  // If greater than 1 degree then move 1 degree towards the goal angle
  else if (theta4 <= g_theta4){
    theta4++;
  }
  else if (theta4 >= g_theta1){
    theta4--;
  }

  if ((theta1 == g_theta1) && (theta2 == g_theta2) && (theta3 == g_theta3) && (theta4 == g_theta4)){
    goal_reached = true;
  }
}

void move_joints(){
  SERVO1.write(theta1);                  // WE HAVE TO FIND A SOLUTION FOR THE CONTINUOUS SERVO 
  //On a continuous rotation servo, this will set the speed of the servo (with 0 being full-speed in one direction, 180 being full speed in the other, and a value near 90 being no movement).
    
  SERVO2.write(theta2);
  SERVO3.write(theta3);
  SERVO4.write(theta4);
  delay(10);
}

void move_stepper(){
  step1();
  delay(10);
  step2();
  delay(10);
  step3();
  delay(10);
  step4();
  delay(10);
}

void step1 (){

  digitalWrite(STEPPER_A1, HIGH);
  digitalWrite(STEPPER_A2, LOW); 
  digitalWrite(STEPPER_B1, LOW); 
  digitalWrite(STEPPER_B2, LOW); 

}

void step2 (){

  digitalWrite(STEPPER_A1, LOW);
  digitalWrite(STEPPER_A2, LOW); 
  digitalWrite(STEPPER_B1, HIGH); 
  digitalWrite(STEPPER_B2, LOW); 

}

void step3 (){

  digitalWrite(STEPPER_A1, LOW);
  digitalWrite(STEPPER_A2, HIGH); 
  digitalWrite(STEPPER_B1, LOW); 
  digitalWrite(STEPPER_B2, LOW); 

}

void step4 (){

  digitalWrite(STEPPER_A1, LOW);
  digitalWrite(STEPPER_A2, LOW); 
  digitalWrite(STEPPER_B1, LOW); 
  digitalWrite(STEPPER_B2, HIGH); 

}

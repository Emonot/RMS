#include <Servo.h>
#define _USE_MATH_DEFINES // for C
#include <math.h>
#include <Stepper.h>

//definning pin for stepper motor
#define STEPPER_A1 5 //green
#define STEPPER_A2 12 //yellow
#define STEPPER_B1 6 //red
#define STEPPER_B2 11 //grey
#define STEPSPERREV 200

//defining pin for the servo motor
#define SERVO1 9
#define SERVO2 10
#define SERVO3 7
#define SERVO4 8

//defining pin for the joysticks
#define JOY1 18
#define JOY2 16
#define JOY3 15
#define JOY4 17

//declare pin for the button
#define INPUT1 4

//declare all of our motor
Servo motor1; //base motor to turn the arm around the Z axis
Servo motor2; //first arm to control a theta angle
Servo motor3; //control the second joint
Servo motor4; //control the grab

Stepper baseStepper(STEPSPERREV, STEPPER_A1, STEPPER_A2, STEPPER_B1, STEPPER_B2);

int buttonState; // declare the button for the manual or automatic mode
int lastButt;
bool auto_switch;

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
float a;
float alpha;

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

int steps;
//Declare lengths of arm limbs
float L2;
float L3;
float L4;

//Other variables
bool goal_reached;
float p1;
float p2;
float c3;


void read_joy(){
  joy1 = analogRead(JOY1);            // reads the value of the potentiometer (value between 0 and 1023)
  //Serial.println("--JOY--");
  //Serial.println(joy1);
  joy1 = map(joy1, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  //Serial.println(joy1);
  joy2 = analogRead(JOY2);            // reads the value of the potentiometer (value between 0 and 1023)
  joy2 = map(joy2, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  joy3 = analogRead(JOY3);            // reads the value of the potentiometer (value between 0 and 1023)
  joy3 = map(joy3, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  joy4 = analogRead(JOY4);            // reads the value of the potentiometer (value between 0 and 1023)
  joy4 = map(joy4, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
}


void set_goals(){
  if (joy1 < 80){
    steps--;
    theta1 = 1.8*steps;
    baseStepper.step(-1);
  }
  
  else if (joy1 > 100){
    steps++;
    g_theta1 = g_theta1 + 1.8;
    theta1 = theta1 + 1.8;
    baseStepper.step(1);
  }
  
  if (joy2 < 80){
    theta2--;
  }
  else if (joy2 > 100){
    theta2++;
  }

  if (joy3 < 80){
    theta3--;
  }
  else if (joy3 > 100){
    theta3++;
  }

  if (joy4 < 80){
    theta4--;
  }
  else if (joy4 > 100){
    theta4++;
  }
}

void check_constraints(){
  
  if (g_theta2 > 90){
    Serial.println("Goal moving outside of workspace, theta2 too big");
    g_theta2 = 90;
  } 
  else if (g_theta2 < 0){
    Serial.println("Goal moving outside of workspace, theta2 too small");
    g_theta2 = 0;
  }

  if (g_theta3 > 180){
    Serial.println("Goal moving outside of workspace, theta3 too big");
    g_theta3 = 180;
  } 
  else if (g_theta3 < 0){
    Serial.println("Goal moving outside of workspace, theta3 too small");
    g_theta3 = 0;
  }

  if (g_theta4 > 180){
    Serial.println("Goal moving outside of workspace, theta4 too big");
    g_theta4 = 180;
  } 
  else if (g_theta4 < 0){
    Serial.println("Goal moving outside of workspace, theta4 too small");
    g_theta4 = 0;
  }
}

void gripper(){
  float angle;
  angle = motor4.read();
  Serial.println(angle);
  if(auto_switch==false){
    motor4.write(70);
    auto_switch = true;
  }
  else{
    motor4.write(100);
    auto_switch = false;
  }
  
}

void check_button(){
  buttonState = digitalRead(INPUT1);
  
  // Check to see if the button has changed to HIGH from LOW
  if ((buttonState == HIGH) && (lastButt == LOW)) {
    gripper();
  }
  Serial.println(buttonState);
  // Record buttonState for next cycle
  lastButt = buttonState;
}

void calc_joint_angle_goals (){
  g_theta1 = atan2(gy,gx);
  
  float theta234 = gfi;
  
  float p1 = ((gx*cos(g_theta1)) + (gy*sin(g_theta1))) - (L4*cos(theta234));
  float p2 = gz - (L4*sin(theta234));
  float c3 = (((p1*p1) + (p2*p2) - (L2*L2) - (L3*L3))/(2*L2*L3));
  
  g_theta3 = atan2(-sqrt(1 - (c3*c3)) , c3);
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

void reset(){
  calc_joint_angle_goals();
  baseStepper.step(steps-90);
  
  motor1.write(g_theta2);
  motor2.write(g_theta3);
  motor3.write(g_theta4);
  delay(100);
  
  theta2 = motor2.read();
  theta3 = motor3.read();
  theta4 = motor4.read();
}

void move_joints(){
  //SERVO1.write(theta1);                   
  //move_stepper();
  
  motor1.write(theta2);
  motor2.write(theta3);
  motor3.write(theta4);
  delay(100);

  theta2 = motor1.read();
  theta3 = motor2.read();
  theta4 = motor3.read();
}

void setup() {
  //put pwm option for each of those pin
  Serial.begin(9600);

  //Initialise limb lengths (THESE WILL NEED CORRECTION / CALIBRATION)
  L2 = 130;
  L3 = 70;
  L4 = 170;
  goal_reached = false;
  steps = 90;

  //Initialise goal positions
  gx = 0;
  gy = 200;
  gz = 0;
  gfi = 0;
  
  // Initialise joint positions of arm
  g_theta1 = 0;
  g_theta2 = motor1.read();
  g_theta3 = motor2.read();
  g_theta4 = motor3.read();

  auto_switch = false;
  baseStepper.setSpeed(20);
  
  motor1.attach(SERVO1);
  motor2.attach(SERVO2);
  motor3.attach(SERVO3);
  motor4.attach(SERVO4);

  theta1 = 0;
  theta2 = motor1.read();
  theta3 = motor2.read();
  theta4 = motor3.read();
  
  
  //initialise the button as input
  pinMode(INPUT1, INPUT);
  //reset();
}

void test(){
  motor1.write(30);
  motor2.write(90);
  motor3.write(90);

  delay(3000);

  motor1.write(0);
  motor2.write(120);
  motor3.write(120);

  delay(3000);
  
}

void set_goals_test(){
  theta1 = motor2.read();
  if (joy1 < 80){
    theta1--;
  }
  else if (joy1 > 100){
    theta1++;
  }
  motor2.write(theta1);
}

void loop() {
  check_button();
  /*
  Serial.println("--START--");
  Serial.print("theta1 = ");
  Serial.println(theta1);
  Serial.print("theta2 = ");
  Serial.println(theta2);
  Serial.print("theta3 = ");
  Serial.println(theta3);
  Serial.print("theta4 = ");
  Serial.println(theta4);
  */
  read_joy();
  set_goals();
  //check_constraints();
  move_joints();                         
}

#include <PS3BT.h>
#include <usbhub.h>

#include <Servo.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>      //Serial Peripheral Interface and it is a way to send data between microcontrollers and other small devices

USB Usb;              //USBHub Hub1(&Usb); // Some dongles have a hub inside

BTD Btd(&Usb);        // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
PS3BT PS3(&Btd);    // This will just create the instance
//PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0x3D, 0x0A, 0x57);    // This will also store the bluetooth address -
// this can be obtained from the dongle when running the sketch

//bool printTemperature, printAngle;       //for measuring temperature NO USE

Servo myservo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position

int motorSpeedA = 0;
int motorSpeedB = 0;
int motorSpeedC = 0;
int motorSpeedD = 0;

int gripperopenspeed = 0;
int gripper_rotate =0;
int gripper_up_down =0;

int rx, ry, r2,l2,lx,ly;

#define enA 6       //Move
#define in1 30
#define enB 7
#define in2 32
#define enC 8
#define in3 31
#define enD 9
#define in4 33

#define engriop 2  //gripper open
#define in5 23
#define in6 25

#define enrotate 3  //gripper rotate
#define in7 27
#define in8 29

#define engripup 4  //gripper up/down
#define in9 22
#define in10 24

#define enslider 5  //back slider
#define in11 26
#define in12 28

#define enlid 10  //back lid
#define in13 35
#define in14 37


void setup() {
  Serial.begin(115200);

#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif

  if (Usb.Init() == -1)
  { Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));

  pinMode(enA, OUTPUT);   
  pinMode(enB, OUTPUT);
  pinMode(enC, OUTPUT);   
  pinMode(enD, OUTPUT);
  pinMode(in1, OUTPUT);   
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);   
  pinMode(in4, OUTPUT);

  pinMode(engriop, OUTPUT); //gripper open;
  pinMode(in5, OUTPUT);   
  pinMode(in6, OUTPUT);

  pinMode(enrotate,OUTPUT); //gripper rotate
  pinMode(in7, OUTPUT);
  pinMode(in8, OUTPUT);

  pinMode(engripup,OUTPUT);  //gripper up/down
  pinMode(in9, OUTPUT);
  pinMode(in10, OUTPUT);

  pinMode(enslider,OUTPUT);   //back slidder
  pinMode(in11, OUTPUT);
  pinMode(in12, OUTPUT);

  pinMode(enlid,OUTPUT);   //back LID
  pinMode(in13, OUTPUT);
  pinMode(in14, OUTPUT);
  
  myservo.attach(49);   //lid up/down
}


void loop()
{
  Usb.Task();

  if (PS3.PS3Connected || PS3.PS3NavigationConnected)
  { //Serial.println(PS3.getAnalogHat(RightHatY));Serial.println(PS3.getAnalogHat(RightHatX));
    
    //RightHatY (Y-Axis of Right) used for forward and backward control
    rx = PS3.getAnalogHat(RightHatX);
    ry = PS3.getAnalogHat(RightHatY);
    r2 = PS3.getAnalogButton(R2);
    l2 = PS3.getAnalogButton(L2);

    lx = PS3.getAnalogHat(LeftHatX);
    ly = PS3.getAnalogHat(LeftHatY);
    
    if (ry <= 115 && rx == 127)     //Forward
    { Serial.println("Forward");
      digitalWrite(in1, HIGH);  //MOTOR A    // Also HIGH for clockwise (Set accordingly)

      digitalWrite(in2, HIGH);  //MOTOR B

      digitalWrite(in3, HIGH);  //MOTOR C

      digitalWrite(in4, HIGH);  //MOTOR D


      motorSpeedA = map(ry, 115, 0, 0, 255);
      motorSpeedB = map(ry, 115, 0, 0, 255);
      motorSpeedC = map(ry, 115, 0, 0, 255);
      motorSpeedD = map(ry, 115, 0, 0, 255);
    }

    else if (ry >= 140 && rx == 127)     //Backward
    { Serial.println("Backward");
      digitalWrite(in1, LOW);  //MOTOR A

      digitalWrite(in2, LOW);  //MOTOR B

      digitalWrite(in3, LOW);  //MOTOR C

      digitalWrite(in4, LOW);  //MOTOR D


      motorSpeedA = map(ry, 140, 255, 0, 255);
      motorSpeedB = map(ry, 140, 255, 0, 255);
      motorSpeedC = map(ry, 140, 255, 0, 255);
      motorSpeedD = map(ry, 140, 255, 0, 255);
    }

    // RightHatX (X-Axis of Right) is used for left and right control;
    else if (rx >= 140 && ry == 127)     //Right
    { Serial.println("Right");
      digitalWrite(in1, HIGH);  //MOTOR A   forward move

      digitalWrite(in2, LOW);  //MOTOR B  backward move

      digitalWrite(in3, LOW);  //MOTOR C   backward move

      digitalWrite(in4, HIGH);  //MOTOR D    forward move


      motorSpeedA = map(rx, 140, 255, 0, 255);
      motorSpeedB = map(rx, 140, 255, 0, 255);
      motorSpeedC = map(rx, 140, 255, 0, 255);
      motorSpeedD = map(rx, 140, 255, 0, 255);
    }

    else if (rx < 115 && ry == 127)     //Left
    { Serial.println("Left");
      digitalWrite(in1, LOW);  //MOTOR A    backward

      digitalWrite(in2, HIGH);  //MOTOR B    forward

      digitalWrite(in3, HIGH);  //MOTOR C    forward

      digitalWrite(in4, LOW);  //MOTOR D    backward


      motorSpeedA = map(rx, 115, 0, 0, 255);
      motorSpeedB = map(rx, 115, 0, 0, 255);
      motorSpeedC = map(rx, 115, 0, 0, 255);
      motorSpeedD = map(rx, 115, 0, 0, 255);
    }

    else if (rx < 127 && ry < 127)     //ForwardLeft
    { Serial.println("ForwardLeft");
      digitalWrite(in1, LOW);  //MOTOR A    stop

      digitalWrite(in2, HIGH);  //MOTOR B    forward

      digitalWrite(in3, HIGH);  //MOTOR C    forward

      digitalWrite(in4, LOW);  //MOTOR D    stop


      motorSpeedA = 0;        //map(RightHatX, 127,0,0,0); //stop
      motorSpeedB = 200;        //map(RightHatX, 127,0,0,255);
      motorSpeedC = 200;        //map(RightHatX, 127,0,0,255);
      motorSpeedD = 0;        //map(RightHatX, 127,0,0,0);
    }

    else if (rx > 127 && ry < 127)     //Forwardright
    { Serial.println("ForwardRight");
      digitalWrite(in1, HIGH);  //MOTOR A    forward

      digitalWrite(in2, LOW);  //MOTOR B    stop

      digitalWrite(in3, LOW);  //MOTOR C    stop

      digitalWrite(in4, HIGH);  //MOTOR D    forward


      motorSpeedA = 200;    //map(RightHatX, 127,0,0,255); //stop
      motorSpeedB = 0;    //map(RightHatX, 127,0,0,0);
      motorSpeedC = 0;    //map(RightHatX, 127,0,0,0);
      motorSpeedD = 200;    //map(RightHatX, 127,0,0,255);
    }

     else if (rx < 127 && ry > 127)     //Backwardleft
    { Serial.println("Backwardleft");
      digitalWrite(in1, LOW);  //MOTOR A   backward

      digitalWrite(in2, LOW);  //MOTOR B    stop

      digitalWrite(in3, LOW);  //MOTOR C    stop

      digitalWrite(in4, LOW);  //MOTOR D   backward


      motorSpeedA = 200;    //map(RightHatX, 127,0,0,255); //stop
      motorSpeedB = 0;    //map(RightHatX, 127,0,0,0);
      motorSpeedC = 0;    //map(RightHatX, 127,0,0,0);
      motorSpeedD = 200;    //map(RightHatX, 127,0,0,255);
    }
   
    else if (rx > 127 && ry > 127)     //Backwardright
    { Serial.println("Backwardright");
      digitalWrite(in1, LOW);  //MOTOR A    stop

      digitalWrite(in2, HIGH);  //MOTOR B    forward

      digitalWrite(in3, HIGH);  //MOTOR C    forward

      digitalWrite(in4, LOW);  //MOTOR D    stop


      motorSpeedA = 0;        //map(RightHatX, 127,0,0,0); //stop
      motorSpeedB = 200;        //map(RightHatX, 127,0,0,255);
      motorSpeedC = 200;        //map(RightHatX, 127,0,0,255);
      motorSpeedD = 0;        //map(RightHatX, 127,0,0,0);
    }

    else
    {
      motorSpeedA = 0;
      motorSpeedB = 0;
      motorSpeedC = 0;
      motorSpeedD = 0;
    }
    //Use of Right Joystick is done in moving forward,backward,left and right

    if (motorSpeedA < 50)       // Prevent buzzing of motor by stoping at slow speed
    {
      motorSpeedA = 0;
    }
    if (motorSpeedB < 50)
    {
      motorSpeedB = 0;
    }
    if (motorSpeedC < 50)
    {
      motorSpeedC = 0;
    }
    if (motorSpeedD < 50)
    {
      motorSpeedD = 0;
    }

    analogWrite(enA, motorSpeedA); // Send PWM signal to motor A
    analogWrite(enB, motorSpeedB); // Send PWM signal to motor B
    analogWrite(enC, motorSpeedC); // Send PWM signal to motor C
    analogWrite(enD, motorSpeedD); // Send PWM signal to motor D
    
    Serial.println(motorSpeedA);
    Serial.println(motorSpeedB);
    Serial.println(motorSpeedC);
    Serial.println(motorSpeedD);
    
    

    if (PS3.getButtonClick(R1))     //clockwiseturn
    { Serial.println("Turn Clockwise");
      digitalWrite(in1, HIGH);  //MOTOR A    forward

      digitalWrite(in2, LOW);  //MOTOR B    backward

      digitalWrite(in3, HIGH);  //MOTOR C    forward

      digitalWrite(in4, LOW);  //MOTOR D    backward


      motorSpeedA = 150; //clockwise turn
      motorSpeedB = 150;
      motorSpeedC = 150;
      motorSpeedD = 150;

      delay(1000);
    }

    else if (PS3.getButtonClick(L1))     //anticlockwiseturn
    { Serial.println("Turn Anticlockwise");
      digitalWrite(in1, LOW);  //MOTOR A    backward

      digitalWrite(in2, HIGH);  //MOTOR B    forward

      digitalWrite(in3, LOW);  //MOTOR C    backward

      digitalWrite(in4, HIGH);  //MOTOR D    forward


      motorSpeedA = 150; //anticlockwise turn
      motorSpeedB = 150;
      motorSpeedC = 150;
      motorSpeedD = 150;

      delay(1000);
    }
    

    
    /*
    if (PS3.getButtonClick(DOWN))    //lid
    { Serial.println("Servoclockwise lid_down");
      servomotorclockwise();
    }

    if (PS3.getButtonClick(UP))
    { Serial.println("Servoanticlockwise lid_up");
      servomotoranticlockwise();
    }
    */
    if (r2>0)   //gripper opening
    { 
      digitalWrite(in5,HIGH);
      digitalWrite(in6,LOW);
      gripperopenspeed = map(r2, 0, 255, 0, 255);
      analogWrite(engriop,gripperopenspeed);
      Serial.println("Gripper Opening");
      Serial.println(gripperopenspeed);
    }

    else if (l2>0)
    {
      digitalWrite(in5,LOW);
      digitalWrite(in6,HIGH);
      gripperopenspeed = map(l2, 0, 255, 0, 255);
      analogWrite(engriop,gripperopenspeed);
      Serial.println("Gripper Closing");
      Serial.println(gripperopenspeed);
    }

    else 
    {
      digitalWrite(in5,LOW);
      digitalWrite(in6,LOW);
    }
    // LeftHatX (X-Axis of Right) is used for rotating gripper;
    if (lx >= 140 && ly == 127)     //Right
    { Serial.println("Gripper Rotate clock");
      digitalWrite(in7,LOW);  //MOTOR clockwise   
      digitalWrite(in8,HIGH);
      gripper_rotate = map(lx, 140, 255, 0, 255);
    }

    else if (lx < 115 && ly == 127)     //Left
    { Serial.println("Gripper Rotate anticlock");
      digitalWrite(in7, HIGH);  //MOTOR anticlock
      digitalWrite(in8, LOW);
      gripper_rotate = map(lx, 115, 0, 0, 255);
    }

    else
    {
      digitalWrite(in7, LOW);  //MOTOR anticlock
      digitalWrite(in8, LOW);
    }
    analogWrite(enrotate,gripper_rotate);

    // LeftHaty is used for UP_DOWN gripper;
    if (ly <= 115 && lx == 127)     //UP
    { Serial.println("Gripper Up");
      digitalWrite(in9, LOW);  
      digitalWrite(in10, HIGH);  
      gripper_up_down = map(ly, 115, 0, 0, 255);
    }

    else if (ly >= 140 && lx == 127)     //Down
    { Serial.println("Gripper Down");
      digitalWrite(in9, HIGH);
      digitalWrite(in10, LOW);
      gripper_up_down = map(ly, 140, 255, 0, 255);
    }

    else
    {
      digitalWrite(in9, LOW);
      digitalWrite(in10, LOW);
    }
    analogWrite(engripup,gripper_up_down);


    if (PS3.getButtonClick(LEFT)) //backslidder up/DOWN
    {
      Serial.println("slidder DOWN");
      digitalWrite(in11, HIGH);
      digitalWrite(in12, LOW);
      analogWrite(enslider,255);
      delay(1000);
    }

    
    else if (PS3.getButtonClick(RIGHT)) 
    {
      Serial.println("slidder UP");
      digitalWrite(in11, LOW);
      digitalWrite(in12, HIGH);
      analogWrite(enslider,255);
      delay(1000);
    }

    else
    {
      digitalWrite(in11, LOW);
      digitalWrite(in12, LOW);
    }

    if (PS3.getButtonClick(DOWN)) //LID up/DOWN
    {
      Serial.println("slidder DOWN");
      digitalWrite(in13, HIGH);
      digitalWrite(in14, LOW);
      analogWrite(enlid,255);
      delay(500);
    }

    
    else if (PS3.getButtonClick(UP)) 
    {
      Serial.println("slidder UP");
      digitalWrite(in13, LOW);
      digitalWrite(in14, HIGH);
      analogWrite(enlid,255);
      delay(500);
    }

    else
    {
      digitalWrite(in13, LOW);
      digitalWrite(in14, LOW);
    }
  }
}

/*
void servomotorclockwise()
{ for (pos = 0; pos <= 90; pos += 1)
  { // goes from 0 degrees to 90 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(50);                       // waits 15ms for the servo to reach the position
  }
}

void servomotoranticlockwise()
{
  for (pos = 90; pos >= 0; pos -= 1)
  { // goes from 90 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(50);                       // waits 15ms for the servo to reach the position}
  }
}
*/

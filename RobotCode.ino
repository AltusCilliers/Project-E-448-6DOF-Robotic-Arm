#include <AccelStepper.h>
#include <ESP32Servo.h>

Servo myservo;  // create servo object to control a servo

int servoPin = 5;


String DataIn;
String command;
char receivedByte;
float Jangles[6];

int i1; //Indexes for Serial Data Parsing
int i2;
int i3;
int i4;
int i5;
int i6;

int counter1 = 0;
int counter2 = 0;
int counter3 = 0;
int counter4 = 0;
int counter5 = 0;
int counter6 = 0;

int homingValue1 = 10000; //Dummy Value for now
int homingValue2 = 1000; //Dummy Value for now
int homingValue3 = -1000; //Dummy Value for now
int homingValue4 = 1000; //Dummy Value for now
int homingValue5 = -500; //Dummy Value for now
int homingValue6 = 1000; //Dummy Value for now
int moveAllowed = 1; //Variable to allow movement of motors or not
int J2SwitchPressed = 0;
int J3SwitchPressed = 0;

int stepsToMoveJ1 = 0;
int stepsToMoveJ2 = 0;
int stepsToMoveJ3 = 0;
int stepsToMoveJ4 = 0;
int stepsToMoveJ5 = 0;
int stepsToMoveJ6 = 0;

////////////////////////////////////////////////////
//ADD LIMITS IN STEPS FOR SAFE OPERATION
////////////////////////////////////////////////////


////////////////////////Define Gear Ratios////////////////////////
const float J1Ratio = 5;
const float J2Ratio = 7;
const float J3Ratio = 7.5;
const float J4Ratio = 3;
const float J5Ratio = 2.25;
const float J6Ratio = 1;
///////////////////////////////////////////////////////////////


////////////////////////Define GPIO Pins////////////////////////
#define J1LimitSwitch 15
#define J2LimitSwitch 2
#define J3LimitSwitch 0
#define J4LimitSwitch 4
#define J5LimitSwitch 16
#define J6LimitSwitch 17
///////////////////////////////////////////////////////////////

////////////////////////Define Motor Pins////////////////////////

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define motorInterfaceType 1
#define dirPin1 22
#define stepPin1 23
#define dirPin2 21
#define stepPin2 19
#define dirPin3 33
#define stepPin3 25
#define dirPin4 26
#define stepPin4 27
#define dirPin5 14
#define stepPin5 12
#define dirPin6 13
#define stepPin6 9

// Create 6 new instances of the AccelStepper class (1 for each motor):
AccelStepper stepper1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1); //Gear Ratio = 100/20 = 5 and x is positive direction accoridng to convention
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2); //Gear Ratio = 140/20 = 7 and x is positive direction accoridng to convention
AccelStepper stepper3 = AccelStepper(motorInterfaceType, stepPin3, dirPin3); //Gear Ratio = 120/16 = 7.5 and - is positive direction accoridng to convention
AccelStepper stepper4 = AccelStepper(motorInterfaceType, stepPin4, dirPin4); //Gear Ratio = 48/16 = 3 and x is positive direction accoridng to convention
AccelStepper stepper5 = AccelStepper(motorInterfaceType, stepPin5, dirPin5); //Gear Ratio = 32/16 = 2 and x is positive direction accoridng to convention
AccelStepper stepper6 = AccelStepper(motorInterfaceType, stepPin6, dirPin6); //Gear Ratio = 1/1 = 1 and x is positive direction accoridng to convention

///////////////////////////////////////////////////////////////


void setup() {
Serial.begin(115200); // open the serial port at 9600 bps:

ESP32PWM::allocateTimer(0);
ESP32PWM::allocateTimer(1);
ESP32PWM::allocateTimer(2);
ESP32PWM::allocateTimer(3);
myservo.setPeriodHertz(50);    // standard 50 hz servo
myservo.attach(servoPin, 1000, 2000); // attaches the servo on pin 18 to the servo object

////////////////////////Define Pins////////////////////////
//int servoPin = 21;
//pinMode(21,OUTPUT);

pinMode(J1LimitSwitch,INPUT);
pinMode(J2LimitSwitch,INPUT_PULLUP);
pinMode(J3LimitSwitch,INPUT_PULLUP);
pinMode(J4LimitSwitch,INPUT_PULLUP);
pinMode(J5LimitSwitch,INPUT_PULLUP);
pinMode(J6LimitSwitch,INPUT_PULLUP);
//attachInterrupt(digitalPinToInterrupt(J2LimitSwitch),motorStop,FALLING);

//Setup Motors
stepper1.setMaxSpeed(200.0);
stepper1.setAcceleration(250.0);
stepper2.setMaxSpeed(10.0);
stepper2.setAcceleration(150.0);
stepper3.setMaxSpeed(50.0);
stepper3.setAcceleration(200.0);
stepper4.setMaxSpeed(50.0);
stepper4.setAcceleration(200.0);
stepper5.setMaxSpeed(30.0);
stepper5.setAcceleration(200.0);
stepper6.setMaxSpeed(25.0);
stepper6.setAcceleration(250.0);

}

void loop() 
{
  
  SerialCommunication();   
  checkLimitSwitches();

  //Run Motors (must be called in main loop as fast as possible)
  stepper1.run();
  stepper2.run();
  stepper3.run();
  stepper4.run();
  stepper5.run();
  stepper6.run();
  
}

void SerialCommunication()
{
  while (Serial.available())
    {

      //Read a char byte one by one
      receivedByte = Serial.read();
      //Add received byte to string buffer containing all sent data
      DataIn += receivedByte;
      //Serial.print("New Input Data Received Is :");
      //Serial.println(DataIn);

      // If input byte is a control char eg. \n, stop receiving data and exit while serial avail. loop
      if (isControl(receivedByte))
      {
        //Serial.println("Break");
        //trim away whitespace and \n
        DataIn.trim();
        command =  DataIn.substring(0,2);
        //Serial.print("Command Is :");
        //Serial.println(command);
        DataIn = DataIn.substring(2);
        //Serial.print("Data In Is :");
        //Serial.println(DataIn);
        
        break; 
      }

    }

    //if Sent Command is move joints
    // Example Command: "MJ,11,22,33,44,55,66" (MJ,joint1 angle, joint2 angle, etc)
    if(command == "MJ")
    {
      //indexes
      i1 = DataIn.indexOf(',');
      i2 = DataIn.indexOf(',',i1+1);
      i3 = DataIn.indexOf(',',i2+1);
      i4 = DataIn.indexOf(',',i3+1);
      i5 = DataIn.indexOf(',',i4+1);
      i6 = DataIn.indexOf(',',i5+1);
      Jangles[0] = -DataIn.substring(i1+1,i2).toFloat();
      Jangles[1] = DataIn.substring(i2+1,i3).toFloat();
      Jangles[2] = -DataIn.substring(i3+1,i4).toFloat();
      Jangles[3] = -DataIn.substring(i4+1,i5).toFloat();
      Jangles[4] = DataIn.substring(i5+1,i6).toFloat();
      Jangles[5] = -DataIn.substring(i6+1).toFloat();
      Serial.print("Joint Angle 1 Is :");
      Serial.println(Jangles[0]);
      Serial.print("Joint Angle 2 Is :");
      Serial.println(Jangles[1]);
      Serial.print("Joint Angle 3 Is :");
      Serial.println(Jangles[2]);
      Serial.print("Joint Angle 4 Is :");
      Serial.println(Jangles[3]);
      Serial.print("Joint Angle 5 Is :");
      Serial.println(Jangles[4]);
      Serial.print("Joint Angle 6 Is :");
      Serial.println(Jangles[5]);

      moveJoints();
    }

    //if Sent Command is jog (a single) joint
    // Example Command: "JJ,3,50" (JJ,joint,angle)
    if(command == "JJ")
    {
      i1 = DataIn.indexOf(',');
      i2 = DataIn.indexOf(',',i1+1);
      int j = DataIn.substring(i1+1,i2).toInt();
      Jangles[j-1] = DataIn.substring(i2+1).toFloat();
      Serial.print("Joint Angle ");
      Serial.print(j);
      Serial.print(" is ");     
      Serial.println(Jangles[j-1]);
      //Change angle to negative of what received due to convention and default direction of stepper motor
      //jogJoint(Jangles[j-1],j);

      switch(j)
      {
        case 1:
          jogJoint(-Jangles[j-1],j);
          break;
        
        case 2:
          jogJoint(Jangles[j-1],j);
          break;
        
        case 3:
          jogJoint(-Jangles[j-1],j);
          break;
        
        case 4:
          jogJoint(-Jangles[j-1],j);
          break;
        
        case 5:
          jogJoint(Jangles[j-1],j);
          break;
        
        case 6:
          jogJoint(-Jangles[j-1],j);
          break;
        
        default:
          break;
      }
      
    }

    //Home Robot
    if(command == "HR")
    {
      homing();
    }

    if(command == "SP")
    {
      stepper1.setCurrentPosition(0);
      stepper2.setCurrentPosition(350); 
      stepper3.setCurrentPosition(0);
      stepper4.setCurrentPosition(0);
      stepper5.setCurrentPosition(0);
      stepper6.setCurrentPosition(0);
      Serial.print("Hi ");
    }

    if(command == "SM")
    {
      stepper1.stop();
      stepper2.stop();
      stepper3.stop();
      stepper4.stop();
      stepper5.stop();
      stepper6.stop();
      Serial.print("HAAAAAAAAAAAAAAALT");
    }

    if(command == "OG")
    {
      int pos = 40;
      Serial.println("Open");
      myservo.write(pos);
    }
    
    if(command == "CG")
    {
      int pos = 10;
      Serial.println("Close");
      myservo.write(pos);
    }

    if(command == "RG")
    {
      int pos = 5;
      Serial.println("Reset");
      myservo.write(pos);
    }
  
    //When Moving etc is Complete, Clear received Buffer
    command = "";
    DataIn = "";
}

void homing()
{
  //moveAllowed = 1;
  Serial.println("Robot is Homing");
  //set acceleration and max speed
  stepper1.setMaxSpeed(150.0);
  stepper1.setAcceleration(850.0);
  stepper2.setMaxSpeed(25.0);
  stepper2.setAcceleration(250.0);
  stepper3.setMaxSpeed(25.0);
  stepper3.setAcceleration(250.0);
  stepper4.setMaxSpeed(25.0);
  stepper4.setAcceleration(250.0);
  stepper5.setMaxSpeed(30.0);
  stepper5.setAcceleration(200.0);
  stepper6.setMaxSpeed(25.0);
  stepper6.setAcceleration(250.0);
    
  //moveTo absolute position in steps
  stepper1.moveTo(homingValue1);
  stepper2.moveTo(homingValue2);
  stepper3.moveTo(homingValue3);
  stepper4.moveTo(homingValue4);
  //stepper5.moveTo(homingValue5);
  stepper6.moveTo(homingValue6);
  Serial.println("Moving to Limit Switch");
  

 
}

void homing1()
{
  //stepper3.setCurrentPosition(0);
  Serial.println("Moving to Home Position 1");
  stepper1.move(-4066); //Number of Offset Steps to go from Limit Switch to Home Position for Joint 3
  stepper1.setMaxSpeed(350.0);
  stepper1.setAcceleration(450.0);
  
}

void homing2()
{
  //stepper3.setCurrentPosition(0);
  Serial.println("Moving to Home Position 2");
  stepper2.move(-148); //Number of Offset Steps to go from Limit Switch to Home Position for Joint 3 
  //CHANGE
  stepper2.setMaxSpeed(30.0);
  stepper2.setAcceleration(100.0);
  
}

void homing3()
{
  //stepper3.setCurrentPosition(0);
  Serial.println("Moving to Home Position 3");
  stepper3.move(354); //Number of Offset Steps to go from Limit Switch to Home Position for Joint 3
  stepper3.setMaxSpeed(50.0);
  stepper3.setAcceleration(200.0);
}


void homing4()
{
  //stepper3.setCurrentPosition(0);
  Serial.println("Moving to Home Position 4");
  stepper4.move(-158); //Number of Offset Steps to go from Limit Switch to Home Position for Joint 3
  stepper4.setMaxSpeed(50.0);
  stepper4.setAcceleration(200.0);
}

void homing5()
{
  Serial.println("Moving to Home Position 5");
  stepper5.move(185); //Number of Offset Steps to go from Limit Switch to Home Position for Joint 3
  stepper5.setMaxSpeed(50.0);
  stepper5.setAcceleration(200.0);
}

void homing6()
{
  Serial.println("Moving to Home Position 6");
  stepper6.move(-98); //Number of Offset Steps to go from Limit Switch to Home Position for Joint 3
  stepper6.setMaxSpeed(25.0);
  stepper6.setAcceleration(250.0);
}

void checkLimitSwitches()
{
  if(digitalRead(J1LimitSwitch) == HIGH)
  {
    counter1=0;  
  }else if (digitalRead(J1LimitSwitch) == LOW)
  {
    counter1++;
    if(counter1==1000 && digitalRead(J1LimitSwitch) == LOW)
    {
      Serial.println("J1 Activated");
      stepper1.stop();
      homing1();
      counter1=0;
    } 
  }
    
//  if(digitalRead(J2LimitSwitch) == LOW && J2SwitchPressed==0)
//  {
//    counter2++;
//    if(counter2==1000 && digitalRead(J2LimitSwitch) == LOW)
//    {
//      Serial.println("J2 Activated");
//      stepper2.stop();
//      homing2();
//      counter2=0;
//    }   
//  } 

  if(digitalRead(J2LimitSwitch) == HIGH)
  {
    counter2=0;  
  }else if (digitalRead(J2LimitSwitch) == LOW)
  {
    counter2++;
    if(counter2==1000 && digitalRead(J2LimitSwitch) == LOW)
    {
      Serial.println("J2 Activated");
      stepper2.stop();
      homing2();
      counter2=0;
    } 
  }
  
  if(digitalRead(J3LimitSwitch) == LOW)
  {
    counter3++;
    if(counter3==1000 && digitalRead(J3LimitSwitch) == LOW)
    {
      Serial.println("J3 Activated");
      stepper3.stop();
      homing3();
      counter3=0;
    } 
  }

//  if(digitalRead(J4LimitSwitch) == LOW)
//  {
//    counter4++;
//    if(counter4==1000 && digitalRead(J4LimitSwitch) == LOW)
//    {
//      Serial.println("J4 Activated");
//      stepper4.stop();
//      homing4();
//      counter4=0;
//    }  
//  } 

if(digitalRead(J4LimitSwitch) == HIGH)
  {
    counter4=0;  
  }else if (digitalRead(J4LimitSwitch) == LOW)
  {
    counter4++;
    if(counter4==1000 && digitalRead(J4LimitSwitch) == LOW)
    {
      Serial.println("J4 Activated");
      stepper4.stop();
      homing4();
      counter4=0;
    } 
  }

//  if(digitalRead(J5LimitSwitch) == LOW)
//  {
//    counter5++;
//    if(counter5==1000 && digitalRead(J5LimitSwitch) == LOW)
//    {
//      Serial.println("J5 Activated");
//      stepper5.stop();
//      homing5();
//      counter5=0;
//    } 
//  }

  if(digitalRead(J5LimitSwitch) == HIGH)
  {
    counter5=0;  
  }else if (digitalRead(J5LimitSwitch) == LOW)
  {
    counter5++;
    if(counter5==1000 && digitalRead(J5LimitSwitch) == LOW)
    {
      Serial.println("J5 Activated");
      stepper5.stop();
      homing5();
      counter5=0;
    } 
  }

//  if(digitalRead(J6LimitSwitch) == LOW)
//  {
//    counter6++;
//    if(counter6==1000 && digitalRead(J6LimitSwitch) == LOW)
//    {
//      Serial.println("J6 Activated");
//      stepper6.stop();
//      homing6();
//      counter6=0;
//    } 
//  }

  if(digitalRead(J6LimitSwitch) == HIGH)
  {
    counter6=0;  
  }else if (digitalRead(J6LimitSwitch) == LOW)
  {
    counter6++;
    if(counter6==1000 && digitalRead(J6LimitSwitch) == LOW)
    {
      Serial.println("J6 Activated");
      stepper6.stop();
      homing6();
      counter6=0;
    } 
  }

  
}

void jogJoint(float jointAngle, int j)
{

  switch(j)
  {
    case 1:
      Serial.println(int(jointAngle*J1Ratio));
      stepsToMoveJ1 = (1600*jointAngle*J1Ratio)/360; //maybe add 1/2 divisor (180) to increase accuracy if needed
      Serial.println(stepsToMoveJ1);
      stepper1.moveTo(stepsToMoveJ1);
      break;

    case 2:
      stepper2.setMaxSpeed(20.0);
      stepper2.setAcceleration(110.0);
      Serial.println(int(jointAngle*J2Ratio));
      stepsToMoveJ2 = (200*jointAngle*J2Ratio)/360; //maybe add 1/2 divisor (180) to increase accuracy if needed
      Serial.println(stepsToMoveJ2);
      stepper2.moveTo(stepsToMoveJ2);
      break;

    case 3:
      Serial.println(int(jointAngle*J3Ratio));
      stepsToMoveJ3 = (200*jointAngle*J3Ratio)/360; //maybe add 1/2 divisor (180) to increase accuracy if needed
      Serial.println(stepsToMoveJ3);
      stepper3.moveTo(stepsToMoveJ3);
      break;
    
    case 4:
      Serial.println(int(jointAngle*J4Ratio));
      stepsToMoveJ4 = (200*jointAngle*J4Ratio)/360; //maybe add 1/2 divisor (180) to increase accuracy if needed
      Serial.println(stepsToMoveJ4);
      stepper4.moveTo(stepsToMoveJ4);
      break;

    case 5:
      Serial.println(int(jointAngle*J5Ratio));
      stepsToMoveJ5 = (200*jointAngle*J5Ratio)/360; //maybe add 1/2 divisor (180) to increase accuracy if needed
      Serial.println(stepsToMoveJ5);
      stepper5.moveTo(stepsToMoveJ5);
      break;

    case 6:
      Serial.println(int(jointAngle*J6Ratio));
      stepsToMoveJ6 = (200*jointAngle*J6Ratio)/360; //maybe add 1/2 divisor (180) to increase accuracy if needed
      Serial.println(stepsToMoveJ6);
      stepper6.moveTo(stepsToMoveJ6);

      break;

    default:
      break;
  }
}

void moveJoints()
{
  stepper2.setMaxSpeed(20.0);
  stepper2.setAcceleration(110.0);
 
  stepsToMoveJ1 = (1600*Jangles[0]*J1Ratio)/360;
  stepsToMoveJ2 = (200*Jangles[1]*J2Ratio)/360;
  stepsToMoveJ3 = (200*Jangles[2]*J3Ratio)/360;
  stepsToMoveJ4 = (200*Jangles[3]*J4Ratio)/360;
  stepsToMoveJ5 = (200*Jangles[4]*J5Ratio)/360;
  stepsToMoveJ6 = (200*Jangles[5]*J6Ratio)/360;

  stepper1.moveTo(stepsToMoveJ1);
  stepper2.moveTo(stepsToMoveJ2);
  stepper3.moveTo(stepsToMoveJ3);
  stepper4.moveTo(stepsToMoveJ4);
  stepper5.moveTo(stepsToMoveJ5);
  stepper6.moveTo(stepsToMoveJ6);

  Serial.println("moveJoints function called");
}

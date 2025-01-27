#include "DRV8825.h"
// For RAMPS 1.4
/*
bugs: Button spam

43-1 reduction 
big wheel = 
little wheel = 15 1/2
*/
#define X_STEP_PIN 54 
#define X_DIR_PIN 55
#define X_ENABLE_PIN 38

#define Y_STEP_PIN 60
#define Y_DIR_PIN 61
#define Y_ENABLE_PIN 56

#define Z_STEP_PIN 46
#define Z_DIR_PIN 48
#define Z_ENABLE_PIN 62

// extruder 1
#define E0_STEP_PIN 26
#define E0_DIR_PIN 28
#define E0_ENABLE_PIN 24

// extruder 2
#define E1_STEP_PIN 36
#define E1_DIR_PIN 34
#define E1_ENABLE_PIN 30

// Wheel drivers pinout --- X/Y Left/Right Side's
// Left Side of Rover --- Pin = X 
DRV8825 leftSide(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, 100 * 64);
// Right Side of Rover --- Pin = Y
DRV8825 rightSide(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, 100 * 64);

// Shoulder driver Pinouts --- E0/E1 front/back of Rover
// Set up for the front Arm controller -- Pin = E1
DRV8825 frontShoulder(E1_STEP_PIN,E1_DIR_PIN,E1_ENABLE_PIN,100*64);
// Set up for the back Arm controller -- Pin = E0
DRV8825 BackShoulder(E0_STEP_PIN,E0_DIR_PIN,E0_ENABLE_PIN,100*64);
// Free driver: Z
DRV8825 FreeDriver(Z_STEP_PIN,Z_DIR_PIN,Z_ENABLE_PIN,100*64);


enum COMMANDS
{
  STOP = 0x00,
  FWD = 0x01,
  REV = 0x02,
  LEFT = 0x03,
  RIGHT = 0x04,
  HALT = 0xff,
  FRONT = 0x05,
  BACK = 0x06,
  RAISE = 0X07,
  LOWER = 0X08
};

byte last_command = STOP;

long last_command_time = 0;  // ms since last command
long command_timeout = 1000; // ms to wait for next command before stopping
long tmp1 = 1;

// change this number to adjust the max rpms of the stepper motors
long maxSpeed = 16;

// global variables -- can be changed in a later version 
long idle_speed = 1;
long time = millis();
long timeout = 0;
long time1 = 0;
// right side
bool rs = false;
// left side
bool ls = false;
// global checkers for starting or stopping 
int on = 0;
int stop = 0;

int leftSpeed = 0;
int rightSpeed = 0;



void setup()
{

  // use USB on serial 115200
  Serial.begin(115200);


  // set up the LED for ability to see if recieving commands
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  // wheels right/left initial
  rightSide.set_enabled(true);
  rightSide.set_direction(false);
  rightSide.set_speed(0);

  leftSide.set_enabled(true);
  leftSide.set_direction(false);
  leftSide.set_speed(0);

  // shoulders front/back initial 
  frontShoulder.set_enabled(true);
  frontShoulder.set_direction(false);
  frontShoulder.set_speed(0);

  BackShoulder.set_enabled(true);
  BackShoulder.set_direction(false);
  BackShoulder.set_speed(0);

}

long last_frequency_check_time = 0;
long counter = 0;


void loop()
{

  time1 = millis();
  update_motors();


  read_serial();
  
  
  // Controlls the slow start for the stepper motor checks if a certain amount of miliseconds have passed and if we want to speed up
  if(time1 - time >= 750 && on == 1)
  {

    Speedup(idle_speed, ls, rs);
  }
  // controlls the soft stop for the stepper motor checks if a certain amount of have passed and if we want to stop
  if(time1 - time >= 750 && stop == 1)
  {
    Stop();
  }
   // call stop function if we get stuck in a loop and it wont slow down after 2500 miliseconds 
  if(timeout == 750)
  {
    stop = 1;
    on = 0;
    Stop();
  }

}
// updates the motors
void update_motors()
{

  leftSide.update();
  rightSide.update();
  BackShoulder.update();
  frontShoulder.update();

}
// getting hung up and sending stop when we dont want to stop
// checks for commands being sent over the Serial port to the arduino/Ramps board
void read_serial()
{
  if (Serial.available())
  {
    last_command = Serial.read();
  
    switch (last_command)
    {
      case STOP:
        stop = 1;
        on = 0;
        frontShoulder.set_speed(0);
        BackShoulder.set_speed(0);
        Stop();
        break;
      case FWD:
      
            if(leftSpeed >= 3 && rightSpeed >= 3) // if we are already at max going forward do nothing
              break;
            if(leftSpeed < 3)
              leftSpeed++;
            if(rightSpeed < 3)
              rightSpeed++;
            on = 1;
            stop = 0;
            tmp1 = idle_speed;
            
            if(leftSpeed < 0)
              ls = false;
            else
              ls = true;
            if(rightSpeed < 0)
              rs = true;
            else
              rs = false;
            Speedup(tmp1, ls, rs);
            
          break;
      case REV:
            
            if(leftSpeed <= -3 && rightSpeed <= -3) // if we are already at max going reverse do nothing
              break;
            if(leftSpeed > -3)
              leftSpeed--;
            if(rightSpeed > -3)
              rightSpeed--;
            on = 1;
            stop = 0;
            tmp1 = idle_speed;
            
            if(leftSpeed < 0)
              ls = false;
            else
              ls = true;
            if(rightSpeed < 0)
              rs = true;
            else
              rs = false;
            Speedup(tmp1, ls, rs);
            

        break;

      case RIGHT:
          if(leftSpeed >= 3 && rightSpeed <= -3) // if we are already at max going right do nothing
              break;
          if(leftSpeed < 3)
            leftSpeed++;
          if(rightSpeed > -3)
            rightSpeed--;
          on = 1;
          stop = 0;
          tmp1 = idle_speed;

          if(leftSpeed < 0)
            ls = false;
          else
            ls = true;
          if(rightSpeed < 0)
            rs = true;
          else
            rs = false;
          Speedup(tmp1, ls, rs);

        break;

      case LEFT:  
          if(rightSpeed >= 3 && leftSpeed <= -3) // if we are already at max going left do nothing
              break;
          if(rightSpeed < 3)
            rightSpeed++;
          if(leftSpeed > -3)
            leftSpeed--;
          on = 1;
          stop = 0;
          tmp1 = idle_speed;
            // passes in as left side , right side 
          if(leftSpeed < 0)
            ls = false;
          else
            ls = true;
          if(rightSpeed < 0)
            rs = true;
          else
            rs = false;
          Speedup(tmp1, ls, rs);
          digitalWrite(LED_BUILTIN, LOW);


        break;

      case FRONT:
        digitalWrite(LED_BUILTIN, HIGH);
        Shoulder(10, true, true);
        break;

      case BACK:

        digitalWrite(LED_BUILTIN, LOW);
        Shoulder(10, false, false);

        break;
      case RAISE:

        digitalWrite(LED_BUILTIN, HIGH);
        backshoulder(10,true,true);
        
        break;

      case LOWER:
        digitalWrite(LED_BUILTIN, HIGH);
        backshoulder(10,false,false);
        break;

      
      default: 
        digitalWrite(LED_BUILTIN, LOW);
        stop = 1;
        on = 0;
        Stop();
        break;
    }
  }
}


// Speed up function used to speed up the stepper motors from 0 to a set variable, takes in tmp as the starting variable for the speed up
void Speedup(long tmp, bool left, bool right)
{    
    Serial.print(tmp);

    // checks for max speed
    if(tmp >= maxSpeed)
      {
          on = 0;
          timeout = millis();
          tmp = maxSpeed;
          idle_speed = tmp;
          leftSide.set_direction(left);
          rightSide.set_direction(right);
          leftSide.set_speed(abs(tmp*leftSpeed));
          rightSide.set_speed(abs(tmp*rightSpeed));
          return;
      }
      // starts sppedup timer
      time = millis();  

      // set direction left for both motors on the left side and right for both motors on right side will be needed when they are connected to one wire

      leftSide.set_direction(left);
      rightSide.set_direction(right);

      // set speed

      leftSide.set_speed(abs(tmp*leftSpeed));
      rightSide.set_speed(abs(tmp*rightSpeed));


      tmp = tmp*2;

      idle_speed = tmp;
      
      Serial.println(tmp);

}
// Speed up function used to speed up the stepper motors from 0 to a set variable, takes in tmp as the starting variable for the speed up
void Shoulder(long tmp, bool left, bool right)
{    
    Serial.print(tmp);



      // set direction left for both motors on the left side and right for both motors on right side will be needed when they are connected to one wire

      frontShoulder.set_direction(left);
     // BackShoulder.set_direction(right);

      // set speed

      frontShoulder.set_speed(10);
      //BackShoulder.set_speed(10);



      
      Serial.println(tmp);

}
void backshoulder(long tmp, bool left, bool right)
{    
    Serial.print(tmp);



      // set direction left for both motors on the left side and right for both motors on right side will be needed when they are connected to one wire

      BackShoulder.set_direction(left);
      // BackShoulder.set_direction(right);

      // set speed

      BackShoulder.set_speed(10);
      //BackShoulder.set_speed(10);



      
      Serial.println(tmp);

}
// The stop function to be called to slowly stop the motors 
void Stop()
{    
    if(idle_speed <= 1)
    {
      idle_speed = 1;
      stop = 0;
      leftSpeed = 0;
      rightSpeed = 0;
      leftSide.set_speed(0);
      rightSide.set_speed(0);

      return;
    }
      time = millis();  
      Serial.println("Running Stop");

      idle_speed = idle_speed/2;

 
      leftSide.set_speed(idle_speed*leftSpeed);
      rightSide.set_speed(idle_speed*rightSpeed);

      
      Serial.println(idle_speed);

}
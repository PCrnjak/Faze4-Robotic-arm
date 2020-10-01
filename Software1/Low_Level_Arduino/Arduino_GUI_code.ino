/*************************************************************************
  By: Petar Crnjak
  version: 2.0
  For more info visit:
  https://github.com/PCrnjak/Faze4-Robotic-arm
**************************************************************************/

#include <Arduino.h>

/*****************************************************************************
  Define pins on pcb
  Defined by the name near the connector.
******************************************************************************/

// Joint6
// Port on PCB board P7
#define enable6 35    // port1  
#define dir6    36    // port2  
#define puls6   37    // port3  

// Joint5
//Port on PCB board P8 // On this connector pins for enable and puls are switched
// so our puls pin on board is connected to enable pin and our enable pin is connected to pulse pin
#define enable5 A21   // port6    
#define dir5    39    // port5  
#define puls5   38    // port4  

// Joint4
//
#define enable4 A22   // port7   analogWrite(A21, (int)val); rezolucija 12
#define dir4    13    // port8 // LED_BUILTIN  
#define puls4   14    // port9  

// Joint3
//Port on PCB board P12
#define enable3 15    // port10  
#define dir3    16    // port11  
#define puls3   17    // port12  

// Joint1
//Port on PCB board P16
#define enable1 20    // port13  
#define dir1    21    // port14  
#define puls1   22    // port15  

// Extra 1
//Port on PCB board P13
#define enable_ex1 2     // port16  
#define dir_ex1    1     // port17  
#define puls_ex1   0    // port18  

//Joint 2
//Port on PCB board P15
#define enable2 5     // port21  
#define dir2    4     // port20  
#define puls2   3     // port19  

// Extra 2
//Port on PCB board P14
#define enable_ex2 8     // port24  
#define dir_ex2    7     // port23  
#define puls_ex2   6     // port22  

//Port on PCB board P20
#define Gripper 23

//Inputs from left to right
#define limit2 28        // O1  
#define limit3 27        // O2  
#define limit4 26        // O3  
#define limit5 25        // O4  
#define limit_ex1 24     // O5  
#define limit_ex2 12     // O6  
#define limit_ex3 9      // O7   
#define limit1 10        // O8  
#define limit6 11        // O9   

// EXTRA PINS from top to bottom
// first is GND
#define ex1 29
#define ex2 30


const int EnablePin[] = {enable1, enable2, enable3, enable4, enable5, enable6}; // pins that enable stepper drivers

const int stepPin[] = {puls1, puls2, puls3, puls4, puls5, puls6}; // STEP pins on stepper drivers, first in array is joint 1, last i joint 6

const int dirPin[] = {dir1, dir2, dir3, dir4, dir5, dir6}; // DIR pins on stepper driver, first in array is joint 1 ,last is joint 6

const int LimitPin[] = {limit1, limit2, limit3, limit4, limit5, limit6}; // Pins for limit switches and inductive sensors

const int joint[7] = {0, 0, 1, 2, 3, 4, 5}; //// Calls---> Joint[1]=0 , Joint[2]=1...

const int meh_limit[] = {20000, 24000, 7000, 4000, 2600, 7700}; //Mechanical limits of joints.Represented in number of steps

bool zeroed[] = {0, 0, 0, 0, 0, 0}; // state variables that tell us if joint is zeroed or not

bool state[] = {0, 0, 0, 0, 0, 0}; // used in move_routine functions TODO test statis

unsigned long previousMillis[] = {0, 0, 0, 0, 0, 0};

int current_pulse_widht[] = {1000, 800, 1200, 900, 900, 900}; // current speed we are moving at

signed int current_position[] = {0, 0, 0, 0, 0, 0}; // our current position , never change this value manually after homing routine
signed int needed_position[] = {0, 0, 0, 0, 0, 0}; // position we want to go to

unsigned long currentMillis = 0;

volatile int error = 0; // Top level condition for robot to work , if 1 robot locks

uint16_t vars2[] = {0, 0, 0, 0, 0, 0};
/*****************************************************************************
  Extra UART ports
 ******************************************************************************/
// RX5 = 34
// TX5 = 33
// RX4 = 31
// TX4 = 32

/*****************************************************************************
   OLED PINS
 ******************************************************************************/
// SDA = 18
// SCL = 19

/*****************************************************************************
    Prototype for used functions
 ******************************************************************************/
void Homing_routine(int joint_num);
void homeAll();
void move_all();
void move_routine_backward(int joint_num);
void move_routine_forward(int joint_num);
void false_swtich_trigger_function1(void);
void false_swtich_trigger_function2(void);
void false_swtich_trigger_function3(void);
void false_swtich_trigger_function4(void);
void false_swtich_trigger_function5(void);
void false_swtich_trigger_function6(void);
void get_data();

void setup() {

  delay(100);

  pinMode(LimitPin[joint[1]], INPUT_PULLUP);
  pinMode(EnablePin[joint[1]], OUTPUT);
  pinMode(stepPin[joint[1]], OUTPUT);
  pinMode(dirPin[joint[1]], OUTPUT);
  digitalWrite(dirPin[joint[1]], HIGH);

  pinMode(LimitPin[joint[2]], INPUT_PULLDOWN);
  pinMode(EnablePin[joint[2]], OUTPUT);
  pinMode(stepPin[joint[2]], OUTPUT);
  pinMode(dirPin[joint[2]], OUTPUT);
  digitalWrite(dirPin[joint[2]], HIGH);

  pinMode(LimitPin[joint[3]], INPUT_PULLDOWN);
  pinMode(EnablePin[joint[3]], OUTPUT);
  pinMode(stepPin[joint[3]], OUTPUT);
  pinMode(dirPin[joint[3]], OUTPUT);
  digitalWrite(dirPin[joint[3]], HIGH);

  pinMode(LimitPin[joint[4]], INPUT_PULLDOWN);
  pinMode(EnablePin[joint[4]], OUTPUT);
  pinMode(stepPin[joint[4]], OUTPUT);
  pinMode(dirPin[joint[4]], OUTPUT);
  digitalWrite(dirPin[joint[4]], HIGH);

  pinMode(LimitPin[joint[5]], INPUT_PULLDOWN);
  pinMode(EnablePin[joint[5]], OUTPUT);
  pinMode(stepPin[joint[5]], OUTPUT);
  pinMode(dirPin[joint[5]], OUTPUT);
  digitalWrite(dirPin[joint[5]], HIGH);

  pinMode(LimitPin[joint[6]], INPUT_PULLUP);
  pinMode(EnablePin[joint[6]], OUTPUT);
  pinMode(stepPin[joint[6]], OUTPUT);
  pinMode(dirPin[joint[6]], OUTPUT);
  digitalWrite(dirPin[joint[6]], HIGH);

  attachInterrupt(limit1, false_swtich_trigger_function1, FALLING);
  attachInterrupt(limit2, false_swtich_trigger_function2, RISING);
  attachInterrupt(limit3, false_swtich_trigger_function3, RISING);
  attachInterrupt(limit4, false_swtich_trigger_function4, RISING);
  attachInterrupt(limit5, false_swtich_trigger_function5, RISING);
  attachInterrupt(limit6, false_swtich_trigger_function6, FALLING);

  delay(5000);

  homeAll();

  detachInterrupt(limit1);
  detachInterrupt(limit2);
  detachInterrupt(limit3);
  detachInterrupt(limit4);
  detachInterrupt(limit5);
  detachInterrupt(limit6);

  current_pulse_widht[joint[1]] = 800;
  current_pulse_widht[joint[2]] = 900;
  current_pulse_widht[joint[3]] = 1200;
  current_pulse_widht[joint[4]] = 900;
  current_pulse_widht[joint[5]] = 900;
  current_pulse_widht[joint[6]] = 900;

  Serial.begin(9600);
  //Serial4.begin(9600);
  delay(1000);
  //Serial4.println("Robot is starting , please wait");
  //Serial4.println("...");
  //Serial4.print("\n");

  delay(1000);

}

void loop() {

  while (error == 0) {

    get_data();
    move_all();
  }
}



void homeAll() {

  int flag = 0;
  /*****************************************************************************
    detach interrupts , if we dont do this robot will lock itself up
    ******************************************************************************/
  detachInterrupt(limit1);
  detachInterrupt(limit2);
  detachInterrupt(limit3);
  detachInterrupt(limit4);
  detachInterrupt(limit5);
  detachInterrupt(limit6);
  /*****************************************************************************
    Set speed for homing routines
  ******************************************************************************/
  current_pulse_widht[joint[1]] = 1500;
  current_pulse_widht[joint[2]] = 1500;
  current_pulse_widht[joint[3]] = 1500;
  current_pulse_widht[joint[4]] = 1500;
  current_pulse_widht[joint[5]] = 1500;
  current_pulse_widht[joint[6]] = 1500;
  /*****************************************************************************
      Enable all motors TODO joints that are on DACS
   ******************************************************************************/

  digitalWrite(EnablePin[joint[1]], LOW);
  delay(200);
  digitalWrite(EnablePin[joint[2]], LOW);
  delay(200);
  digitalWrite(EnablePin[joint[3]], LOW);
  delay(200);
  analogWrite(EnablePin[joint[5]], 0);
  delay(200);
  analogWrite(EnablePin[joint[4]], 0);
  delay(200);
  digitalWrite(EnablePin[joint[6]], LOW);

  /*****************************************************************************
       Set direction for all motors
  ******************************************************************************/
  digitalWrite(dirPin[joint[1]], HIGH);
  digitalWrite(dirPin[joint[2]], HIGH);
  digitalWrite(dirPin[joint[3]], HIGH);
  digitalWrite(dirPin[joint[4]], HIGH);
  digitalWrite(dirPin[joint[5]], HIGH);
  digitalWrite(dirPin[joint[6]], LOW);

  /*****************************************************************************
    Home joints 6 ,5 and 4
  ******************************************************************************/
  int i = 0;
  while ((zeroed[joint[6]] != 1)  or (zeroed[joint[5]] != 1) or (zeroed[joint[4]] != 1)) {

    Homing_routine(joint[4]);
    Homing_routine(joint[5]);
    Homing_routine(joint[6]);

    /*****************************************************************************
      Robot lock
    ******************************************************************************/
    while (flag == 1) {}

    /*****************************************************************************
       check if we are moving to much to home the motor
       if we do lock the robot
       ******************************************************************************/
    for (i = 4; i < 7; i++) {
      if ((current_position[joint[i]] + 50) >= meh_limit[joint[i]]) {
        Serial4.println("Error: It took too long for joint ");
        Serial4.print(i);
        Serial4.print(" s limit switch to trigger, restart your controller");
        flag = 1;
        break;

      }

    }

  }

  /*****************************************************************************
     Set current positions to 0 since we hit our homing switches
  ******************************************************************************/
  current_position[joint[6]] = 75;
  current_position[joint[5]] = 0;
  current_position[joint[4]] = 0;
  delay(500);
  /*****************************************************************************
    Now we will move from those limit switches for specific amounts
     We are doing this so that robot will never hit limit switches in normal operation
    In the same step we also move them to standby position              (this is position in kinematics
      diagram where all joints are in 0 position)
     We dont need to offset joint 6 since it is continous rotation joint
    We offset current position by :
    .  *            Joint 5 = offset 110 = new 0 , standby positon new = 1173
                Joint 4 = offset 110 = new 0 , standby positon new = 2275
  ******************************************************************************/
  needed_position[joint[6]] = 0;

  needed_position[joint[5]] = 1283;

  needed_position[joint[4]] = 2385;

  while (needed_position[joint[6]] != current_position[joint[6]] or needed_position[joint[5]] != current_position[joint[5]] or needed_position[joint[4]] != current_position[joint[4]] ) {
    move_all();

  }

  /*****************************************************************************
     These positons represent robot in its standby position
     This position would be 0 positions in robot kinematic diagram.
  ******************************************************************************/
  current_position[joint[6]] = 3838;
  needed_position[joint[6]] = 3838;

  current_position[joint[5]] = 1173;
  needed_position[joint[5]] = 1173;

  current_position[joint[4]] = 2275;
  needed_position[joint[4]] = 2275;

  /*****************************************************************************
       Home only joint 3
   ******************************************************************************/
  delay(200);
  while ((zeroed[joint[3]] != 1)) {
    Homing_routine(joint[3]);
    while (flag == 1) {}
    if ((current_position[joint[3]] + 50) >= meh_limit[joint[3]]) {
      Serial4.println("Error: It took too long for joint ");
      Serial4.print(3);
      Serial4.print(" s limit switch to trigger, restart your controller");
      flag = 1;
      break;

    }

  }

  /*****************************************************************************
    Here we only move from limit switch , and will later move with joint 2 to
     standby position
   ******************************************************************************/

  delay(100);
  current_position[joint[3]] = 0;
  needed_position[joint[3]] = 350;

  while (needed_position[joint[3]] != current_position[joint[3]]  ) {
    move_all();

  }
  current_position[joint[3]] = 0;
  needed_position[joint[3]] = 0;

  /*****************************************************************************
    Home only joint 2
  ******************************************************************************/
  delay(200);
  while ((zeroed[joint[2]] != 1)) {
    Homing_routine(joint[2]);
    while (flag == 1) {}
    if ((current_position[joint[2]] + 50) >= meh_limit[joint[2]]) {
      Serial4.println("Error: It took too long for joint ");
      Serial4.print(2);
      Serial4.print(" s limit switch to trigger, restart your controller");
      flag = 1;
      break;

    }

  }

  /*****************************************************************************
     Here we only move from limit switch , and will later move with joint 3 to
    standby position
  ******************************************************************************/

  delay(100);
  current_position[joint[2]] = 0;
  needed_position[joint[2]] = 700;

  while (needed_position[joint[2]] != current_position[joint[2]] ) {
    move_all();

  }

  current_position[joint[2]] = 0;
  needed_position[joint[2]] = 0;

  /*****************************************************************************
    Home only joint 1
  ******************************************************************************/

  delay(200);
  while ((zeroed[joint[1]] != 1)) {
    while (flag == 1) {}
    Homing_routine(joint[1]);
    if ((current_position[joint[1]] + 50) >= meh_limit[joint[1]]) {
      Serial4.println("Error: It took too long for joint ");
      Serial4.print(1);
      Serial4.print(" s limit switch to trigger, restart your controller");
      flag = 1;
      break;

    }

  }

  delay(100);
  current_position[joint[1]] = 10000;
  needed_position[joint[1]] = 9500; //10500
  delay(5);

  while (needed_position[joint[1]] != current_position[joint[1]]  ) {
    move_all();

  }

  current_position[joint[1]] = 10000;
  needed_position[joint[1]] = 10000;

  /*****************************************************************************
     Move joints 2 and 3 to standby positons
    .  ******************************************************************************/
  digitalWrite(dirPin[joint[2]], LOW);
  needed_position[joint[2]] = 10000 ;

  digitalWrite(dirPin[joint[3]], LOW);
  needed_position[joint[3]] = 2650;

  while (needed_position[joint[2]] != current_position[joint[2]] or needed_position[joint[3]] != current_position[joint[3]]) {
    move_all();

  }

  /*****************************************************************************
    Attach interrupts
    They should never activate in normal operation
    We do not attach interrupts for limit 1 and 6
  ******************************************************************************/

  //attachInterrupt(limit1, false_swtich_trigger_function1, FALLING);
  attachInterrupt(limit2, false_swtich_trigger_function2, RISING);
  attachInterrupt(limit3, false_swtich_trigger_function3, RISING);
  attachInterrupt(limit4, false_swtich_trigger_function4, RISING);
  attachInterrupt(limit5, false_swtich_trigger_function5, RISING);
  //attachInterrupt(limit6, false_swtich_trigger_function6, FALLING);

}

/*****************************************************************************
  Similar to move forward/backward but has extra steps
    to check if switches are pressed
******************************************************************************/
void Homing_routine(int joint_num) {
  if (digitalRead(LimitPin[joint_num]) == LOW) {
    currentMillis = micros();
    //// state is used to prevent triggering of this if statement twice in row
    ///// same goes for second one , this secures perfect square wave form
    if (currentMillis - previousMillis[joint_num] >= current_pulse_widht[joint_num] and state[joint_num] == 0) {
      previousMillis[joint_num] = currentMillis;
      digitalWrite(stepPin[joint_num], HIGH);
      state[joint_num] = 1;

    }
    else if (currentMillis - previousMillis[joint_num] >= current_pulse_widht[joint_num] and state[joint_num] == 1) {
      previousMillis[joint_num] = currentMillis;
      digitalWrite(stepPin[joint_num], LOW);
      state[joint_num] = 0;
      current_position[joint_num] = current_position[joint_num] + 1;


    }

  }
  if (digitalRead(LimitPin[joint_num]) == HIGH and zeroed[joint_num] == 0 ) {
    current_position[joint_num] = 0;
    Serial4.println("Joint ");
    Serial4.print(joint_num + 1);
    Serial4.println(" in 0 position.");
    zeroed[joint_num] = 1;

  }

}

/*****************************************************************************
  Function that checks if we are at desired position
    If we are not move until we are
 ******************************************************************************/
void move_all() {

  if (needed_position[joint[1]] > current_position[joint[1]] ) {
    digitalWrite(dirPin[joint[1]], LOW);
    move_routine_forward(joint[1]);

  }
  else if (needed_position[joint[1]] < current_position[joint[1]] ) {
    digitalWrite(dirPin[joint[1]], HIGH);
    move_routine_backward(joint[1]);

  }
  if (needed_position[joint[2]] > current_position[joint[2]] ) {
    digitalWrite(dirPin[joint[2]], LOW);
    move_routine_forward(joint[2]);

  }
  else if (needed_position[joint[2]] < current_position[joint[2]] ) {
    digitalWrite(dirPin[joint[2]], HIGH);
    move_routine_backward(joint[2]);

  }
  if (needed_position[joint[3]] > current_position[joint[3]]) {
    digitalWrite(dirPin[joint[3]], LOW);
    move_routine_forward(joint[3]);

  }
  else if (needed_position[joint[3]] < current_position[joint[3]] ) {
    digitalWrite(dirPin[joint[3]], HIGH);
    move_routine_backward(joint[3]);

  }
  if (needed_position[joint[4]] > current_position[joint[4]] ) {
    digitalWrite(dirPin[joint[4]], LOW);
    move_routine_forward(joint[4]);

  }
  else if (needed_position[joint[4]] < current_position[joint[4]] ) {
    digitalWrite(dirPin[joint[4]], HIGH);
    move_routine_backward(joint[4]);

  }
  if (needed_position[joint[5]] > current_position[joint[5]] ) {
    digitalWrite(dirPin[joint[5]], LOW);
    move_routine_forward(joint[5]);

  }
  else if (needed_position[joint[5]] < current_position[joint[5]]) {
    digitalWrite(dirPin[joint[5]], HIGH);
    move_routine_backward(joint[5]);

  }
  if (needed_position[joint[6]] > current_position[joint[6]] ) {
    digitalWrite(dirPin[joint[6]], HIGH);
    move_routine_forward(joint[6]);

  }
  else if (needed_position[joint[6]] < current_position[joint[6]] ) {
    digitalWrite(dirPin[joint[6]], LOW);
    move_routine_backward(joint[6]);

  }

}

/*****************************************************************************
    Routine to move forward
 ******************************************************************************/
void move_routine_forward(int joint_num) {
  currentMillis = micros();
  ///state is used to prevent triggering of this if statement twice in row
  // same goes for second one , this secures perfect square wave form
  if (currentMillis - previousMillis[joint_num] >= current_pulse_widht[joint_num] and state[joint_num] == 0) {
    previousMillis[joint_num] = currentMillis;
    digitalWrite(stepPin[joint_num], HIGH);
    state[joint_num] = 1;

  }

  else if (currentMillis - previousMillis[joint_num] >= (current_pulse_widht[joint_num]) and state[joint_num] == 1) {
    previousMillis[joint_num] = currentMillis;
    digitalWrite(stepPin[joint_num], LOW);
    state[joint_num] = 0;
    current_position[joint_num] = current_position[joint_num] + 1;

  }

}

/*****************************************************************************
   Routine to move backward
******************************************************************************/
void move_routine_backward(int joint_num) {
  currentMillis = micros();
  ////// state is used to prevent triggering of this if statement twice in row
  ///// same goes for second one , this secures perfect square wave form
  if (currentMillis - previousMillis[joint_num] >= current_pulse_widht[joint_num] and state[joint_num] == 0) {
    previousMillis[joint_num] = currentMillis;
    digitalWrite(stepPin[joint_num], HIGH);
    state[joint_num] = 1;

  }

  else if (currentMillis - previousMillis[joint_num] >= (current_pulse_widht[joint_num]) and state[joint_num] == 1) {
    previousMillis[joint_num] = currentMillis;
    digitalWrite(stepPin[joint_num], LOW);
    state[joint_num] = 0;
    current_position[joint_num] = current_position[joint_num] - 1;

  }

}

/*****************************************************************************
   If any switch is pressed during normal operation , disable robot
******************************************************************************/
void false_swtich_trigger_function1( ) {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 250ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 250)
  {
    error = 1;
    Serial4.println("");
    Serial4.print("Error: Joint 1 switch triggered unexpectedly");
  }
  last_interrupt_time = interrupt_time;

}

void false_swtich_trigger_function2( ) {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 250ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 250)
  {
    error = 1;
    Serial4.println("");
    Serial4.print("Error: Joint 2 switch triggered unexpectedly");
  }
  last_interrupt_time = interrupt_time;

}

void false_swtich_trigger_function3( ) {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 250ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 250)
  {
    error = 1;
    Serial4.println("");
    Serial4.print("Error: Joint 3 switch triggered unexpectedly");
  }
  last_interrupt_time = interrupt_time;

}

void false_swtich_trigger_function4( ) {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 250ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 250)
  {
    error = 1;
    Serial4.println("");
    Serial4.print("Error: Joint 4 switch triggered unexpectedly");
  }
  last_interrupt_time = interrupt_time;

}

void false_swtich_trigger_function5( ) {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 250ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 250)
  {
    error = 1;
    Serial4.println("");
    Serial4.print("Error: Joint 5 switch triggered unexpectedly");
  }
  last_interrupt_time = interrupt_time;

}

void false_swtich_trigger_function6( ) {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 250ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 250)
  {
    error = 1;
    Serial4.println("");
    Serial4.print("Error: Joint 6 switch triggered unexpectedly");
  }
  last_interrupt_time = interrupt_time;

}

void get_data() {

  static uint16_t vars[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  static int incomingByte = 0;
  static bool flag1 = 0;
  static bool flag2 = 0;
  static bool flag3 = 0;
  static int i = 0;

  if (Serial.available() > 0) {
    incomingByte = Serial.read();

    if ( flag2 == 1) {
      flag3 = 1;

    }
    if (incomingByte == 255 and flag1 == 1) {
      flag2 = 1;

    }
    if (incomingByte == 255) {
      flag1 = 1;

    }

    if (flag3 == 1) {
      vars[i] = incomingByte;
      i = i + 1;
      if (i == 12) {
        flag1 = 0;
        flag2 = 0;
        flag3 = 0;
        i = 0;

        vars2[0] = vars[0] << 8 | vars[1];
        vars2[1] = vars[2] << 8 | vars[3];
        vars2[2] = vars[4] << 8 | vars[5];
        vars2[3] = vars[6] << 8 | vars[7];
        vars2[4] = vars[8] << 8 | vars[9];
        vars2[5] = vars[10] << 8 | vars[11];

        needed_position[joint[1]] = vars2[joint[1]];
        needed_position[joint[2]] = vars2[joint[2]];
        needed_position[joint[3]] = vars2[joint[3]];
        needed_position[joint[4]] = vars2[joint[4]];
        needed_position[joint[5]] = vars2[joint[5]];
        needed_position[joint[6]] = vars2[joint[6]];
        /*
          Serial4.println(vars2[0]);
          Serial4.println(vars2[1]);
          Serial4.println(vars2[2]);
          Serial4.println(vars2[3]);
          Serial4.println(vars2[4]);
          Serial4.println(vars2[5]);
        */

      }

    }

  }

}

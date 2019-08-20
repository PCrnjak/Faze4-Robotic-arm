 /*
The MIT License

Copyright (c) 2019 Petar Crnjak

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.


Description: Faze4 
Software version: V1.0
Date: 10.5.2019
Programmer: Petar Crnjak



/* 
 *  I know this code is one hot mess . but it works.
 *  It is only used to test your arm dont use it for anything serious or rely to much on it.
 *  
 * Stepper serial monitor controller:
 * Can control multiple steppers simultaneously ,
 * and apply different acceleration and deceleration profiles to individual stepper
 *  
 *  This program was written for STM32F103C8 also known as Blue Pill
 *  Dont try to use this code on arduino uno and its brethren that run on 16 MHZ clock
 *  it will be too slow and unstable , some alternatives could be teensy , other boards from STM32 fammily, ESP8266, ESP32...
 *  Even tho code might not compile on those boards.
 *  
 *  
 *  
 *  
 */
#include <SPI.h>
#include <SD.h>
#include<math.h>


#define START PA12  // pin for start switch , robot will not start until you press this
#define Stepper_power PA15 /// pin that activates reley that turns on the supply for all steppers
#define E_Stop PB15 // Estop switch that stops whole arm


#define puls1 PB3
#define puls2 PB4
#define puls3 PB5
#define puls4 PB6
#define puls5 PB7
#define puls6 PB8

#define dir1 PB9
#define dir2 PB10
#define dir3 PB11
#define dir4 PB12
#define dir5 PB13
#define dir6 PB14

#define limit1 PA0
#define limit2 PA1
#define limit3 PA2
#define limit4 PA3
#define limit5 PC15
#define limit6 PC14

#define enable1 PB1
#define enable2 PB0
#define enable3 PA8
#define enable4 PA9
#define enable5 PA10
#define enable6 PA11

// 1 Free pin PC13

#define not_connected_2 PC13

String readString;
File myFile;
String temp="";

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

const int chipSelect = PA4;

/// Pins where you connect you sd card reader .
/// only use one without logic level shifter, like this one:
/// https://www.ebay.com/itm/1-piece-SD-Card-Module-Slot-Socket-Reader-For-Arduino-PIC-AVR-STM32-MSP430/264293725311?hash=item3d8922347f:g:okQAAOSwY7lcuBG3

//PA4 - SS (CS)
//PA5 - SCK1/ CLK
//PA6 - MISO1
//PA7 - MOSI1

int accel_steps_startup[] = {0,0,0,0,0,0};
int accel_steps_deacc[]= {0,0,0,0,0,0};


int record_positions[6][70]={};
// step angle of your stepper motor , usually 1.8 
float Step_angle[]={1.8,1.8,0.9,1.8,1.8,1.8};
//String readString;
const int microstep[]={2,8,2,2,2,2};// 2 is half step , 1 is full step, calcultated by ==> 1 / ( whatever microstepping tehnique you use)

const int EnablePin[]={enable1,enable2,enable3,enable4,enable5,enable6}; // pins that enable stepper drivers

const int stepPin[] = {puls1,puls2,puls3,puls4,puls5,puls6}; // STEP pins on stepper drivers, first in array is joint 1, last i joint 6

const int dirPin[] = {dir1,dir2,dir3,dir4,dir5,dir6};  // DIR pins on stepper driver, first in array is joint 1 ,last is joint 6

const int LimitPin[] = {limit1,limit2,limit3,limit4,limit5,limit6}; // LIMIT SWITCHES pins , first in array is joint 1 , last is joint 6

const int meh_limit[] = {20000,24000,7000,4000,4000,8000}; //Mechanical limits of joints.Represented in number of steps 

const int joint[7]={0,0,1,2,3,4,5};//// Calls---> Joint[1]=0 , Joint[2]=1...

/// sample_time is time in wich we change the speed of motor . 
/// if you for example set sample time to 20 it will calculate new speed every 20 seconds until it gets to desired speed
/// since it is int you cant enter any number lower then 1
/// smaller accel time will maybe need bigger sample time
int sample_time[]={20,20,20,20,20,20}; 
///Speed if move is really small and there is no time to accelerate 
const int small_pw[]={1000,1500,1500,1000,1200,800};
/// speed at wich steppers home
const int homo_pw[]={1000,1000,1000,1000,1000,1000};
/// speed if acceleration if disabled
const int no_accel[]={1000,1500,1000,1000,1000,500};
/// accel time is time in wich we want motor to accelerate from prev_pulse_widht to the pulse_widht
//(if prev_pulse_widht is large number (>7000) it is considered zero velocity) NEVER SET PREV_PULSE_WIDHT TO 0!!!!
/// all pulse widht values are linked to speed by this equation : n = 60 / ( 200 * microstep * 2 * pulse_widht *0.000001)
/////////////////////////////////////////////////////////////////////////////////////////////////////

float accel_time[]={1,1,1,0.6,0.4,0.4};
int accel[]={0,0,0,0,0,0};
//////////////////////////////////////////////////////////////////////////////////////////////////////

/// accel_steps are number of steps needed to accelerate from prev_pulse widht to the pulse_widht
/// calculated by this equation accel_steps[i]=(int)(((0.5*accel[i]*pow(accel_time[i],2)+prev_ang_vel[i]*accel_time[i]*0.01)*360)/(2*PI*0.9));
/// This value is just calculated and shown in table when you first start the code
///value that is actually used, is steps that mcu count up to the
// DIDNT USE THIS , calculated ones are really close to ones that it counts. 
/// value of needed angular velocity , you can check that value by uncommenting //Serial.print(accel_steps[joint_num]); in acceleration function
unsigned int accel_steps[]={0,0,0,0,0,0};
//constat calcultated in setup of program that is used alot in loop.
/// we use this  beacause in our formulas we use alot of constats like , microstep value, step angle... that  we dont want to re enter in our main loop every time and some of them are floats.
long const_s[]={0,0,0,0,0,0};

/////////////////////////////////////////////////////////////////////////////////////////////////////
///// 9000 == ZERO VELOCITY
const int prev_pulse_widht[]={9000,9000,9000,9000,9000,9000};  
int prev_ang_vel[]={0,0,0,0,0,0};
//////////////////////////////////////////////////////////////////////////////////////////////////////
//prev_ang_vel[1]=(int)(100000*PI)/(2*prev_pulse_widht[1]*microstep[1]);

//current_pulse_widht is used to store speed we are at during acceleration
int current_pulse_widht[]={0,0,0,0,0,0};
int current_ang_vel[]={0,0,0,0,0,0};

//////////////////////////////////////////////////////////////////////////////////////////////////////
/// maximum speeds for steppers
/// less then 300-400 is not good
const int pulse_widht[]={700,700,700,700,500,500}; //Pulse widht for stepper motors ( in microseconds) 
int ang_vel[]={0,0,0,0,0,0};
//////////////////////////////////////////////////////////////////////////////////////////////////////

unsigned long previousMillis[]={0,0,0,0,0,0};
unsigned long prevMillis_acc[]={0,0,0,0,0,0};
signed int current_position[]={0,0,0,0,0,0};
signed int needed_position[]={0,0,0,0,0,0};
signed int prev_position[]={0,0,0,0,0,0};
unsigned long currentMillis=0;
float reducer[]={15,27,15,11,11,19};

// different flags used in code
byte zeroed[]={0,0,0,0,0,0};
 // Condition for entering while loop to read next joint position.
byte  flag_done[]={1,1,1,1,1,1};
byte state[]={0,0,0,0,0,0};  
byte acc_dec[]={1,1,1,1,1,1}; // 1 if accelerating 
byte small_speed[]={0,0,0,0,0,0};
unsigned int brojac[] = {0,0,0,0,0,0};
//volatile int switch_false_trigger[]={0,0,0,0,0,0};
volatile int error = 0;
volatile int start_state = 0;
byte accel_off[] ={0,0,0,0,0,1} ; /// if 1 acceleraion is disabled
bool commands_flag=0;
bool recording = 1;
int position_counter=0;
bool loop_var=0;
int y_cnt=0;
int saved = 0; //if we are using file that is saved(1) or not(0)
String c_file; // string that tells us what is currently open file

unsigned int speed_step[]={0,0,0,0,0,0};
unsigned int sample2[]={0,0,0,0,0,0};

void setup(){
delay(2000);
Serial.begin(9600);

while (!Serial){}    

Serial.println("Robot is homing , please wait"); 
Serial.println("...");
Serial.print("\n");



//--------------------------------------------------------------------
///Set inputs and outputs
pinMode(START, INPUT_PULLDOWN);
pinMode(E_Stop, INPUT_PULLDOWN);
pinMode(Stepper_power,OUTPUT);

pinMode(LimitPin[joint[1]], INPUT_PULLUP);
pinMode(stepPin[joint[1]],OUTPUT); 
pinMode(dirPin[joint[1]],OUTPUT); 
digitalWrite(dirPin[joint[1]],HIGH);

pinMode(LimitPin[joint[2]], INPUT_PULLDOWN);
pinMode(stepPin[joint[2]],OUTPUT); 
pinMode(dirPin[joint[2]],OUTPUT); 
digitalWrite(dirPin[joint[2]],HIGH);

pinMode(LimitPin[joint[3]], INPUT_PULLDOWN);
pinMode(stepPin[joint[3]],OUTPUT); 
pinMode(dirPin[joint[3]],OUTPUT); 
digitalWrite(dirPin[joint[3]],HIGH);

pinMode(LimitPin[joint[4]], INPUT_PULLDOWN);
pinMode(stepPin[joint[4]],OUTPUT); 
pinMode(dirPin[joint[4]],OUTPUT); 
digitalWrite(dirPin[joint[4]],HIGH);

pinMode(LimitPin[joint[5]], INPUT_PULLDOWN);
pinMode(stepPin[joint[5]],OUTPUT); 
pinMode(dirPin[joint[5]],OUTPUT); 
digitalWrite(dirPin[joint[5]],HIGH);

pinMode(LimitPin[joint[6]], INPUT_PULLUP);
pinMode(stepPin[joint[6]],OUTPUT); 
pinMode(dirPin[joint[6]],OUTPUT); 
digitalWrite(dirPin[joint[6]],HIGH);
//--------------------------------------------------------------------



//--------------------------------------------------------------------
attachInterrupt(START,Start_func,RISING);
attachInterrupt(E_Stop,E_stop_func,RISING);
attachInterrupt(limit1, false_swtich_trigger_function1, FALLING);
attachInterrupt(limit2, false_swtich_trigger_function2, RISING);
attachInterrupt(limit3, false_swtich_trigger_function3, RISING);
attachInterrupt(limit4, false_swtich_trigger_function4, RISING);
attachInterrupt(limit5, false_swtich_trigger_function5, RISING);
attachInterrupt(limit6, false_swtich_trigger_function6, FALLING);
//--------------------------------------------------------------------

int speed_RPM;
int i =0;


/// speeds accelerations and everyting else here is before reducers , so be carefull
/// const_s , ang_vel_final,accel ,speed_increment are in real life actually divided by 100. Here they are multiplied by 100 to avoid using float.
Serial.print("                     const_s      ang_vel_final        accel    accel_steps    Speed_increment     Speed_RPM  ");
Serial.print("\n");
for(i=0;i<=5;i++){
Serial.print("joint"); Serial.print(i+1);  
Serial.print("\t");
Serial.print("\t");
const_s[i]=(2*PI*100000000)/((360/Step_angle[i])*microstep[i]*2);
Serial.print("\t");
Serial.print(const_s[i]);
Serial.print("\t");
Serial.print("\t");
ang_vel[i]=const_s[i]/(pulse_widht[i]); 
Serial.print(ang_vel[i]);

accel[i]=(ang_vel[i]-prev_ang_vel[i]) /(accel_time[i]);
Serial.print("\t");
Serial.print("\t");
Serial.print(accel[i]);
accel_steps[i]=((90*accel[i]*accel_time[i]*accel_time[i]*microstep[i])/(PI*Step_angle[i]*100));
Serial.print("\t");

Serial.print(accel_steps[i]);

current_pulse_widht[i]=prev_pulse_widht[i];
Serial.print("\t");
Serial.print("\t");
speed_step[i]=round(accel[i]*sample_time[i]*0.001);
Serial.print(speed_step[i]);
sample2[i]=sample_time[i]*1000;
Serial.print("\t");
Serial.print("\t");
Serial.print("\t");
Serial.println(((ang_vel[i]*30)/(314)));
}
Serial.print("\n");



//// Lost step compensation
/// here we simulate aceletarion and deaceleration of each motor by using dir pins as step outputs.
/// then we calculate how much steps we need to compensate in deaceleration to get a good looking curve
/// this makes our curve less accurate but stable
//--------------------------------------------------------------------

 i = 0;
for(i=0;i<=5;i++){
needed_position[i]= (accel_steps[i] * 2)+50;
//Serial.println(needed_position[i]);
}
Serial.println("Analyzing lost steps... please wait.");
recording = 0; 

while(needed_position[joint[1]]!=current_position[joint[1]] or needed_position[joint[2]]!=current_position[joint[2]] or needed_position[joint[3]]!=current_position[joint[3]]or 
needed_position[joint[4]]!=current_position[joint[4]]or needed_position[joint[5]]!=current_position[joint[5]] or needed_position[joint[6]]!=current_position[joint[6]]){

if(needed_position[joint[1]]>current_position[joint[1]] ){  
move_routine_startup(joint[1]);
}
if(needed_position[joint[2]]>current_position[joint[2]] ){  
move_routine_startup(joint[2]);
}
if(needed_position[joint[3]]>current_position[joint[3]] ){  
move_routine_startup(joint[3]);
 }
if(needed_position[joint[4]]>current_position[joint[4]] ){  
move_routine_startup(joint[4]);
}
if(needed_position[joint[5]]>current_position[joint[5]] ){  
move_routine_startup(joint[5]);
}
if(needed_position[joint[6]]>current_position[joint[6]] ){  
move_routine_startup(joint[6]);
}
 
}
//// TODO TEST 
recording = 1; 

/// sets some parametars for normal operation               
i = 0;                 
for(i=0;i<=5;i++){
current_position[i]=0;
acc_dec[i]=1;
needed_position[i]=0;
state[i]=1;
current_pulse_widht[i]=prev_pulse_widht[i];
accel_steps_deacc[i]=accel_steps[i]-accel_steps_startup[i]; /// steps needed to decelerate
Serial.println(accel_steps_deacc[i]);         
}
//--------------------------------------------------------------------




detachInterrupt(limit1);
detachInterrupt(limit2);
detachInterrupt(limit3);
detachInterrupt(limit4);
detachInterrupt(limit5);
detachInterrupt(limit6);

SD_status();
SD_init();
homeAll();


//// For emergency stop just stop the robot, dont remove power from the stepper drivers with their enable pins.
//// Not energized steppers will fall under the weight of the arm and probably break everyting .
//// Code like this does not even touch enable pins , since i didnt want to make that any kind of error ( false switch trigger, E-stop, or something else) 
//// to remove power from steppers. So if 

/*
///POWER ON PROCEDURE
while(start_state==0){ //// Until start switch is pressed robot will be trapped in this loop
//do nothing
}
start_state==0;
digitalWrite(Stepper_power,HIGH);

/*  
*/
//recording = 1; 
//standby_position();
}



//--------------------------------------------------------------------
///Main loop
void loop(){
//Allows to stop or pause robot when it is looping thru its positons
stop_pause();
//If any of the switches is activated after homing , this disables whole robot
while(error==0){

  // Move all joints at the same time
  move_all();

  //Allows to stop or pause robot when it is looping thru its positons
  stop_pause();
  
  //Check if we are at needed positons
  while(needed_position[joint[1]]==current_position[joint[1]] and needed_position[joint[2]]==current_position[joint[2]] and needed_position[joint[3]]==current_position[joint[3]]and 
  needed_position[joint[4]]==current_position[joint[4]]and needed_position[joint[5]]==current_position[joint[5]] and needed_position[joint[6]]==current_position[joint[6]]){
  if(loop_var==0){
   user_command_input();
    commands_flag=0;
  }
  
  // if robot is looping we dont need to input position values
  if(loop_var==0){

    //If doing individual tests comment this and add enter positons for one of joints
    enter_positions();


    Serial.println("Wait until the robot finishes");

    //If we are saving our positions 
    //--------------------------------------------------------------------
    if(recording==1){
      int x=0;
      for(x=0;x<6;x++){
            record_positions[x][position_counter]=needed_position[x];
            ///Serial.println(record_positions[x][position_counter]);
     }
      position_counter=position_counter+1;
      
    }
    //--------------------------------------------------------------------

   //--------------------------------------------------------------------
   //If we are looping
   }else if(loop_var==1){
       // Serial.println("LOOOPSI");
       if(y_cnt==position_counter){
        y_cnt=0;
       }
      if(y_cnt<position_counter){
        int x=0;
           for(x=0;x<6;x++){
              needed_position[x]=record_positions[x][y_cnt];   
              prev_position[x]= current_position[x];
              current_pulse_widht[x]=prev_pulse_widht[x];
              brojac[x]=0;
              acc_dec[x]=1;
              
              if(abs(current_position[x]-needed_position[x])<=2*accel_steps[x]){
                 small_speed[x]=1;
                   }else{
                 small_speed[x]=0;
                   }
  
              if(needed_position[x]>current_position[x] ){  
                 digitalWrite(dirPin[x],LOW);
                      }
              else if(needed_position[x]<current_position[x] ){
                  digitalWrite(dirPin[x],HIGH);
                      }  
                
         }
         
     y_cnt=y_cnt+1;

         }
       }
     }
   }
}

//--------------------------------------------------------------------



//--------------------------------------------------------------------
///Homing routine for individual joints
void Homing_routine(int joint_num){
//Serial.print(joint_num);
if(digitalRead(LimitPin[joint_num])==LOW){
 currentMillis=micros();
 ////// state is used to prevent triggering of this if statement twice in row 
 ///// same goes for second one , this secures perfect square wave form
 if(currentMillis-previousMillis[joint_num]>=homo_pw[joint_num] and state[joint_num]==0){
   previousMillis[joint_num]=currentMillis;
   digitalWrite(stepPin[joint_num],HIGH);
   state[joint_num]=1;  
  }
 else if(currentMillis-previousMillis[joint_num]>=homo_pw[joint_num] and state[joint_num]==1) {
   previousMillis[joint_num]=currentMillis;
   digitalWrite(stepPin[joint_num],LOW);
   state[joint_num]=0;
   current_position[joint_num]=current_position[joint_num] + 1;
    
  }
}
if(digitalRead(LimitPin[joint_num])==HIGH and zeroed[joint_num]==0 ){
  current_position[joint_num]=0;
  Serial.println("Joint ");
  Serial.print(joint_num + 1);
  Serial.println(" in 0 position.");
  zeroed[joint_num]=1;
 }
}
//--------------------------------------------------------------------



//--------------------------------------------------------------------
//--------------------------------------------------------------------
//Homing with some aditional steps
void homeAll(){
int flag=0;

//detach interrupts , if we dont do this robot will lock itself up 
detachInterrupt(limit1);
detachInterrupt(limit2);
detachInterrupt(limit3);
detachInterrupt(limit4);
detachInterrupt(limit5);
detachInterrupt(limit6);

//Enable all motors
digitalWrite(EnablePin[joint[1]],HIGH);
delay(200);
digitalWrite(EnablePin[joint[2]],HIGH);
delay(200);
digitalWrite(EnablePin[joint[3]],HIGH);
delay(200);
digitalWrite(EnablePin[joint[4]],HIGH);
delay(200);
digitalWrite(EnablePin[joint[5]],HIGH);
delay(200);
digitalWrite(EnablePin[joint[6]],HIGH);

//Set direction
digitalWrite(dirPin[joint[1]],HIGH);
digitalWrite(dirPin[joint[2]],HIGH);
digitalWrite(dirPin[joint[3]],HIGH);
digitalWrite(dirPin[joint[4]],HIGH);
digitalWrite(dirPin[joint[5]],HIGH);
digitalWrite(dirPin[joint[6]],HIGH);

//Home all motors
int i=0;
while((zeroed[joint[6]]!=1)  or (zeroed[joint[5]]!=1) or (zeroed[joint[4]]!=1)){
Homing_routine(joint[4]);
Homing_routine(joint[5]);
Homing_routine(joint[6]);

//check if we are moving to much to home the motor
//if we do lock the robot
for(i=4;i<7;i++){
if((current_position[joint[i]]+50)>=meh_limit[joint[i]]){
  Serial.println("Error: It took too long for joint ");
  Serial.print(i);
  Serial.print(" s limit switch to trigger, restart your controller");
  flag=1;  
  break;
}
}


}

//Robot lock
while(flag==1){
//Serial.println("Doing nothing , restart");
//delay(1000);
}

current_position[joint[6]]=0;
current_position[joint[5]]=0;
current_position[joint[4]]=0;
delay(500);


//--------------------------------------------------------------------
/// Zero positons and offsets 

digitalWrite(dirPin[joint[6]],HIGH);
needed_position[joint[6]]=75;

digitalWrite(dirPin[joint[5]],LOW);
needed_position[joint[5]]=1283;  /// zero position offset is 110

digitalWrite(dirPin[joint[4]],LOW);
needed_position[joint[4]]=2385;  /// zero position offset is 110 ,zero_position = 2275 + 110(offset)


while(needed_position[joint[6]]!=current_position[joint[6]] or needed_position[joint[5]]!=current_position[joint[5]] or needed_position[joint[4]]!=current_position[joint[4]] ){

  move_all();

}

current_position[joint[6]]=4000;
needed_position[joint[6]]=4000;

current_position[joint[5]]=1173;
needed_position[joint[5]]=1173;

current_position[joint[4]]=2275;
needed_position[joint[4]]=2275;

//--------------------------------------------------------------------


//Home only J3 
//--------------------------------------------------------------------
delay(200);

while((zeroed[joint[3]]!=1)){
Homing_routine(joint[3]);

if((current_position[joint[3]]+50)>=meh_limit[joint[3]]){
  Serial.println("Error: It took too long for joint ");
  Serial.print(3);
  Serial.print(" s limit switch to trigger, restart your controller");
  flag=1;  
  break;
}
}

//Robot lock
while(flag==1){
//Serial.println("Doing nothing , restart");
//delay(1000);
}

delay(100);
current_position[joint[3]]=0;
digitalWrite(dirPin[joint[3]],LOW);
needed_position[joint[3]]=350;

while(needed_position[joint[3]]!=current_position[joint[3]]  ){
move_all();
}
current_position[joint[3]]=0;
needed_position[joint[3]]=0;
//--------------------------------------------------------------------



//Home only J2
//--------------------------------------------------------------------
delay(200);
while((zeroed[joint[2]]!=1)){

Homing_routine(joint[2]);
if((current_position[joint[2]]+50)>=meh_limit[joint[2]]){
  Serial.println("Error: It took too long for joint ");
  Serial.print(2);
  Serial.print(" s limit switch to trigger, restart your controller");
  flag=1;  
  break;
}

}

//Robot lock
while(flag==1){
//Serial.println("Doing nothing , restart");
//delay(1000);
}

accel_off[1]=1;
delay(100);
current_position[joint[2]]=0;
digitalWrite(dirPin[joint[2]],LOW);
needed_position[joint[2]]=700;

while(needed_position[joint[2]]!=current_position[joint[2]] ){
move_all();
}
accel_off[1]=0;
current_position[joint[2]]=0;
needed_position[joint[2]]=0;
//--------------------------------------------------------------------


//Home only J1
//--------------------------------------------------------------------
delay(200);
while((zeroed[joint[1]]!=1)){

Homing_routine(joint[1]);
if((current_position[joint[1]]+50)>=meh_limit[joint[1]]){
  Serial.println("Error: It took too long for joint ");
  Serial.print(1);
  Serial.print(" s limit switch to trigger, restart your controller");
  flag=1;  
  break;
}

}

//Robot lock
while(flag==1){
//Serial.println("Doing nothing , restart");
//delay(1000);
}

delay(100);
current_position[joint[1]]=10000;
digitalWrite(dirPin[joint[1]],HIGH);
needed_position[joint[1]]=10500;


while(needed_position[joint[1]]!=current_position[joint[1]]  ){
move_all();
}

current_position[joint[1]]=10000;
needed_position[joint[1]]=10000;


//--------------------------------------------------------------------



// move robot away from limit switches and set those positons 
// as new zero points , meaning it never should press switches in normal operation



///Attach interrupts
//They should never activate in normal operation
//attachInterrupt(limit1, false_swtich_trigger_function1, FALLING);
attachInterrupt(limit2, false_swtich_trigger_function2, RISING);
attachInterrupt(limit3, false_swtich_trigger_function3, RISING);
attachInterrupt(limit4, false_swtich_trigger_function4, RISING);
attachInterrupt(limit5, false_swtich_trigger_function5, RISING);
//attachInterrupt(limit6, false_swtich_trigger_function6, FALLING);



prev_position[joint[1]]=0;
prev_position[joint[2]]=0;
prev_position[joint[3]]=0;
prev_position[joint[4]]=0;
prev_position[joint[5]]=0;
prev_position[joint[6]]=0;

}
//--------------------------------------------------------------------
//--------------------------------------------------------------------




//--------------------------------------------------------------------
//Moves the robot to some predefined position(Zero positon defined by kinematic diagram)
void standby_position(){
///////////////////////////////////////////////////
//recording = 0; 
///////////////////////////////////////////////////
needed_position[joint[1]]=10000;
needed_position[joint[2]]=9650;
needed_position[joint[3]]=5600;
needed_position[joint[4]]=2275;
needed_position[joint[5]]=1173;
needed_position[joint[6]]=4000;


//Set direction

int x=0;
for(x=0;x<6;x++){
if(needed_position[x]>current_position[x] ){  
                 digitalWrite(dirPin[x],LOW);
                      }
               if(needed_position[x]<current_position[x] ){
                  digitalWrite(dirPin[x],HIGH);
                      }  
}

                     
while(needed_position[joint[1]]!=current_position[joint[1]] or needed_position[joint[2]]!=current_position[joint[2]] or needed_position[joint[3]]!=current_position[joint[3]]or 
needed_position[joint[4]]!=current_position[joint[4]]or needed_position[joint[5]]!=current_position[joint[5]] or needed_position[joint[6]]!=current_position[joint[6]]){

move_all(); 
}


 if(recording==1){
      int x=0;
      for(x=0;x<6;x++){
            record_positions[x][position_counter]=needed_position[x];
            ///Serial.println(record_positions[x][position_counter]);
     }
      position_counter=position_counter+1;
      
    }
/*
if(recording==1){
int x=0 ;
for(x=0;x<6;x++){
record_positions[x][position_counter]=current_position[x];
 prev_position[x]= current_position[x];
 current_pulse_widht[x]=prev_pulse_widht[x] ;
 brojac[x]=0;
 acc_dec[x]=1;
}
position_counter=position_counter+1;
}

*/

recording = 1;

}
//--------------------------------------------------------------------



//--------------------------------------------------------------------
//Routine to move forward
void move_routine_forward(int joint_num){
 currentMillis=micros();
 ////// state is used to prevent triggering of this if statement twice in row 
 ///// same goes for second one , this secures perfect square wave form
 
 if(currentMillis-previousMillis[joint_num]>=current_pulse_widht[joint_num] and state[joint_num]==0){
    previousMillis[joint_num]=currentMillis;
    digitalWrite(stepPin[joint_num],HIGH);
    state[joint_num]=1;  
 }
 
 else if(currentMillis-previousMillis[joint_num]>=(current_pulse_widht[joint_num]) and state[joint_num]==1) {
    previousMillis[joint_num]=currentMillis;
    digitalWrite(stepPin[joint_num],LOW);
    state[joint_num]=0;
    current_position[joint_num]=current_position[joint_num] + 1;

 } 
 acceleration(joint_num);
 
}
//--------------------------------------------------------------------



//--------------------------------------------------------------------
//Routine to move backward
void move_routine_backward(int joint_num){
 currentMillis=micros();
 ////// state is used to prevent triggering of this if statement twice in row 
 ///// same goes for second one , this secures perfect square wave form
 
  if(currentMillis-previousMillis[joint_num]>=current_pulse_widht[joint_num] and state[joint_num]==0){
    previousMillis[joint_num]=currentMillis;
    digitalWrite(stepPin[joint_num],HIGH);
    state[joint_num]=1;  
  }
  
 else if(currentMillis-previousMillis[joint_num]>=(current_pulse_widht[joint_num]) and state[joint_num]==1) {
    previousMillis[joint_num]=currentMillis;
    digitalWrite(stepPin[joint_num],LOW);
    state[joint_num]=0;
    current_position[joint_num]=current_position[joint_num] - 1;
  
  }
acceleration(joint_num);
   
}
//--------------------------------------------------------------------



//--------------------------------------------------------------------
// Routine to get next position of joint
void get_next_position(int joint_num){
  
  while (Serial.available() and flag_done[joint_num]==1) {
    char c = Serial.read();  //gets one byte from serial buffer
    readString += c; //makes the string readString
    delay(2);  //slow looping to allow buffer to fill with next character
  }

  
  readString.trim();


  if (readString.length() >0) {
            ///Serial.println(readString);  //so we can see ze string
             Serial.println("");
            // Serial.print(readString);
            //Serial.print(current_position[joint_num]);
            
            if(readString == "p"){
              needed_position[joint_num]= current_position[joint_num];
            }else {
             needed_position[joint_num] = readString.toInt(); // readString into a number
            }
            
            if(needed_position[joint_num]>=0 and needed_position[joint_num]<=meh_limit[joint_num]){
        
            Serial.print("joint ");
            Serial.print(joint_num + 1); 
            Serial.print(" is going to : ");
            Serial.println(needed_position[joint_num]);
            Serial.println("");
            readString=""; //empty for next input
            
            flag_done[joint_num]=0;
            prev_position[joint_num]= current_position[joint_num];
            current_pulse_widht[joint_num]=prev_pulse_widht[joint_num] ;
            brojac[joint_num]=0;
            acc_dec[joint_num]=1;
            if(abs(current_position[joint_num]-needed_position[joint_num])<=2*accel_steps[joint_num]){
               small_speed[joint_num]=1;
              }else{
               small_speed[joint_num]=0;
             }
        
               
        if(needed_position[joint_num]>current_position[joint_num] ){  
          digitalWrite(dirPin[joint_num],LOW);
        }

        
        if(needed_position[joint_num]<current_position[joint_num] ){
          digitalWrite(dirPin[joint_num],HIGH);
         /// move_routine_backward(joint[1]);
         }  
          
             }else {
           needed_position[joint_num]=current_position[joint_num];
           flag_done[joint_num]=1;
           Serial.print("you entered invalid position. ");
           Serial.print("please enter position from 0 to 4000 ");
           readString=""; //empty for next input 
    }
  } 
}
//--------------------------------------------------------------------



//--------------------------------------------------------------------
// Aceleration
void acceleration(int joint_num){
if(accel_off[joint_num]==0){
  if(small_speed[joint_num]==0){
    if( acc_dec[joint_num]==1 ){
      currentMillis=micros();
      if(currentMillis-prevMillis_acc[joint_num]>=sample2[joint_num]){
        
        prevMillis_acc[joint_num]=currentMillis;
               
          current_ang_vel[joint_num]=const_s[joint_num]/current_pulse_widht[joint_num]+speed_step[joint_num];       
          current_pulse_widht[joint_num]=const_s[joint_num]/current_ang_vel[joint_num];
          if(current_pulse_widht[joint_num]<pulse_widht[joint_num]){
           /// accel_steps[joint_num]=abs(current_position[joint_num] - prev_position[joint_num]);
            current_pulse_widht[joint_num]=pulse_widht[joint_num];
            acc_dec[joint_num]=0;  
          
            
            //Serial.println("");
           
            
    }
   
  }
  
 }else if(abs(current_position[joint_num]-needed_position[joint_num])<=accel_steps_deacc[joint_num] and acc_dec[joint_num]==0){ 
    currentMillis=micros();
    if(currentMillis-prevMillis_acc[joint_num]>=sample2[joint_num]){
      prevMillis_acc[joint_num]=currentMillis;


      
      current_ang_vel[joint_num]=const_s[joint_num]/current_pulse_widht[joint_num]-speed_step[joint_num];
      current_pulse_widht[joint_num]=const_s[joint_num]/current_ang_vel[joint_num];
      
  }
  
  if(current_pulse_widht[joint_num]>=10000){
    current_pulse_widht[joint_num]=10000;
  }
   

 }
 }else{
  current_pulse_widht[joint_num]=small_pw[joint_num]; 
}

}else{
  current_pulse_widht[joint_num]=no_accel[joint_num];  
}
}
//--------------------------------------------------------------------



//--------------------------------------------------------------------
////False trigger warnings, if any of these triggers robot will be disabled until you reset the controller
void false_swtich_trigger_function1( ){
static unsigned long last_interrupt_time = 0;
 unsigned long interrupt_time = millis();
 // If interrupts come faster than 250ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > 250) 
 {
 error=1;
Serial.println("");
Serial.print("Error: Joint 1 switch triggered unexpectedly");
 }
 last_interrupt_time = interrupt_time;
}

void false_swtich_trigger_function2( ){
static unsigned long last_interrupt_time = 0;
 unsigned long interrupt_time = millis();
 // If interrupts come faster than 250ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > 250) 
 {
 error=1;
Serial.println("");
Serial.print("Error: Joint 2 switch triggered unexpectedly");
 }
 last_interrupt_time = interrupt_time;
}

void false_swtich_trigger_function3( ){
static unsigned long last_interrupt_time = 0;
 unsigned long interrupt_time = millis();
 // If interrupts come faster than 250ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > 250) 
 {
 error=1;
Serial.println("");
Serial.print("Error: Joint 3 switch triggered unexpectedly");
 }
 last_interrupt_time = interrupt_time;
}

void false_swtich_trigger_function4( ){
static unsigned long last_interrupt_time = 0;
 unsigned long interrupt_time = millis();
 // If interrupts come faster than 250ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > 250) 
 {
 error=1;
Serial.println("");
Serial.print("Error: Joint 4 switch triggered unexpectedly");
 }
 last_interrupt_time = interrupt_time;
}

void false_swtich_trigger_function5( ){
static unsigned long last_interrupt_time = 0;
 unsigned long interrupt_time = millis();
 // If interrupts come faster than 250ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > 250) 
 {
 error=1;
Serial.println("");
Serial.print("Error: Joint 5 switch triggered unexpectedly");
 }
 last_interrupt_time = interrupt_time;
}

void false_swtich_trigger_function6( ){
static unsigned long last_interrupt_time = 0;
 unsigned long interrupt_time = millis();
 // If interrupts come faster than 250ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > 250) 
 {
 error=1;
Serial.println("");
Serial.print("Error: Joint 6 switch triggered unexpectedly");
 }
 last_interrupt_time = interrupt_time;
}

void E_stop_func( ){
static unsigned long last_interrupt_time = 0;
 unsigned long interrupt_time = millis();
 // If interrupts come faster than 250ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > 250) 
 {
 error=1;
Serial.println("");
Serial.print("Error: E-Stio triggered");
 }
 last_interrupt_time = interrupt_time;
}




void Start_func(){
static unsigned long last_interrupt_time = 0;
 unsigned long interrupt_time = millis();
 // If interrupts come faster than 250ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > 250) 
 {
start_state=1;
 
 }
 last_interrupt_time = interrupt_time;
}

//--------------------------------------------------------------------




//--------------------------------------------------------------------
void move_all(){
  
 if(needed_position[joint[1]]>current_position[joint[1]] ){  
    move_routine_forward(joint[1]);
   }
  else if(needed_position[joint[1]]<current_position[joint[1]] ){
    move_routine_backward(joint[1]);
   }
  if(needed_position[joint[2]]>current_position[joint[2]] ){   
    move_routine_forward(joint[2]);
   }
  else if(needed_position[joint[2]]<current_position[joint[2]] ){
    move_routine_backward(joint[2]);
   }
   if(needed_position[joint[3]]>current_position[joint[3]]){  
    move_routine_forward(joint[3]);
   }
  else if(needed_position[joint[3]]<current_position[joint[3]] ){
    move_routine_backward(joint[3]);
   }
  if(needed_position[joint[4]]>current_position[joint[4]] ){   
    move_routine_forward(joint[4]);
   }
  else if(needed_position[joint[4]]<current_position[joint[4]] ){
    move_routine_backward(joint[4]);
   }
   if(needed_position[joint[5]]>current_position[joint[5]] ){  
    move_routine_forward(joint[5]);
   }
  else if(needed_position[joint[5]]<current_position[joint[5]]){
    move_routine_backward(joint[5]);
   }
  if(needed_position[joint[6]]>current_position[joint[6]] ){   
    move_routine_forward(joint[6]);
   }
  else if(needed_position[joint[6]]<current_position[joint[6]] ){
    move_routine_backward(joint[6]);
   }
}
//--------------------------------------------------------------------




//--------------------------------------------------------------------
//Allows you to stop, pause or view commands while robot is moving
void stop_pause(){

  if(Serial.available()){
    while (Serial.available()) {
      //delay(1);  
      char c = Serial.read();
      readString += c; 
   }
   readString.trim();
   if (readString.length() >0) {
    readString.toUpperCase();
    if (readString == "STOP"){
      Serial.println("STOP");
      loop_var=0;
    }
    if (readString == "PAUSE"){
      Serial.println("PAUSE");   
      error=1;   
    }

    if(error==1){
      if(readString == "COMMANDS"){
        commands();
      }
    }

    if(readString == "GO"){
      Serial.print("GO");
      error=0;
    }else{
      if (readString.length() >0) {
      Serial.print(readString);
      Serial.print("   is not valid command");
      Serial.println("");
      readString="";
      }
    }
   
   }
   readString=""; 
  }
}
//--------------------------------------------------------------------




//--------------------------------------------------------------------
//Commands that are available 
void user_command_input(){
  
 Serial.println("Enter next command: "); 
   while(commands_flag==0){
    while (Serial.available()) {
    delay(3);  
    char c = Serial.read();
    readString += c; 
  }
  readString.trim();
  if (readString.length() >0) {
    
 // Serial.println(readString);
  readString.toUpperCase();
 // Serial.println(readString);
  
    if (readString == "RECON"){
      Serial.println("Recon");
      readString="";
      recording = 1;
      commands_flag=0; 
      Serial.println("Enter next command: "); 
    }
    if (readString == "RECOFF"){
      Serial.println("Recoff");  
      readString="";    
      recording = 0;
      commands_flag=0; 
       Serial.println("Enter next command: "); 
    }
    if (readString == "DEL"){
      Serial.println("Del");
      readString="";
      DEL();
      commands_flag=0; 
      Serial.println("Enter next command: "); 
    }
    if (readString == "DELALL"){
      Serial.println("Delall");  
      readString="";  
      DEL_ALL();
      commands_flag=0;   
      Serial.println("Enter next command: "); 
    }
    if (readString == "COMMANDS"){
      Serial.println("Commands");
      readString="";
      commands();
      commands_flag=0; 
      Serial.println("Enter next command: "); 
    }

    if (readString == "STANDBY"){
      Serial.println("STANDBY");
      readString="";
      standby_position();
      commands_flag=0; 
      Serial.println("Enter next command: "); 
    }
    
    if (readString == "HOME"){
          // homeAll(); 
           commands_flag=0; 
           readString="";
           Serial.println("Enter next command: "); 
    }

    if (readString == "OPEN"){
           // homeAll(); 
            commands_flag=0; 
            SD_open();
            readString="";
            Serial.println("Enter next command: "); 
    }

    if (readString == "CLOSE"){
           // homeAll(); 
            commands_flag=0; 
            SD_close();
            readString="";
            Serial.println("Enter next command: "); 
    }

    if (readString == "SAVE"){
            SD_save_file();
            commands_flag=0; 
            readString="";
            Serial.println("Enter next command: "); 
    }
    
    if (readString == "NEXT"){
      Serial.println("Next");
      commands_flag=1;
      readString="";
    }
    if (readString == "LOOP"){
      Serial.println("Loop");    
      readString="";  
      loop_var=1;
      commands_flag=1;
    }else{
      if (readString.length() >0) {
      Serial.print(readString);
      Serial.print("   is not valid command");
      Serial.println("");
      readString="";
      Serial.println("Enter next command: "); 
      }
    }

    readString="";
  }

   }
  
}
//--------------------------------------------------------------------



//--------------------------------------------------------------------
//Prints all available commands, recorded positions,and other info
void commands(){
  if(c_file.length()==0){
    Serial.println("No file is opened.");
  }else{
    Serial.println("");
    Serial.print("Current file is : "); 
    Serial.println(c_file);
  
  }


 Serial.println(" Joint1  Joint2  Joint3  Joint4  Joint5  Joint6  ");
for(int y=0;y<position_counter;y++){
  Serial.println("");
  for(int x=0;x<6;x++){
    Serial.print(record_positions[x][y]);
    Serial.print("\t");
  }
}

  Serial.println("");
  Serial.print("Recorded positions: ");
  Serial.print(position_counter);
  Serial.println("");
  Serial.println("COMMANDS--Prints all available commands");
  Serial.println("RECON---Starts recording positions of robotic arm");
  Serial.println("RECOFF--Stops recording positions of robotic arm");
  Serial.println("DEL-----Deletes previous position. Can be used multiple times in row to delete more moves");
  Serial.println("DELALL--Deletes all recorded positions");
  Serial.println("HOME----Executes homing routine");
  Serial.println("NEXT----allows you to enter next positions for robot, entering p insted of number will set position to previous one");
  Serial.println("LOOP----Loops thru all recorded positions");
  Serial.println("STOP----Stops robot if it is moving , but after it executes current move.Also it exits loop command.");
  Serial.println("PAUSE---Stops robots movements immediately");
  Serial.println("GO------If paused robot starts moving from paused position");
  Serial.println("STANDBY-Robot moves to standby position");
  ////////////////////////////
  Serial.println("SAVE----allows you to save all moves made in a file");
  Serial.println("OPEN--allows you to select movement file saved on sd card");
  Serial.println("CLOSE--allows you to close current SD file");

  SD_status();
  
}
//--------------------------------------------------------------------



void DEL(){
  position_counter=position_counter-1;
  Serial.println("");
  Serial.println("Recorded positions: ");
  Serial.print(position_counter);
}

void DEL_ALL(){
  position_counter=0;
  Serial.println("");
  Serial.println("Recorded positions: ");
  Serial.print(position_counter);
}



//--------------------------------------------------------------------
///Gets status of SD card and info about it
void SD_status(){
   Serial.print("\nInitializing SD card...");

  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    return;
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }

  // print the type of card
  Serial.print("\nCard type: ");
  switch (card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    return;
  }


  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("\nVolume type is FAT");
  Serial.println(volume.fatType(), DEC);
  Serial.println();

  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize *= 512;                            // SD card blocks are always 512 bytes
  Serial.print("Volume size (bytes): ");
  Serial.println(volumesize);
  Serial.print("Volume size (Kbytes): ");
  volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print("Volume size (Mbytes): ");
  volumesize /= 1024;
  Serial.println(volumesize);


  Serial.println("\nFiles found on the card (name, date and size in bytes): ");
  root.openRoot(volume);

  // list all files in the card with date and size
  root.ls(LS_R | LS_DATE | LS_SIZE);
  Serial.print("Initializing SD card...");
 
}

//--------------------------------------------------------------------




//--------------------------------------------------------------------
//// SAVES THE FILE THAT IS OPENED AT THIS MOMENT , IF NO FILE IS OPENED 
//// PROMPTS YOU TO ENTER A NAME FOR NEW FILE TO BE SAVED
void SD_save_file(){

if(saved == 1){ /// if we are saving file that is saved on SD card
  char filename[c_file.length()+1];
  c_file.toCharArray(filename, sizeof(filename));
  myFile.close();
  SD_status();
  /// we remove all positions from this file by removing it and then recreating it
  /// All positons are saved in buffer when we open the file( you can only save the file if it is opened)
  //SD.begin();
  SD.remove(filename);// remove is a function in SD library to delete a file
  SD_status();
  if (SD.exists(filename)) { /// check if file exists when we removed it 
    Serial.println("Error");
  } else { //If the file was successfully deleted display message that the file was already deleted.
    Serial.println("File was removed .");
  }
  if(!SD.exists(filename)){ // if file does not exist create it 
  myFile = SD.open(filename,FILE_WRITE); // SD.open expects char array so we need to convert string to chara array  
 }
 SD_status();
  
  
 /// myFile = SD.open(filename, FILE_WRITE);

buffer_to_SD();

/////If the file we are saving does not exist
}else{
 
byte good=0;
Serial.println("Enter the name of the file:");
readString="";
while(good==0){
while (Serial.available()) {
    delay(3);  
    char c = Serial.read();
    readString += c; 
  }
  readString.trim();
  if (readString.length() >0) {
   temp= String(readString);
   // temp=readString;
    good = 1;
    readString="";
  }
}
good=0;
///Serial.println(temp);
/// Here we create the new file
temp.concat(".txt");
char filename[temp.length()+1];
temp.toCharArray(filename, sizeof(filename));
if(!SD.exists(filename)){
  myFile = SD.open(filename,FILE_WRITE); // SD.open expects char array so we need to convert string to chara array  
 }
 //myFile.close();
 Serial.print("you File name is : ");
 Serial.println(temp);
 saved=1;
 c_file = temp;
 //Serial.println(c_file);
 temp="";

 buffer_to_SD();
}

//myFile.close(); /// MOŽDA ??
  
}
//--------------------------------------------------------------------



//--------------------------------------------------------------------
//Open a file from SD card and loads its saved positions
//It also checks if all positions are inside mechanical limitations 
// if they are not it will not execute anything from file and will return to standby position

void SD_open(){

byte flag=0;
byte good=0;
SD_status();
Serial.println("");
Serial.println("Enter the name of the file you want to open");
readString="";
temp="";
  
while(flag==0){
    
while(good==0){
while (Serial.available()) {
    delay(3);  
    char c = Serial.read();
    readString += c; 
  }
  readString.trim();
  if (readString.length() >0) {
   temp= String(readString);
   // temp=readString;
    good = 1;
    readString="";
  }
}


temp.concat(".txt");
char filename[temp.length()+1];
temp.toCharArray(filename, sizeof(filename));

if(!SD.exists(filename)){
  good=0;
  flag=0;
  Serial.println("");
 Serial.print(" File named : ");
 Serial.print(temp);
 Serial.println(" does not exist.");
 Serial.println(" Please enter a new file to open ( watch out on capital letters and spaces): ");
 temp="";
 readString="";
}else{
  Serial.println("");
  Serial.print(" File named : ");
 Serial.print(temp);
 Serial.println("  exists.");
myFile = SD.open(filename,FILE_READ);//// NEEDS TO BE FILE OPEN ,FILE WRITE DELETES THE CONTENT OF FILE I THINK.
good=1;
flag=1;

saved=1;
c_file="";
c_file = temp;

}
  
}

memset(record_positions, 0, sizeof(record_positions));
temp="";
while (myFile.available()) {
     temp=myFile.readString(); 
      
    }


byte done=0;
int len=0;
int i=0;
int x=0;
String temp2="";
int var=0;

 while(done == 0){
 int len=0;
  len=temp.length();
 ///Serial.println(len);
 temp.replace("\n","w");
 Serial.println(temp);
 len=temp.length();
 ///Serial.println(len);
 for(i=0;i<=len;i++){  
 // Serial.println(skra.charAt(i));             
   if(isDigit(temp.charAt(i))){
      temp2=temp2+temp.charAt(i);              
   }else if(isSpace(temp.charAt(i))){     
      record_positions[x][var]=temp2.toInt();             
       Serial.println(record_positions[x][var]);
      x=x+1;
      temp2=""; 
 
   } else if(temp.charAt(i)== 'w' ){
   // Serial.print("\n is done");
     record_positions[x][var]=temp2.toInt();
     Serial.println(record_positions[x][var]);
      x=0;
     var=var + 1;  
     temp2=""; 
   
   }else if(temp.charAt(i)=='\0'){///// nije dobro vjerovatno
      record_positions[x][var]=temp2.toInt();
      Serial.println(record_positions[x][var]);
      //Serial.println("break");0
              break; 
       }
 }   


done=1;
 }
 
position_counter=var+1;

}
//--------------------------------------------------------------------




//--------------------------------------------------------------------
//Close current file
void SD_close(){
  myFile.close();
  c_file="";
  saved=0; 
  temp="";
  memset(record_positions, 0, sizeof(record_positions));
  position_counter=0;
  standby_position();
}
//--------------------------------------------------------------------



//--------------------------------------------------------------------
//initialize SD card
void SD_init(){
  if (!SD.begin(PA4)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
}
//--------------------------------------------------------------------




//--------------------------------------------------------------------
//Buffer to SD card
void buffer_to_SD(){
  //// Record all new updated steps to the file
    if (myFile) {
    Serial.print("Writing to ");
    Serial.print(c_file);
    Serial.print("...");

   //Writing array of recorded steps to sd card
   int x=0;
   int y=0;
   for(x=0;x<position_counter;x++){
      for(y=0;y<=5;y++){
        if(x==0){
        myFile.print(record_positions[y][x]);
        }else if(y==0){
          myFile.println();
          myFile.print(record_positions[y][x]);
        }else{
          myFile.print(record_positions[y][x]);
        }
        if(y<5){
        myFile.print(" ");
        }
        if(y==5){
          myFile.print(";");
        }
      }
    
   }
   
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening file");
  }
}
//--------------------------------------------------------------------




//--------------------------------------------------------------------
//Enter positons 
void enter_positions(){
    Serial.print("Enter  position for joint 1  ");
    flag_done[joint[1]]=1;
    get_next_position(joint[1]);
    ///while(needed_position[joint[1]]==current_position[joint[1]]){  ///// ovaj dio joĹˇ istestiraj 
    while(flag_done[joint[1]]==1){
      get_next_position(joint[1]);
    }
    
    Serial.print("Enter position for joint 2  ");
    flag_done[joint[2]]=1;
    get_next_position(joint[2]);
    while(flag_done[joint[2]]==1){
      get_next_position(joint[2]);
    }
    
    Serial.print("Enter position for joint 3  "); 
    flag_done[joint[3]]=1;
    get_next_position(joint[3]);
    while(flag_done[joint[3]]==1){
      get_next_position(joint[3]);
    }
    
    Serial.print("Enter position for joint 4  ");
    flag_done[joint[4]]=1;
    get_next_position(joint[4]);
    while(flag_done[joint[4]]==1){
      get_next_position(joint[4]);
    }
    
    Serial.print("Enter position for joint 5  ");
    flag_done[joint[5]]=1;
    get_next_position(joint[5]);
    while(flag_done[joint[5]]==1){
      get_next_position(joint[5]);
    }
    
    Serial.print("Enter position for joint 6 "); 
    flag_done[joint[6]]=1;
    get_next_position(joint[6]);
    while(flag_done[joint[6]]==1){
      get_next_position(joint[6]);  
    }
}
//--------------------------------------------------------------------



//--------------------------------------------------------------------
void move_routine_startup(int joint_num){
 currentMillis=micros();
 ////// state is used to prevent triggering of this if statement twice in row 
 ///// same goes for second one , this secures perfect square wave form
 
  if(currentMillis-previousMillis[joint_num]>=current_pulse_widht[joint_num] and state[joint_num]==0){
    previousMillis[joint_num]=currentMillis;
    digitalWrite(dirPin[joint_num],HIGH);
    state[joint_num]=1;  
  }
  
 else if(currentMillis-previousMillis[joint_num]>=(current_pulse_widht[joint_num]) and state[joint_num]==1) {
    previousMillis[joint_num]=currentMillis;
    digitalWrite(dirPin[joint_num],LOW);
    state[joint_num]=0;
    current_position[joint_num]=current_position[joint_num] + 1;
  
  }
acceleration_startup(joint_num);
   
}
//--------------------------------------------------------------------




//--------------------------------------------------------------------
void acceleration_startup(int joint_num){

 
    if( acc_dec[joint_num]==1 ){
      currentMillis=micros();
      if(currentMillis-prevMillis_acc[joint_num]>=sample2[joint_num]){
        
        prevMillis_acc[joint_num]=currentMillis;
               
          current_ang_vel[joint_num]=const_s[joint_num]/current_pulse_widht[joint_num]+speed_step[joint_num];       
          current_pulse_widht[joint_num]=const_s[joint_num]/current_ang_vel[joint_num];
          if(current_pulse_widht[joint_num]<pulse_widht[joint_num]){
           /// accel_steps[joint_num]=abs(current_position[joint_num] - prev_position[joint_num]);
            current_pulse_widht[joint_num]=pulse_widht[joint_num];
            acc_dec[joint_num]=0;  
           
            
            //Serial.println("");
           
            
    }
   
  }
  
 }else if(abs(current_position[joint_num]-needed_position[joint_num])<=accel_steps[joint_num] and acc_dec[joint_num]==0){ 
    currentMillis=micros();
    if(currentMillis-prevMillis_acc[joint_num]>=sample2[joint_num]){
      prevMillis_acc[joint_num]=currentMillis;
      current_ang_vel[joint_num]=const_s[joint_num]/current_pulse_widht[joint_num]-speed_step[joint_num];
      current_pulse_widht[joint_num]=const_s[joint_num]/current_ang_vel[joint_num];
      
  }
  if(current_pulse_widht[joint_num]>=10000){
    current_pulse_widht[joint_num]=0;
   
    
      accel_steps_startup[joint_num]=abs(current_position[joint_num]-needed_position[joint_num]);
    
    
  }
 }
}
//--------------------------------------------------------------------

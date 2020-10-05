
/*************************************************************************
  By: Petar Crnjak
  Code is used to test Faze4 V2 distribution board
  For more info visit:
  https://github.com/PCrnjak/Faze4-Robotic-arm
**************************************************************************/



// Pin definitions 
// We are using first port, so pins are puls1, dir1, en1
const int stepPin = 12; 
const int dirPin = 24;  
const int enaPin = 25; 

// pulse_widht will determine the speed of stepper
volatile int pulse_widht = 150;
byte state=LOW;  
unsigned long previousMillis=0;
unsigned long current_position=0;
unsigned long needed_position=4400;


void setup(){

pinMode(enaPin,OUTPUT);
pinMode(stepPin,OUTPUT); 
pinMode(dirPin,OUTPUT); 
digitalWrite(dirPin,HIGH);


delay(1500);

}


void loop(){

if(needed_position>current_position && current_position>=0){  
digitalWrite(dirPin,LOW);
move_routine();
  }
if(needed_position<current_position && current_position>=0){
digitalWrite(dirPin,HIGH);
move_routine();
}
if(current_position == needed_position){
  current_position = needed_position * (-1);
}


}



void move_routine(){
unsigned long currentMillis=micros();
 ////// state is used to prevent triggering of this if statement twice in row 
 ///// same goes for second one , this secures perfect square wave form
  if(currentMillis-previousMillis>=pulse_widht and state==LOW){
    previousMillis=currentMillis;
    digitalWrite(stepPin,HIGH);
    state=HIGH;  
  }
  if(currentMillis-previousMillis>=(pulse_widht) and state==HIGH) {
    previousMillis=currentMillis;
    digitalWrite(stepPin,LOW);
    state=LOW;
    current_position=current_position + 1;
    
  }
}

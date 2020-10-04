
#define LIMIT1 11
#define LIMIT2 10
#define LIMIT3 9
#define LIMIT4 8
#define LIMIT5 7
#define LIMIT6 6
#define SENSOR1 5
#define SENSOR2 2
#define SENSOR3 1


#define PUL1 12
#define DIR1 24
#define ENA1 25

#define PUL2 26
#define DIR2 27
#define ENA2 28

#define PUL3 29
#define DIR3 30
#define ENA3 35

#define PUL4 36
#define DIR4 37
#define ENA4 38

#define PUL5 39
#define DIR5 13
#define ENA5 14

#define PUL6 15
#define DIR6 16
#define ENA6 17

#define PUL7 18
#define DIR7 19
#define ENA7 20

#define PUL8 21
#define DIR8 22
#define ENA8 23

#define GRIPPER 0


void setup() {

  Serial.begin(9600);
  Serial4.begin(9600);
  Serial5.begin(9600);

  /// OUTPUTS ///
  pinMode(PUL1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(ENA1, OUTPUT);
  pinMode(PUL2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(ENA2, OUTPUT);
  pinMode(PUL3, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(ENA3, OUTPUT);
  pinMode(PUL4, OUTPUT);
  pinMode(DIR4, OUTPUT);
  pinMode(ENA4, OUTPUT);
  pinMode(PUL5, OUTPUT);
  pinMode(DIR5, OUTPUT);
  pinMode(ENA5, OUTPUT);
  pinMode(PUL6, OUTPUT);
  pinMode(DIR6, OUTPUT);
  pinMode(ENA6, OUTPUT);
  pinMode(PUL7, OUTPUT);
  pinMode(DIR7, OUTPUT);
  pinMode(ENA7, OUTPUT);
  pinMode(PUL8, OUTPUT);
  pinMode(DIR8, OUTPUT);
  pinMode(ENA8, OUTPUT);


  pinMode(GRIPPER,  OUTPUT); // GRIPPER

  pinMode(LIMIT1, INPUT);
  pinMode(LIMIT2, INPUT);
  pinMode(LIMIT3, INPUT);
  pinMode(LIMIT4, INPUT);
  pinMode(LIMIT5, INPUT);
  pinMode(LIMIT6, INPUT);
  pinMode(SENSOR1, INPUT);
  pinMode(SENSOR2, INPUT);
  pinMode(SENSOR3, INPUT);

  /// INPUTS ///






}

void loop() {

  digitalWrite(PUL1, HIGH);
  digitalWrite(PUL2, HIGH);
  digitalWrite(PUL3, HIGH);
  digitalWrite(PUL4, HIGH);
  digitalWrite(PUL5, HIGH);
  digitalWrite(PUL6, HIGH);
  digitalWrite(PUL7, HIGH);
  digitalWrite(PUL8, HIGH);


  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, HIGH);
  digitalWrite(DIR3, HIGH);
  digitalWrite(DIR4, HIGH);
  digitalWrite(DIR5, HIGH);
  digitalWrite(DIR6, HIGH);
  digitalWrite(DIR7, HIGH);
  digitalWrite(DIR8, HIGH);


  digitalWrite(ENA1, HIGH);
  digitalWrite(ENA2, HIGH);
  digitalWrite(ENA3, HIGH);
  digitalWrite(ENA4, HIGH);
  digitalWrite(ENA5, HIGH);
  digitalWrite(ENA6, HIGH);
  digitalWrite(ENA7, HIGH);
  digitalWrite(ENA8, HIGH);
  digitalWrite(GRIPPER, HIGH);

  Serial4.println("Serial4 is working!");
  Serial5.println("Serial5 is working!");
  delay(2000);


  digitalWrite(PUL1, LOW);
  digitalWrite(PUL2, LOW);
  digitalWrite(PUL3, LOW);
  digitalWrite(PUL4, LOW);
  digitalWrite(PUL5, LOW);
  digitalWrite(PUL6, LOW);
  digitalWrite(PUL7, LOW);
  digitalWrite(PUL8, LOW);


  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  digitalWrite(DIR3, LOW);
  digitalWrite(DIR4, LOW);
  digitalWrite(DIR5, LOW);
  digitalWrite(DIR6, LOW);
  digitalWrite(DIR7, LOW);
  digitalWrite(DIR8, LOW);


  digitalWrite(ENA1, LOW);
  digitalWrite(ENA2, LOW);
  digitalWrite(ENA3, LOW);
  digitalWrite(ENA4, LOW);
  digitalWrite(ENA5, LOW);
  digitalWrite(ENA6, LOW);
  digitalWrite(ENA7, LOW);
  digitalWrite(ENA8, LOW);
  digitalWrite(GRIPPER, LOW);

  Serial.print("LIMIT1 is: "); Serial.println(digitalRead(LIMIT1));
  Serial.print("LIMIT2 is: "); Serial.println(digitalRead(LIMIT2));
  Serial.print("LIMIT3 is: "); Serial.println(digitalRead(LIMIT3));
  Serial.print("LIMIT4 is: "); Serial.println(digitalRead(LIMIT4));
  Serial.print("LIMIT5 is: "); Serial.println(digitalRead(LIMIT5));
  Serial.print("LIMIT6 is: "); Serial.println(digitalRead(LIMIT6));
  Serial.print("SENSOR1 is: "); Serial.println(digitalRead(SENSOR1));
  Serial.print("SENSOR2 is: "); Serial.println(digitalRead(SENSOR2));
  Serial.print("SENSOR3 is: "); Serial.println(digitalRead(SENSOR3));

  delay(2000);


}

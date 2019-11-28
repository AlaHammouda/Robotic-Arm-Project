 
const int stepPin_1 = 2;
const int dirPin_1 = 5;
const int stepPin_2 = 3;
const int dirPin_2 =6;
const int stepPin_3 = 4;
const int dirPin_3 = 7;
const int stepPin_4 = 12;
const int dirPin_4 = 13;
int stepper_Freq=650;

void setup(){
  pinMode(stepPin_1, OUTPUT);
  pinMode(dirPin_1, OUTPUT);
  pinMode(stepPin_2, OUTPUT);
  pinMode(dirPin_2, OUTPUT);
  pinMode(stepPin_3, OUTPUT);
  pinMode(dirPin_3, OUTPUT);
  pinMode(stepPin_4, OUTPUT);
  pinMode(dirPin_4, OUTPUT);
//rotate_stepper(10,1,stepPin_1,dirPin_1);
//rotate_stepper(10,0,stepPin_1,dirPin_1);
  digitalWrite(dirPin_1, 1); // Enables the motor to move in a particular direction
  digitalWrite(dirPin_2, 1); // Enables the motor to move in a particular direction
  digitalWrite(dirPin_3, 1); // Enables the motor to move in a particular direction
  digitalWrite(dirPin_4, 1); // Enables the motor to move in a particular direction

  for (int x = 0; x < 2000; x++) {
    digitalWrite(stepPin_1, HIGH);digitalWrite(stepPin_2, HIGH);digitalWrite(stepPin_3, HIGH);digitalWrite(stepPin_4, HIGH);
    delayMicroseconds(stepper_Freq);  //300       // 100
        digitalWrite(stepPin_1, LOW);digitalWrite(stepPin_2, LOW);digitalWrite(stepPin_3, LOW);digitalWrite(stepPin_4, LOW);
    delayMicroseconds(stepper_Freq);              // 100
  }

}


void loop() {
}



void rotate_stepper(float dist,int sens,int stepPin,int dirPin)
{  dist=200*dist;
  digitalWrite(dirPin, sens); // Enables the motor to move in a particular direction
  for (int x = 0; x < dist; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepper_Freq);  //300       // 100
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepper_Freq);              // 100
  }
}

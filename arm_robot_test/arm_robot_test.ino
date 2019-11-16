 
const int stepPin = 3;//3
const int dirPin = 2;//2

void setup(){
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
bas(80);
//haut(40);
}


void loop() {
}



void haut(float dist)
{  dist=447*dist;
  digitalWrite(dirPin, LOW); // Enables the motor to move in a particular direction
  for (int x = 0; x < dist; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(80);  //300       // 100
    digitalWrite(stepPin, LOW);
    delayMicroseconds(80);              // 100
  }
}


void bas(float dist)
{   dist=490*dist;
  digitalWrite(dirPin, HIGH); //Changes the rotations direction
  // Makes 400 pulses for making two full cycle rotation
  for (int x = 0; x < dist; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(80);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(80);
  }
}

#define EC1_A 50
#define EC1_B 51
#define IN1 5
#define IN2 6
#define IN3 7
#define IN4 8 

#define naprijed 2
#define nazad 3

void setup() { 
  pinMode (EC1_A,INPUT);
  pinMode (EC1_B,INPUT);
  pinMode (IN1,OUTPUT);
  pinMode (IN2,OUTPUT);
  pinMode (IN3,OUTPUT);
  pinMode (IN4,OUTPUT);
  pinMode (naprijed, INPUT_PULLUP);
  pinMode (nazad, INPUT_PULLUP);
  Serial.begin (9600);  


} 

void loop() { 
  int naprijedPritisnuto = digitalRead(naprijed);
  int nazadPritisnuto = digitalRead(nazad); // HIGH/LOW odnosno 1 ili 0, nema volta ima volta
  if(naprijedPritisnuto == LOW && nazadPritisnuto == HIGH) {
    digitalWrite(IN2, LOW);
    analogWrite(IN1, 200);
  
    digitalWrite(IN4, HIGH);
    analogWrite(IN3, 200);
  } else if(nazadPritisnuto == LOW && naprijedPritisnuto == HIGH) {
    digitalWrite(IN1, LOW);
    analogWrite(IN2, 200);
  
    digitalWrite(IN3, LOW);
    analogWrite(IN4, 200);
  } else {
    digitalWrite(IN2, LOW);
    analogWrite(IN1, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(IN3, LOW);
  }
  
}


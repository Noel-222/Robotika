/*
 * CopyRight www.osoyoo.com
*/

const int pulsesPerRevolution = 565;  // broj impulsa po punom okretaju (OTPRILIKE.. )

#define SPEED 80    
#define TURN_SPEED 100    
#define FR_speedPin 11   //  Front Wheel PWM pin connect Right MODEL-X ENA 
#define FR_dirPin1 5     //Front Right)) Motor direction pin 1 to Right MODEL-X IN1  (K1)
#define FR_dirPin2 6     //Front Right)) Motor direction pin 2 to Right MODEL-X IN2   (K1)
#define C1 3    // žuta žica na encoderu –> A signal (interrupt)
#define C2 53   // zelena žica na encoderu –> B signal

volatile long pos = 0;

// očitava signale a i b
void readEncoder() {
  if (digitalRead(C2) == HIGH)
    pos++;
  else
    pos--;
}

// kontroliranje motora
void go_advance(int speed, float turns) {    // funckija za ići naprijed
  long startPos = pos;  // zapamti početni položaj
  long targetPulses = turns * pulsesPerRevolution;

  FR_fwd(speed);

  // Dok se motor ne okrene zadani broj okretaja
  while (abs(pos - startPos) < targetPulses) {
    long remaining = targetPulses - abs(pos - startPos);
    if (remaining < 10) {  // zadnjih 10 impulsa
      FR_fwd(speed / 2);  // uspori motor
    }
    delay(1);
  }

  stop_Stop();  // stani kad se dosegne broj okretaja
}

void go_back(int speed, float turns) {     // funckija za ići nazad
  long startPos = pos;
  long targetPulses = turns * pulsesPerRevolution;

  FR_bck(speed);

/* Petlja se izvršava dokle god razlika između trenutne pozicije motora (pos)
   i početne pozicije (startPos) nije manja od ciljanog broja impulsa (targetPulses)
*/ 
while (abs(pos - startPos) < targetPulses) {

    // Izračunava koliko impulsa još preostaje do cilja
    long remaining = targetPulses - abs(pos - startPos);

    // Provjera da li smo u zadnjih 50 impulsa prije cilja
    if (remaining < 50) {  // zadnjih 50 impulsa
        // Ako smo blizu cilja, uspori motor na pola brzine
        FR_bck(speed / 4);  // funkcija koja upravlja motorom u određenom smjeru
    }

    // Male kašnjenje da petlja ne radi prebrzo i da sustav može "uhvatiti" poziciju
    delay(1);  // 1 milisekunda
}

  stop_Stop();
}

void FR_fwd(int speed)  //front-right wheel forward turn
{
  digitalWrite(FR_dirPin1, LOW);
  digitalWrite(FR_dirPin2, HIGH);
  analogWrite(FR_speedPin, speed);
}

void FR_bck(int speed)  // front-right wheel backward turn
{
  digitalWrite(FR_dirPin1, HIGH);
  digitalWrite(FR_dirPin2, LOW);
  analogWrite(FR_speedPin, speed);
}

void stop_Stop()    //Stop
{
  analogWrite(FR_speedPin, 0);
}

//Pins initialize
void init_GPIO()
{
  pinMode(C1, INPUT);
  pinMode(C2, INPUT);

  pinMode(FR_dirPin1, OUTPUT);
  pinMode(FR_dirPin2, OUTPUT);
  pinMode(FR_speedPin, OUTPUT);

  stop_Stop();
}

void setup()
{
  Serial.begin(9600);
  init_GPIO();
  attachInterrupt(digitalPinToInterrupt(C1), readEncoder, RISING);

  pos = 0;
  go_advance(SPEED, 4);    // zadamo motoru da ide naprijed i napravi 4 okretaja
  delay(1000);
  stop_Stop();
  delay(1000);

  go_back(SPEED, 2);    // zadamo motoru da ide nazad i napravi 2 okretaja
  delay(1000);
  stop_Stop();
  delay(1000);
}

void loop(){
  Serial.println(pos);
}

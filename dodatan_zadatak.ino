/*
 * ===========================================================================
 * VOŽNJA NA CILJ (  15 cm naprijed , 90 stupnjeva u desno , 22 cm nazad  )
 * ===========================================================================
*/

// --- Pinovi za Lijevog Enkodera ---
// Ovo nisu bilo koji pinovi! Na Arduino Mega, pinovi 18 i 19
// su posebni HARDVERSKI INTERAPT pinovi (INT5 i INT4).
#define ENCODER_L_C1_PIN 18 // Signal A
#define ENCODER_L_C2_PIN 19 // Signal B

// --- Pinovi za Desnog Enkodera (NOVI PINOVI) ---
// Pinove 20 i 21 ČUVAMO za I2C komunikaciju (za IMU senzor kasnije).
// Zato desni enkoder spajamo na pinove 2 i 3, koji su
// također HARDVERSKI INTERAPT pinovi (INT0 i INT1).
#define ENCODER_R_C1_PIN 2  // Signal A
#define ENCODER_R_C2_PIN 3  // Signal B

// --- Pinovi za Lijevog Motora (PRETPOSTAVKA) ---
// Ovo su pinovi koji idu s Arduina na tvoj Motor Driver (npr. Model Y Board)
#define MOTOR_L_EN_PIN  10 // EN (Enable) pin, mora biti PWM pin (oznaka ~) za kontrolu BRZINE
#define MOTOR_L_IN1_PIN 11 // IN1 (Input 1) pin za SMJER
#define MOTOR_L_IN2_PIN 12 // IN2 (Input 2) pin za SMJER

// --- Pinovi za Desnog Motora (PRETPOSTAVKA) ---
#define MOTOR_R_EN_PIN  5 // EN (Enable) pin, mora biti PWM pin (oznaka ~)
#define MOTOR_R_IN1_PIN 6 // IN1 (Input 1)
#define MOTOR_R_IN2_PIN 7 // IN2 (Input 2)

// --- Globalne Varijable za Enkodere ---
//
// 'volatile' je ključna riječ. Ona govori Arduinu:
// "Ova varijabla se može promijeniti u bilo kojem trenutku izvan
// glavnog 'loop()'-a (npr. preko interapta)". To sprječava
// kompajler da "pametuje" i slučajno preskoči čitanje nove vrijednosti.
//
// 'long' koristimo umjesto 'int'. 'int' ide samo do 32,767.
volatile long encoderPosL = 0;
volatile long encoderPosR = 0;
int faza = 1;  // 1 = vožnja prema targetPosition, 2 = vožnja prema targetPosition2

// --- Postavke Regulatora ---
// OVO SU VRIJEDNOSTI KOJE ĆEŠ NAJVIŠE PODEŠAVATI (TUNIRATI)!
// P-Regulator znači "Proporcionalni Regulator".
// Logika: BrzinaMotora = Kp * Greška (Koliko smo daleko od cilja)

double Kp = 0.5;      // Proporcionalni koeficijent. "Jačina gasa".
                      // Ako je Kp mali, robot će sporo ići do cilja.
                      // Ako je Kp prevelik, robot će "proletjeti" preko cilja
                      // i vraćati se natrag (oscilirati). Kreni s malim (0.5).

int minSpeed = 70;    // Motori se neće pokrenuti ako im daš 'analogWrite(10)'.
                      // Trebaju minimalnu snagu da savladaju trenje.
                      // Ovo je ta minimalna snaga (0-255).

int maxSpeed = 200;   // Maksimalna brzina (0-255). Ne idemo odmah na 255
                      // da imamo prostora za kontrolu.

int deadZone = 20;    // "Mrtva zona". Koliko blizu cilja (+/- 20 pulseva)
                      // smatramo da je "dovoljno dobro"?
                      // Ovo sprječava da motor "drhti" oko cilja.

// --- GLAVNI CILJ ---
// Ovdje unesi broj koji si dobio u KORAKU 1 (Kalibracija).
long targetPosition = 1500; 
long targetPosition3 = -1850;
long targetPositionL = 870;
long targetPositionR = -820;
bool voznjaAktivna = true; // "Zastavica" (flag). Dok je 'true', robot vozi.
                           // Kad stigne, postavit ćemo je na 'false'.

void setup() {
  Serial.begin(115200); // Pokreni Serial Monitor za praćenje

  // --- Postavi Pinove Motora ---
  // Svi pinovi koji šalju naredbe motor driveru su IZLAZI (OUTPUT)
  pinMode(MOTOR_L_EN_PIN, OUTPUT);
  pinMode(MOTOR_L_IN1_PIN, OUTPUT);
  pinMode(MOTOR_L_IN2_PIN, OUTPUT);
  pinMode(MOTOR_R_EN_PIN, OUTPUT);
  pinMode(MOTOR_R_IN1_PIN, OUTPUT);
  pinMode(MOTOR_R_IN2_PIN, OUTPUT);

  // --- Postavi Pinove Enkodera ---
  // Svi pinovi enkodera su ULAZI (INPUT)
  // Koristimo 'INPUT_PULLUP'. Ovo aktivira interni otpornik na Arduinu
  // koji "vuče" napon na pinu na 5V (HIGH). Enkoderi često
  // samo spoje pin na 0V (GND), a PULLUP osigurava da je pin
  // stabilno HIGH dok ga enkoder ne povuče LOW. Sprječava "plutanje"
  // signala i lažna očitanja.
  pinMode(ENCODER_L_C1_PIN, INPUT_PULLUP);
  pinMode(ENCODER_L_C2_PIN, INPUT_PULLUP);
  pinMode(ENCODER_R_C1_PIN, INPUT_PULLUP);
  pinMode(ENCODER_R_C2_PIN, INPUT_PULLUP);

  // --- Zakači Interapte ---
  // Ovdje se događa magija. Govorimo Arduinu:
  // "Hey, kad god vidiš UZLAZNI RUB (RISING, prijelaz s LOW na HIGH)
  //  na pinu X, ODMAH pokreni funkciju Y."
  
  // 'digitalPinToInterrupt(PIN)' pretvara broj pina (npr. 18)
  // u interni naziv interapta (npr. INT5).
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_C1_PIN), readEncoderL_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_C2_PIN), readEncoderL_B, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_C1_PIN), readEncoderR_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_C2_PIN), readEncoderR_B, RISING);

  Serial.println("Robot spreman. Pokrecem voznju od 15 cm...");
  Serial.print("Ciljana pozicija: ");
  Serial.println(targetPosition);
  delay(2000); // Pauza od 2 sekunde da se stigneš pripremiti
}

void loop() {

  // 1. Provjera globalne zastavice vožnje.
  // Ako je 'voznjaAktivna == false', završili smo sve ciljeve.
  // Ugasimo motore i više ništa ne radimo u ovoj petlji.
  if (!voznjaAktivna) {
    motorStopL();
    motorStopR();
    return;
  }

  // Ovo je dobra praksa jer se 'encoderPosL/R' mogu promijeniti
  // u SREDINI loop() funkcije.
  long trenutnaPozL = encoderPosL;
  long trenutnaPozR = encoderPosR;

  /* ============================================================
                  FAZA 1 - vožnja na cilj 1
     ============================================================
  
   U FAZI 1 robot vozi prema 'targetPosition'.
   Kada oba motora jave da su stigla unutar "mrtve zone",
   motori se zaustavljaju i prelazimo u FAZU 2.
  */
  if (faza == 1) {

    // 3. P-regulator za lijevi i desni motor.
    bool lijeviStigao = runMotorP(targetPosition, trenutnaPozL, motorForwardL, motorReverseL, motorStopL);

    bool desniStigao  = runMotorP(targetPosition, trenutnaPozR, motorForwardR, motorReverseR, motorStopR);

    // 4. Provjera jesu li oba motora stigla na cilj.
    if (lijeviStigao && desniStigao) {

      Serial.println("--- CILJ 1 DOSEGNUT ---");

      // 4A. Odmah zaustavimo motore.
      motorStopL();
      motorStopR();

      // 4B. Pauza da se robot smiri.
      delay(1000);

      /* ============================================================
                  RESET ENKODERA ( JAKO VAŽNO )
         ============================================================
      
       Ako ne resetiramo encodere, drugi P-regulator bi krenuo
       računati grešku na temelju pozicije od 1. vožnje
      */
      noInterrupts();      // Privremeno isključi interapte
      encoderPosL = 0;     // Oboje postavi na 0
      encoderPosR = 0;
      interrupts();        // Vrati interapte

      Serial.println("Enkoderi resetirani. Pokrećem FAZU 2...");

      // Prelazak u drugu fazu
      faza = 2;

      delay(500); // Kratka pauza
    }

  }
  /* ============================================================
                    FAZA 2 - Okretanje za 90 desno
     ============================================================
  
   Nakon resetiranja encodera, robot kreće prema drugom cilju
   Logika je ista kao u fazi 1, ali sada koristimo targetPositionL i targetPositionR
   jedan motor se okreće u jednom smjeru dok se drugi motor okreće u drugom smijeru 
   zbog čega dolazi do okretanja robota
  */
   else if(faza == 2) {
    
    bool lijeviStigao2 = runMotorP(targetPositionL, trenutnaPozL, motorForwardL, motorReverseL, motorStopL);
    bool desniStigao2  = runMotorP(targetPositionR, trenutnaPozR, motorForwardR, motorReverseR, motorStopR);
 
    if (lijeviStigao2 && desniStigao2) {

      Serial.println("--- CILJ 2 DOSEGNUT ---");

      motorStopL();
      motorStopR();

      noInterrupts();
      encoderPosL = 0;
      encoderPosR = 0;
      interrupts();
      Serial.println("Enkoderi resetirani. Pokrećem FAZU 3...");

      // Prelazak u drugu fazu
      faza = 3;

      delay(500);
    }
  }

  /* ============================================================
                        FAZA 3 - voznja na cilj 3
     ============================================================
  
   Nakon resetiranja encodera, robot kreće prema trećem cilju
  */ 
  else if(faza == 3) {

    bool lijeviStigao3 = runMotorP(targetPosition3, trenutnaPozL,
                                   motorForwardL, motorReverseL, motorStopL);

    bool desniStigao3  = runMotorP(targetPosition3, trenutnaPozR,
                                   motorForwardR, motorReverseR, motorStopR);

    // 5. Ako smo stigli na drugi cilj, vožnja je gotova.
    if (lijeviStigao3 && desniStigao3) {

      Serial.println("--- CILJ 3 DOSEGNUT ---");

      motorStopL();
      motorStopR();

      // ovdje gasimo glavni "voznjaAktivna" flag.
      // To zaustavlja sve i sprječava dodatne pokrete.
      voznjaAktivna = false;
      return;
    }
  }

  // 6. Informativni ispis pozicija encodera.
  Serial.print("L = ");
  Serial.print(trenutnaPozL);
  Serial.print(" | R = ");
  Serial.println(trenutnaPozR);

  // 7. Pauza između iteracija.
  // P-regulator radi vrlo dobro s 20ms petljom.
  delay(20);
}


// --- Generička Funkcija P-Regulatora ---
//
// Ovo je "mozak" operacije.
// Prima cilj, trenutnu poziciju i "pokazivače na funkcije"
// (to su funkcije koje će pozvati da pokrene motor).
// Vraća 'bool' (true/false) da javi je li stigao.
//
bool runMotorP(long target, long current, void (*fwd)(int), void (*rev)(int), void (*stop)()) {
  
  // 1. Izračunaj GREŠKU (Error). Ključan dio.
  // Koliko smo daleko od cilja? (npr. 12450 - 1000 = 11450)
  int error = target - current;

  // 2. Provjera "mrtve zone".
  // 'abs()' je apsolutna vrijednost (npr. abs(-15) = 15).
  // Ako je greška unutar +/- 'deadZone' (npr. 20),
  // stani i javi da si stigao.
  if (abs(error) <= deadZone) {
    stop();       // Pozovi funkciju za zaustavljanje (npr. motorStopL)
    return true;  // Javi 'loop'-u da je stigao
  }

  // 3. Izračun P-regulatora. Ovdje je 'P' (Proporcija).
  // Brzina je proporcionalna grešci.
  // Ako je 'error' velik (11450), brzina će biti velika.
  // Ako je 'error' mali (100), brzina će biti mala.
  int motorSpeed = Kp * error;

  // 4. Ograniči brzine (Saturation & Clamping)
  if (motorSpeed > 0) { // Ako je 'error' pozitivan, trebamo ići NAPRIJED
    // 'constrain' je super funkcija: (vrijednost, min, max)
    // Osigurava da brzina nije ni manja od 'minSpeed' ni veća od 'maxSpeed'.
    motorSpeed = constrain(motorSpeed, minSpeed, maxSpeed);
    fwd(motorSpeed); // Pozovi funkciju za naprijed (npr. motorForwardL)
  
  } else { // Ako je 'error' negativan, prešli smo cilj, trebamo ići NATRAG
    // 'constrain' radi s pozitivnim brojevima, pa koristimo abs()
    motorSpeed = constrain(abs(motorSpeed), minSpeed, maxSpeed);
    rev(motorSpeed); // Pozovi funkciju za natrag (npr. motorReverseL)
  }
  
  return false; // Javi 'loop'-u da još nismo stigli
}

// --- Funkcije za Upravljanje Motorima (Lijevi) ---
// Ove funkcije primaju 'pwm' (brzinu) i pale/gase IN1/IN2
// za smjer, te šalju PWM brzinu na EN pin.

void motorForwardL(int pwm) {
  digitalWrite(MOTOR_L_IN1_PIN, LOW);
  digitalWrite(MOTOR_L_IN2_PIN, HIGH);
  analogWrite(MOTOR_L_EN_PIN, pwm);
}
void motorReverseL(int pwm) {
  digitalWrite(MOTOR_L_IN1_PIN, HIGH);
  digitalWrite(MOTOR_L_IN2_PIN, LOW);
  analogWrite(MOTOR_L_EN_PIN, pwm);
}
void motorStopL() { // Zaustavljanje (oba IN pina na LOW)
  digitalWrite(MOTOR_L_IN1_PIN, LOW);
  digitalWrite(MOTOR_L_IN2_PIN, LOW);
  analogWrite(MOTOR_L_EN_PIN, 0);
}

// --- Funkcije za Upravljanje Motorima (Desni) ---
void motorForwardR(int pwm) {
  digitalWrite(MOTOR_R_IN1_PIN, HIGH);
  digitalWrite(MOTOR_R_IN2_PIN, LOW);
  analogWrite(MOTOR_R_EN_PIN, pwm);
}
void motorReverseR(int pwm) {
  digitalWrite(MOTOR_R_IN1_PIN, LOW);
  digitalWrite(MOTOR_R_IN2_PIN, HIGH);
  analogWrite(MOTOR_R_EN_PIN, pwm);
}
void motorStopR() {
  digitalWrite(MOTOR_R_IN1_PIN, LOW);
  digitalWrite(MOTOR_R_IN2_PIN, LOW);
  analogWrite(MOTOR_R_EN_PIN, 0);
}


// --- ISR (Interrupt) Funkcije (Kopirane iz kalibracijskog koda) ---
//
// Ovdje se događa "kvadraturna" logika.
// Enkoder ima 2 signala (A i B) koji su fazno pomaknuti.
// Kada se A digne (RISING), provjerimo B.
// Ako je B LOW, vrtimo se npr. naprijed (count++).
// Ako je B HIGH, vrtimo se natrag (count--).
// Ista logika vrijedi i za B (samo obrnuto).
// Ovo nam daje 2 očitanja po ciklusu i detekciju smjera.

// --- LIJEVI MOTOR ---
void readEncoderL_A() { // Kad se C1 (18) digne...
  if (digitalRead(ENCODER_L_C2_PIN) == LOW) { // ...provjeri C2 (19). Ako je LOW...
    encoderPosL++; // ...broji naprijed.
  } else {
    encoderPosL--; // ...inače broji natrag.
  }
}
void readEncoderL_B() { // Kad se C2 (19) digne...
  if (digitalRead(ENCODER_L_C1_PIN) == HIGH) { // ...provjeri C1 (18). Ako je HIGH...
    encoderPosL++; // ...broji naprijed.
  } else {
    encoderPosL--; // ...inače broji natrag.
  }
}

// --- DESNI MOTOR ---
// VAŽNO: Desni motor je fizički okrenut naopačke u odnosu na lijevi.
// Zato su '++' i '--' OBRNUTI.
// (Ako ti oba broje u minus dok guraš robota naprijed,
//  samo zamijeni '++' i '--' u ove dvije funkcije).
void readEncoderR_A() { // Kad se C1 (2) digne...
  if (digitalRead(ENCODER_R_C2_PIN) == LOW) { // ...provjeri C2 (3). Ako je LOW...
    encoderPosR--; // ...broji natrag (obrnuto od lijevog).
  } else {
    encoderPosR++; // ...broji naprijed.
  }
}
void readEncoderR_B() { // Kad se C2 (3) digne...
  if (digitalRead(ENCODER_R_C1_PIN) == HIGH) { // ...provjeri C1 (2). Ako je HIGH...
    encoderPosR--; // ...broji natrag.
  } else {
    encoderPosR++; // ...broji naprijed.
  }
}



#define enkoderPinA 2   // pin A od enkodera (spojen na digitalni pin 2)
#define enkoderPinB 3   // pin B od enkodera (spojen na digitalni pin 3)
#define motorPinA 5     // PWM pin za upravljanje brzinom motora
#define motorPinB 6     // pin za smjer (ovdje se koristi kao LOW)

const float IMPULSA_PO_OKRETU = _____;     // broj impulsa po jednom okretu osovine motora (možemo očitati sa odela našeg motora)
const float PROMJER_KOTACA = ______;          // promjer kotača u centimetrima
const float OPSEG_KOTACA = 3.1416 * PROMJER_KOTACA; // opseg = π * promjer (udaljenost za 1 okret)

volatile long brojImpulsa = 0; // ukupni broj impulsa (volatile jer se mijenja unutar prekida)

void setup() {
  Serial.begin(9600);

  pinMode(enkoderPinA, INPUT_PULLUP);
  pinMode(enkoderPinB, INPUT_PULLUP);

  // spajanje prekida (interrupta) na pinove enkodera
  attachInterrupt(digitalPinToInterrupt(enkoderPinA), azurirajEnkoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enkoderPinB), azurirajEnkoder, CHANGE);

  // postavljanje pinova motora kao izlaznih
  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);
}

void loop() {
  // izračunaj udaljenost u centimetrima
  // formula: (broj_impulsa / impulsa_po_okretu) * opseg_kotača --  dio formule u zagradi izračunava ukupan broj okretaja koje je kotač napravio.
  // Množenjem ukupnog broja okretaja s opsegom kotača dobivate ukupnu prevaljenu udaljenost.
  float udaljenost_cm = (brojImpulsa / IMPULSA_PO_OKRETU) * OPSEG_KOTACA;

  // ispiši podatke u serijski monitor (za praćenje kretanja robota)
  Serial.print("Impulsi: ");
  Serial.print(brojImpulsa);
  Serial.print(" | Udaljenost: ");
  Serial.print(udaljenost_cm);
  Serial.println(" cm");

  // pokreni motor unaprijed srednjom brzinom
  analogWrite(motorPinA, 127); // za arduino tinkercad motor srednja brzina je 127... za naš možemo očitat
  digitalWrite(motorPinB, LOW); 

  delay(500); 
}
// Poziva se automatski svaki put kad dođe do promjene na pinovima enkodera
void azurirajEnkoder() {
  int stanjeA = digitalRead(enkoderPinA); 
  int stanjeB = digitalRead(enkoderPinB); 

  // ako su signali isti, znači da se motor vrti u jednom smjeru
  // ako su različiti, motor se vrti u suprotnom smjeru
  if (stanjeA == stanjeB)
    brojImpulsa++;   // povećaj broj impulsa (rotacija naprijed)
  else
    brojImpulsa--;   // smanji broj impulsa (rotacija unazad)
}

#define POTENTIOMETRE_KP A1
#define POTENTIOMETRE_KI A2
#define POTENTIOMETRE_KD A3

void setup() {
  Serial.begin(9600);
}

void loop() {
  int valueP = analogRead(POTENTIOMETRE_KP);      // realizar la lectura analógica raw
  int valueI = analogRead(POTENTIOMETRE_KI);      // realizar la lectura analógica raw
  int valueD = analogRead(POTENTIOMETRE_KD);      // realizar la lectura analógica raw
  Serial.print("Value: ");
  Serial.print(valueP);
  Serial.print("\t");
  Serial.print(valueI);
  Serial.print("\t");
  Serial.println(valueD);
  delay(100);
}

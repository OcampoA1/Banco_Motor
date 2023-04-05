#include "HX711.h"

const int DOUT=A3;
const int CLK=A2;

HX711 balanza;

void setup() {
  Serial.begin(9600);
  pinMode(5, OUTPUT);
  balanza.begin(DOUT, CLK);
  Serial.print("Lectura del valor del ADC:  ");
  Serial.println(balanza.read());
  Serial.println("No ponga ningun  objeto sobre la balanza");
  Serial.println("Destarando...");
  Serial.println("...");
  balanza.set_scale(439430.25); // Establecemos la escala
  balanza.tare(20);  //El peso actual es considerado Tara.
  
  Serial.println("Listo para pesar");  
}

void loop() {
  analogWrite(5, 250);
  Serial.print("Peso: ");
  Serial.print(balanza.get_units(20)*(-9.8),3);
  Serial.println(" kg");
  delay(500);
}

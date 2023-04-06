#include <string.h>
#include <EEPROM.h>
#include "HX711.h"
#define pinMotor 5
//variables consola
int direccion = 0;
uint8_t percentage = 10; //< Porcentaje de la estimulación
uint32_t periodo = 500; //< Valor del periodo, por defecto 500ms 
uint32_t Fs = 200; //< Valor de la frecuencia de muestreo, por defecto 200Hz
uint16_t n_samples = 1; //< Numero de muestras, por defecto 1 
float current_max = 1; //< Numero de corriente maxima, por defecto 1 Amperio.
bool verb = false; //< Bandera que dice si esta en verbose on u off (false es VERBOSE OFF)
bool stm = true; //< Bandera para saber que tipo de estimulacion se realiza, por defecto true (true = MODE STP)
bool red_flag = false;
// Variables de interrupcion y sensorica
int interruptPin =2;
uint32_t n_preescaler = (16000000)/256; // desde 1Hz - 62.5kHz
volatile int freq = 0; //Interrupcion para RPM 
unsigned long time1, time2 = 0;  // Ventana de tiempo RPM 
float measurement_i = 0;
float measurement_vel = 0;
float measurement_temp = 0;
uint32_t actual_vel,actual_vel_freq, avg_vel = 0;
int count = 0;
float actual_current, avg_current = 0;
float sens_current = 0.185;
uint32_t offset_i = 0;
// Temperature sensor var
float actual_temp, avg_temp = 0;
float offset_temp = 0;

float measurement_celda = 0;
float avg_celda = 0;
float actual_celda = 0;
float resultado = 0;
// Variables para generar señal 
int MODE = 0; // step: 0, trigg: 1
uint32_t t_actual = 0;
uint32_t t_anterior = 0;
unsigned long int T = 500000; // T para trigg, dt para step    CONSOLA
unsigned long int dy = 10; // paso de % en step            CONSOLA
unsigned long int dt; //
int Vmax; // Vmax =5V (trigg) o Vmax = 100% (step)
unsigned int y_puntos; // numero de escalones hasta el maximo
long int m; //pendiente de señal triangular; T debe ser mayor que 2 us
unsigned long int acum = 0;
float y;
int b;
bool rising;
bool enabled_sig = false; ///< Dice si el sistema esta activo o no



//HX711 balanza; //< Objeto para la balanza

void setup() {
  
  pinMode(pinMotor, OUTPUT); //< Pin para el motor
  pinMode(A0, INPUT); //< A0 sensado de corriente
  pinMode(A4, INPUT); //< A4 sensado de temperatura
  //balanza.begin(A3,A4); //< Se incializa la celda de carga
  //balanza.set_scale(439430.25); // Establecemos la escala
  //balanza.tare(20);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP); //< Pin para realizar la interupcción en el sensado de RPM
  attachInterrupt(digitalPinToInterrupt(interruptPin), counter_function, RISING); //
  cli(); //< Detener interrupciones
  TCCR1A = 0; //pwm, se estbalece en cero, dado que por defecto esta en 1
  TCCR1B = 0; //prescalador, se establece en cero

  // Se inicializa el preescalador a utilizar
  TCCR1B |= B00000100; // Se establece un preescalador de 256, 
  // TCCR1B |= (1 << CS12); //Otra forma de poner en 1 CS12 para que el preescalador sea 256

  //Se habilita la forma de realizar interrupciones (Compare Match)
  TIMSK1 |= B00000010; //Se indica el registro con el que se va realizar la comparación, MODO OCR1A, se pone OCIE1A en 1
  //Serial.print("Este: ");
  //Serial.println(d);
  // Se fija el valor del registro
  OCR1A = n_preescaler/Fs; //Dado que se va contar hasta 5ms, con un preescalador de 256
  sei(); //< Habiulitan interrupciones
  Serial.begin(9600);
  

}

void loop() {
  // Generar señal
  t_actual = micros();
  if (t_actual >= t_anterior + dt) {
    noInterrupts();
    t_anterior = micros();
    if (enabled_sig) {
      t_anterior = micros();
      acum += dt;
      //Serial.println(acum);
      y = m*(float(acum)/1000000) + b; //paso acum a segundos
      //Serial.println(y);
      y = constrain(y,0,Vmax);
      y = map(int(y*100),0,Vmax*100,0,255); // se convierte float de 0.01 a 5 como entero de 1 a 500
      analogWrite(pinMotor,int(y)); // CORREGIR -1 Y >255

      if(acum >= T/2 && rising == true){
        m = -m;
        //Serial.print("m = "); Serial.println(m);
        b = Vmax-m*(float(T/2)/1000000); // intercepto
        //Serial.print("b = "); Serial.println(b);
        rising = false;
      }
      else if(acum >= T && rising == false){
        acum = 0;
        m = -m;
        b = 0;
        rising = true;
      }
    } 
    interrupts();     
  }

  if(verb){
    Serial.print(" V: ");
    Serial.print(measurement_vel);
    Serial.print(" m/s |");
    Serial.print("I: ");
    Serial.print(measurement_i);
    Serial.print(" A | ");
    Serial.print("T: ");
    Serial.print(measurement_temp);
    Serial.print(" C |");
    Serial.println("F: ");
    //Serial.print(measurement_celda);
    //Serial.println("N ");

  }
  else{
    save_data_MEM(measurement_i, measurement_vel, measurement_temp);
  }

  time2 = millis();
  if(time2 > (time1 + 1000)){
    noInterrupts();
    time1 = millis();
    actual_vel_freq = freq*30;
    freq = 0;
    interrupts();    
  }  

  // Inicio de consola
    if(Serial.available() > 0){
    TIMSK1 |= B00000000; //Desenable compare match A
    String comando = Serial.readStringUntil('\n');
    TIMSK1 |= B00000010; //Enable compare match A
    if(comando == "START"){
      enabled_sig = true;
      Serial.print(">> ");
      Serial.println("STIMULUS ACTIVATED");
    }
    else if(comando == "STOP"){
      digitalWrite(pinMotor, LOW);
      enabled_sig = false;
      Serial.print(">> ");
      Serial.println("STIMULUS DISABLED");
    }
    else if(comando.startsWith("MODE STP ")){
      percentage = comando.substring(9).toInt();
      String percentage_1 = comando.substring(9);
      bool percen = valor(percentage_1);
      Serial.print(">> ");
      if(percen){
        Serial.println("Valor invalido");
      }
      else if(percentage < 0){
        Serial.println("Valor invalido");
      }
      else if(percentage > 100){
        Serial.println("Valor invalido");
      }
      else{
        stm = true;
        T = periodo*1000;
        MODE = 0;
        dy = percentage;
        modeInit(MODE);
        Serial.print("DONE STP ");
        Serial.println(percentage);
      }
    }
    else if(comando.startsWith("MODE TRG ")){
      percentage = comando.substring(9).toInt();
      String percentage_1 = comando.substring(9);
      bool percen = valor(percentage_1);
      Serial.print(">> ");
      if(percen){
        Serial.println("Valor invalido");
      }
      else if(percentage < 0){
        Serial.println("Valor invalido");
      }
      else if(percentage > 100){
        Serial.println("Valor invalido");
      }
      else{
        stm = false;
        T = periodo*1000;
        MODE = 1;
        Serial.print("DONE TRG ");
        Serial.println(percentage);
      }
    }
    else if(comando.startsWith("PERIOD ")){
      periodo = comando.substring(7).toInt();
      String periodo_1=comando.substring(7);
      bool per = valor(periodo_1);
      Serial.print(">> ");
      if(per){
        Serial.println("Valor invalido");
      }
      else if(periodo <= 0){
        Serial.println("Valor invalido");
      }
      else{
        T = periodo*1000;
        modeInit(MODE);
        Serial.print("DONE ");
        Serial.println(periodo);
      }
    }
    else if(comando.startsWith("FSAMPLE ")){
      Fs = comando.substring(8).toInt();
      String Fs_1 = comando.substring(8);
      bool fsamp = valor(Fs_1);
      Serial.print(">> ");
      if(fsamp){
        Serial.println("Valor invalido");
      }
      else if(Fs < 0){
        Serial.println("Valor invalido");
      }
      else if(Fs > 65535){
        Serial.println("Escoge un valor menor de frecuencia de muestreo");
      }
      else{
        Serial.print("DONE ");
        Serial.println(Fs);
      }
    }
    else if(comando.startsWith("AVERAGE ")){
      n_samples = comando.substring(8).toInt();
      String n_samples_1 = comando.substring(8);
      bool aver = valor(n_samples_1);
      Serial.print(">> ");
      if(aver){
        Serial.println("Valor invalido");
      }
      else if(n_samples < 0){
        Serial.println("Valor invalido");
      }
      else if(n_samples > 65535){
        Serial.println("Escoge otro valor menor al numero ingresado");
      }
      else{
        Serial.print("DONE ");
        Serial.println(n_samples);
      }
    }
    else if(comando.startsWith("MAXCURRENT ")){
      current_max = comando.substring(11).toInt();
      String current_max_1 = comando.substring(11);
      bool curr = valor(current_max_1);
      Serial.print(">> ");
      if(curr){
        Serial.println("Valor invalido");
      }
      else if(current_max < 0){
        Serial.println("Valor invalido");
      }
      else{
        Serial.print("DONE ");
        Serial.println(current_max);
      }
    }
    else if(comando == "VERBOSE ON"){
      Serial.print(">> ");
      verb = true;
      Serial.println("DONE");
    }
    else if(comando == "VERBOSE OFF"){
      Serial.print(">> ");
      verb = false;
      Serial.println("DONE");      
    }
    else if(comando == "STATE"){
      Serial.println("--------------------");
      if(stm){
        Serial.print(">> ");
        Serial.print("MODE: ");
        Serial.print("STP ");
        Serial.println(percentage);
      }
      else{
        Serial.print(">> ");
        Serial.print("MODE: ");
        Serial.print("TRG ");
        Serial.println(percentage);        
      }
      Serial.print(">> ");
      Serial.print("PERIOD: ");
      Serial.println(periodo);
      Serial.print(">> ");
      Serial.print("FRECUENCY: ");
      Serial.println(1000000/periodo);
      Serial.print(">> ");
      Serial.print("AVERAGE: ");
      Serial.println(n_samples);
      if(verb){
        Serial.print(">> ");
        Serial.println("VERBOSE: ON");
      }
      else{
        Serial.print(">> ");
        Serial.println("VERBOSE: OFF");
      }
      Serial.print(">> ");
      Serial.print("MAXCURRENT: ");
      Serial.println(current_max);
      Serial.println("--------------------");
    }
    else if(comando == "FDATA"){
      noInterrupts();
      extract_data_MEM();
      interrupts();
      //Valores a guardar
    }
    else if(comando == "RESET"){
      current_max = 1;
      Fs = 200;
      n_samples = 1;
      periodo = 500;
      stm = true;
      verb = false;
      percentage = 10;
      dy = percentage;
      T = periodo*1000;
      MODE = 0;
      modeInit(MODE);
    }
    else{
      Serial.print(">> ");
      Serial.println("Comando invalido");
    }

  }

}
//float sense_celda(){
//  float mono = balanza.get_units(20)*(-10);
//  return mono;
//}
float sense_temperature(){
  float voltage = analogRead(A4) * (5.0 / 1023.0);
  float temp = offset_temp + voltage * 100;
  return temp;
}
float sense_current(){
  float voltaje = analogRead(A0) * (5.0 / 1023.0);  
  float current = abs(voltaje -2.5)/sens_current ; // calculate current (-0.12)
  current = current-0.045;
  return current;
}
void update_measurement(){
  if (count == n_samples){
      
      measurement_i = avg_current/n_samples;
      measurement_vel = avg_vel/n_samples;
      measurement_temp = avg_temp/n_samples;
      //measurement_vel = measurement_vel*2*3.1416*0.01/60;
     // measurement_celda = avg_celda / n_samples;

      avg_current = 0;
      avg_vel = 0;
      avg_temp = 0;
      avg_celda = 0;
      count = 0;

  }
  else {
    // sense all variables
    actual_current = sense_current();
    delayMicroseconds(104); // Wait for the ADC to settle
    actual_vel = actual_vel_freq;    
    actual_temp = sense_temperature();
    //delayMicroseconds(104);
    //actual_celda = sense_celda();
    // update average
    //avg_celda += actual_celda;
    avg_current += actual_current;   
    avg_vel += actual_vel;
    avg_temp += actual_temp;
    count += 1;    
  }
}
ISR(TIMER1_COMPA_vect){
  // when actual_time == 1/fs
  TCNT1 = 0;
  update_measurement();
}
void counter_function(){
  freq++;
}

void modeInit(int modeInt){
  /// INICIALIZACION ///
  if (modeInt == 0) {                                               //STEP
    Vmax = 100; // 100%
    // dt = 500000; // dt (duracion de escalon)
    // dy = 10;
    dt = T; // dt -> T (duracion de escalon)
    y_puntos = Vmax/dy;
    T = (2*y_puntos)*dt; // periodo total es T = (2*n%)*dt
  }
  else if (modeInt == 1) {                                          //TRIG
    Vmax = 5; // voltaje maximo 5V
    // T = 2000000; // periodo = 500ms*1000 --> trabajando en us
    dt = T/1000; // T(us)/1000 | puntos NO SE PUEDE TENER UN PERIODO MENOR A 1 ms
  }
  
  m = (Vmax*1000000)/(T/2);
  b=0;
  rising = true; 
}

bool valor(String str) {
  bool hasLetters = false; // Se inicializa un bool en false, lo que indica que no hay caracteres que no sean numeros
  for (int i = 0; i < str.length(); i++) { // Se recorre el String que está como parámetro de la función 
    if (isAlpha(str.charAt(i))) { // Verifica si cada caracter es letra o no 
      hasLetters = true; // Si el caracter es letra, entonces la bandera se vuelve true 
      break; // Se para el programa cuando se detecta una letra
    }
  }
  return hasLetters; // Retorna el bool 
}
void save_data_MEM(float corriente, float rpm,float temperatura){
  EEPROM.put(direccion, corriente);
  direccion += 4;
  EEPROM.put(direccion, rpm);
  //Serial.println(EEPROM.get(eeAddress, resultado),3);
  direccion += 4;
  EEPROM.put(direccion, temperatura);
  direccion += 4;
}

void extract_data_MEM(){
  for (int i = 0; i < EEPROM.length(); i+= 4){  //(apuntador)*4
    Serial.print("Position: ");
    Serial.println(i);
    Serial.println(EEPROM.get(i, resultado),3);
  }
}

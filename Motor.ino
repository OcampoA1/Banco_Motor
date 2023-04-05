#include <string.h>
#include <EEPROM.h>
#include <TimerOne.h>
#define PIN_OUT 10   // Define el pin de salida
//---Varibles corriente ----
float Sensibilidad = 0.139; //< Sensibilidad en voltios/Amperio para sensor de 5A
//---Variables consola ----
bool flag = false; //< Se establece la bandera para que el sistema empiece solo cuando se activa el comando START 
//uint32_t fsample = 200; //< Frecuencia de muestreo por defecto de 200Hz
//int average = 1; //< Numero de muestras por defecto
int current_max = 1; //< Corriente por defecto
uint32_t  periodo = 500; //< Se establece el valor por defecto del periodo
bool verb = false; //< Bandera para VERBOSE
bool mode_1 = true; //<Bandera para tipo de estimulo 
int percentage = 100; //< Percentage_value
bool detener = false;  // Inicializa la bandera de detención en falso
bool modo = true; //Bandera para estimulacion
// ---- Variables sensadas ---- 
uint32_t n_preescaler = (16000000)/256; // desde 1Hz - 62.5kHz
unsigned int Fs = 200; // Ingresado por consola
int interruptPin = 2; // Pin de interrupción
volatile unsigned long freq = 0; //Interrupcion para RPM 
unsigned long time1, time2 = 0;  // Ventana de tiempo RPM 
int count_samples = 0; //Contador para numero de muestras
int n_samples = 1; //Numero de muestras establecido por defecto, se puede modificar por el usuario
unsigned int rpm_avg = 0; //RPM_promedio con muestras --- 4 bytes
unsigned int rpm; //RPM instante
float temp_avg, current_avg = 0; // Corriente promedio --- 8bytes
float Temp, current; //Instante
// -- variables de generación----
/// VARIABLES DE GENERACION ///
#define pinMotor 5
int MODE = 0; // step: 0, trigg: 1
unsigned long int t_actual = 0;
unsigned long int t_anterior = 0;
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


void modeInit(int mode); // Prototipo funcion de generacion

void setup() {
  
  Serial.begin(9600);
  pinMode(pinMotor, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(interruptPin, INPUT_PULLUP);

  cli(); //< Detener interrupciones
  TCCR1A = 0; //pwm, se estbalece en cero, dado que por defecto esta en 1
  TCCR1B = 0; //prescalador, se establece en cero

  // Se inicializa el preescalador a utilizar
  TCCR1B |= B00000100; // Se establece un preescalador de 256, 
  //TCCR1B |= (1 << CS12); //Otra forma de poner en 1 CS12 para que el preescalador sea 256

  //Se habilita la forma de realizar interrupciones (Compare Match)
  TIMSK1 |= B00000010; //Se indica el registro con el que se va realizar la comparación, MODO OCR1A, se pone OCIE1A en 1
  //Serial.print("Este: ");
  //Serial.println(d);
  // Se fija el valor del registro
  OCR1A = n_preescaler/Fs; //Dado que se va contar hasta 5ms, con un preescalador de 256
  //Serial.println(OCR1A);
  sei(); //< Habiulitan interrupciones
  attachInterrupt(digitalPinToInterrupt(interruptPin), counter_function, RISING); // RISING detecta flancos de subida



}
float temperature(){

  Temp = analogRead(A0);
  Temp = ((5.0 * Temp * 100.0)/1023.0)-3.81;// temperatura en C°
  return Temp;

}

float current_measurment(){
  current = analogRead(A1);
  current = current * (5.0 / 1023.0);
  current = ((current-2.5)/0.185)-0.3845;
  return current;

}

void sensado_variables(){

  if(n_samples > count_samples){
    count_samples ++;
    rpm_avg +=rpm;
    temp_avg += temperature(); 
    current_avg += current_measurment();


  }
  else{
    rpm_avg = rpm_avg / n_samples;
    temp_avg = temp_avg / n_samples;
    current_avg = abs(current_avg / n_samples);
    //empuje_avg = empuje_avg/ n_samples;
    //pwm_avg = pwm_avg /n_samples;
    int j=0, z=4, w=5, x=9, h=11;
    while(true){
        EEPROM.put(j,rpm_avg);
        EEPROM.put(z,temp_avg);
        EEPROM.put(w,current_avg);
        EEPROM.put(x,empuje_avg);
        EEPROM.put(h,pwm_avg);
        j = j+12;
        z = z+12;
        w = w+12;
        x = x+12;
        h = h+12;
        if(verb){
          Serial.print("Velocidad: ");
          Serial.print(EEPROM.get(j,rpm_avg));
          Serial.print(", Temperatura: ");
          Serial.print(EEPROM.get(z,temp_avg));
          Serial.print(", Corriente: ");
          Serial.print(EEPROM.get(w,current_avg));
          Serial.print(", Empuje: ");
          Serial.print(EEPROM.get(x,empuje_avg));
          Serial.print(", PWM: ");
          Serial.println(EEPROM.get(h,pwm_avg));
          if(j == 1020){
            j=0;
            z=4;
            w=5;
            x=9;
            h=11;          
          }          

        }
        delay(1000);
     
    }
    //Serial.print(rpm_avg);
    //Serial.print(" RPM, ");
    //Serial.print(temp_avg);
    //Serial.print(" °C, ");
    //Serial.print(current_avg);
    //Serial.println(" A");
    rpm_avg = 0;
    temp_avg = 0;
    current_avg = 0;
    count_samples = 0;
  }


}
ISR(TIMER1_COMPA_vect){
  TCNT1 = 0; // Se reinicia el valor del temporizador para que no cuente hasta el valor maximo
  sensado_variables();
 
}

void counter_function(){
  freq++;
}

void loop() {

  /// GENERACION ///
  t_actual = micros();
  if (t_actual >= t_anterior + dt) {
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
  }


  // -- Inicio de consola --
  if(Serial.available() > 0){
    
    String comando = Serial.readStringUntil('\n');

    if(comando == "START"){
      Serial.print(">> ");
      Serial.println("STIMULUS ACTIVATED");
      enabled_sig = true;      
            
    }
    
    else if(comando == "STOP"){
      Serial.print(">> ");
      digitalWrite(pinMotor, LOW); // establece salida en 0V
      enabled_sig = false;
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
          mode_1 = true;
          T = periodo*1000; // se conserva el ultimo numero ingresado
          MODE = 0;
          dy = percentage;   
          modeInit(MODE);     
          Serial.print("DONE STP ");
          Serial.println(percentage);
          flag = false;

        

        //EEPROM.put(16, percentage);

      }
    }
    else if (comando.startsWith("MODE TRG ")){
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
        if(flag == true){
          mode_1 = false;
          T = periodo*1000;
          MODE = 1;
          modeInit(MODE);
          Serial.print("DONE TRG ");
          Serial.println(percentage);
          flag = false;
        }

        //EEPROM.put(16, percentage);

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
      else if(periodo<=0){
        Serial.println("Valor invalido");
      }
      else{
        Serial.print("DONE ");
        Serial.println(periodo);
        //EEPROM.put(0, periodo);
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
          Serial.println("frecuencia de muestreo negativa");
        }
        else {
          Serial.print("DONE ");
          Serial.println(Fs);
          //EEPROM.put(4,fsample);
        }
      
    
    }
    else if(comando.startsWith("AVERAGE ")){
      n_samples = comando.substring(8).toInt();
      
      String n_samples_1 = comando.substring(8);
      bool aver = valor(n_samples_1);
      Serial.print(">> ");
      if(aver){
        Serial.println("Numero de muestras invalido");
      }
      else if(n_samples < 0){
        Serial.println("Numero de muestras invalido");
      }
      else{
        Serial.print("DONE ");
        Serial.println(n_samples);
        //EEPROM.put(8, average);
      }
    }
    else if(comando.startsWith("MAXCURRENT ")){
      current_max = comando.substring(11).toInt();
      String current_max_1 = comando.substring(11);
      bool curr = valor(current_max_1);
      Serial.print(">> ");
      if(curr){
        Serial.println("Corriente invalida");
      }
      else if(current_max<0){
        Serial.println("Corriente invalida, menor a 0");
      }
      else{
        Serial.print("DONE ");
        Serial.println(current_max);
        //EEPROM.put(12,current);
        
      }
      
    }
    else if(comando == "VERBOSE ON"){
      
      Serial.print(">> ");
      verb = true;
      //Serial.println("DONE");
      //Serial.print(rpm_avg);
      //Serial.print(" RPM, ");
      //Serial.print(temp_avg);
      //Serial.print(" °C, ");
      //Serial.print(current_avg);
      //Serial.println(" A");





    }
    else if (comando == "VERBOSE OFF"){
      
      Serial.print(">> ");      
      verb = false;
      Serial.print("DONE");
      



    }
    else if(comando == "STATE"){
      
      
      Serial.println("------------------------");
      if(mode_1 == true){
        Serial.print(">> ");
        Serial.print("MODE: ");
        Serial.print("STP ");
        Serial.println(percentage);
        //Serial.println(EEPROM.get(16, percentage));

      }
      else{
        Serial.print(">> ");
        Serial.print("MODE: ");
        Serial.print("TRG ");
        Serial.println(percentage);
        //Serial.println(EEPROM.get(16, percentage));          
      }
      
      Serial.print(">> ");
      Serial.print("PERIOD: ");
      Serial.println(periodo);
      //Serial.println(EEPROM.get(0, periodo));
      Serial.print(">> ");
      Serial.print("FRECUENCY: ");
      Serial.println(Fs);
      //Serial.println(EEPROM.get(4, fsample));
      Serial.print(">> ");
      Serial.print("AVERAGE: ");
      Serial.println(n_samples);
      //Serial.println(EEPROM.get(8, average));
      if(verb == true){
        Serial.print(">> ");
        Serial.print("VERBOSE: ");
        Serial.println("ON");
        
      }else{
        Serial.print(">> ");
        Serial.print("VERBOSE: ");
        Serial.println("OFF");

      }
      //Serial.print(">> ");
      //Serial.print("VERBOSE: ");
      //Serial.println(EEPROM.get(9, verb));
      Serial.print(">> ");
      Serial.print("MAXCURRENT: ");
      Serial.println(current_max);
      //Serial.println(EEPROM.get(12, current));
      Serial.println("------------------------");
      
      
    }
    else if(comando == "FDATA"){
      if(verb == true){
        Serial.print(">> ");
        Serial.println("La opcion de VERBOSE estaen ON, no se guardaron datos");
      }
      else{

      }
    }
    else if(comando == "RESET"){
      current_max = 1;
      Fs = 200;
      n_samples = 1;
      periodo = 500;
      mode_1 = true;
      verb = false;
      percentage = 100;
      //EEPROM.put(0, periodo);
     // EEPROM.put(4, fsample);
     // EEPROM.put(8, average);
     // EEPROM.put(12, current);
     // EEPROM.put(16, percentage);
      


    }
    else{
      Serial.print(">> ");
      Serial.println("Comando invalido");
    }
    }
    
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

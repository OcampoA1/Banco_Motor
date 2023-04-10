/**
  * @file Banco_Motor.ino
  *
  * @mainpage Banco de pruebas de motor y hélice
  *
  * @authors Creado por: Alejandro Ocampo Rojas
  *
  * @date 10/04/2023
  *
  * @section descripción Descripción
  * Codigo para controlar banco de motor y hélice a partir de una consola de comandos. Se excita el motor con 2 señales de estimulación (STP, TRG), además, se sensan 5 variables,
  * corriente, temperatura, empuje, porcentaje de dureza y RPM, donde dichos datos se guardan en la memoria EEPROM, y pueden ser mostrados en consola cuando el usuario desee.
  * 
  *
  * @section circuito Circuito
  * - Pin 5 como pin del motor (OUTPUT)
  * - Pin A2 y A3 para la celda de carga (INPUT)
  * - Pin A0 para la corriente (INPUT)
  * - Pin A4 para la temperatura (INPUT)
  * - Pin 2 de interrupción, para RPM
  *
  * @section librerías Librerías
  * - EEPROM
  * - NBHX711
  * - String
  */

// Bibliotecas
#include <string.h>
#include <EEPROM.h>
#include <NBHX711.h>
// Se define el pin del motor
#define pinMotor 5
// Variables para guardar en la memoria
int u=0; //< Variable que se utiliza como contador en la función de extracción de memoria
int x_=0; //< Contador para guardar y extraer de la memoria la variable corriente
int y_=4; //< Contador para guardar y extraer de la memoria la variable rpm
int z_=6;  //< Contador para guardar y extraer de la memoria la varibale empuje 
int w_ = 10; //< Contador para guardar y extraer de la memoria la variable temperatura
int p_ = 14; //< Contador para guardar y extraer de la memoria la variable duty 

//variables consola
uint8_t duty=0; //< Variable que guarda el valor del porcentaje de dureza (1 Byte)
uint8_t percentage = 50; //< Porcentaje de la estimulación
uint32_t periodo = 500; //< Valor del periodo, por defecto 500ms 
uint32_t Fs = 200; //< Valor de la frecuencia de muestreo, por defecto 200Hz
uint16_t n_samples = 1; //< Numero de muestras, por defecto 1 
float current_max = 1; //< Numero de corriente maxima, por defecto 1 Amperio.
bool verb = false; //< Bandera que dice si esta en verbose on u off (false es VERBOSE OFF)
bool stm = true; //< Bandera para saber que tipo de estimulacion se realiza, por defecto true (true = MODE STP)
bool red_flag = false; //< Bandera para mostrar por consola las variables capturadas cuando se cumple el numero de muestras

// Variables de interrupcion y sensorica
int interruptPin =2; //< Pin de interrupción para medir RPM
uint32_t n_preescaler = (16000000)/256; //< Se establece el valor de frecuencia respecto el preescalador de 256 (1Hz - 62.5kHz)
volatile int freq = 0; //<Interrupcion para RPM  
unsigned long time1, time2 = 0;  //< Ventana de tiempo RPM 
float measurement_i = 0; //< Variable que saca la corriente cuando ya se ha dividido por el numero de muestras (4 Bytes)
uint16_t measurement_vel = 0; //< Variable que saca RPM cuando ya se ha dividido por el numero de muestras (2 Bytes)
uint8_t measurement_duty = 0; //< Variable que saca el porcentaje de dureza cuando ya se ha divido por el numero de muestras (1 Byte)
float measurement_temp = 0; //< Variable que saca la temperatura cuando ya se ha divido por el numero de muestras (4 Bytes)
uint16_t actual_vel,actual_vel_freq, avg_vel = 0; //< Se establecen variables que son manipuladas para hallar el RPM del sistema con respecto al numero de muestras 
int count = 0; //< Contador que se compara con el número de muestras
float actual_current, avg_current = 0; //< Variables que se utilizan para calcular la corriente respecto al numero de muestras
float sens_current = 0.185; //< Sensibilidad del sensor de corriente
float actual_temp, avg_temp = 0; //< Variables que se utilizan para calcular la temperatura respecto al numero de muestras
float offset_temp = 0; //< Variable que se utiliza para agregar un offset al valor de la temperatura
float celda = 0; //< Variable que se utiliza para calcular el empuje
float measurement_celda, avg_celda, actual_celda = 0; //< Variables que se utiliza para calcular el empuje respecto al numero de muestras 

// Variables para generar señal 
int MODE = 0; //< STP: 0, TRG: 1
uint32_t t_actual = 0; //< Variable para generar ventana de tiempo dentro del loop
uint32_t t_anterior = 0; //< Variable para generar ventana de tiempo dentro del loop
unsigned long int T = 500000; //< T para TRG, dt para STP   
unsigned long int dy = 10; //< paso de % en STP           
unsigned long int dt; //< duración de estimulación
int Vmax; //< Vmax =5V (TRG) o Vmax = 100% (STP)
unsigned int y_puntos; //< Numero de escalones hasta el maximo
long int m; //< Pendiente de señal triangular; T debe ser mayor que 2 us
unsigned long int acum = 0; //< Variable utilizada para generar estimulos
float y; //variable utilizada para representar la estimulación final en un analogWrite()
int b; //< Variable utilizada para el intercepto
bool rising; //< Variable que se utiliza para verificar el cambio de pendiente
bool enabled_sig = false; //< Dice si el sistema esta activo o no


NBHX711 hx711(A3,A2, 20); //< Se define un objeto para el modulo HX711
void modeInit(int MODE); //< Se establece la función para generar

/** @brief Se inicializa el setup()
    - Se inicializa la función que determina la estimulación  modeInit
    - Se inicializa el conversor HX711 con su respectivo Scale y tare
    - Se inicializa el pin del motor como OUTPUT
    - Se inicializa el pin de sensado de corriente (A0), temperatura (A4) y pin de interrupción
    - Se establece el numero para realizar Compare Match con un preescalador de 256, para poder tener interrupciones para medir las variables sensadas.
    - Se crea ventana de tiempo para calcular RPM
    - Se inicia comunicación serial
 */

void setup() {

  modeInit(MODE); //Se inicializa por defecto la funcion STP
  hx711.begin(); // Se inicializa el objeto
  hx711.setScale(439430.25); // Se agrega el Scale calibrado
  hx711.tare(20); // Se agrega la tare para la celda de carga
  pinMode(pinMotor, OUTPUT); //< Pin para el motor
  pinMode(A0, INPUT); //< A0 sensado de corriente
  pinMode(A4, INPUT); //< A4 sensado de temperatura
  pinMode(interruptPin, INPUT_PULLUP); //< Pin para realizar la interupcción en el sensado de RPM
  attachInterrupt(digitalPinToInterrupt(interruptPin), counter_function, RISING); // Se establece la función para realizar la interrupción para medir RPM
  cli(); //< Detener interrupciones
  TCCR1A = 0; //pwm, se estbalece en cero, dado que por defecto esta en 1
  TCCR1B = 0; //prescalador, se establece en cero

  // Se inicializa el preescalador a utilizar
  TCCR1B |= B00000100; // Se establece un preescalador de 256, 


  //Se habilita la forma de realizar interrupciones (Compare Match)
  TIMSK1 |= B00000010; //Se indica el registro con el que se va realizar la comparación, MODO OCR1A, se pone OCIE1A en 1
  // Se fija el valor del registro
  OCR1A = n_preescaler/Fs; //Dado que se va contar hasta 5ms, con un preescalador de 256
  sei(); // Habiulitan interrupciones
  Serial.begin(9600);// Se inicia comunicación serial
  

}

/** @brief Se inicializa el loop()
    - Se genera la señal de estimulación en una ventana de tiempo
    - Se realiza la consola de comandos
    - Si el VERBOSE esta en ON se imprimen las variables sensadas con respecto al numero de muestras y frecuencia de muestreo
    - Si el VERBOSE esta en OFF se guardan las variables sensadas con respecto al numero de muestras y frecuencia de muestreo
    - Se verifica que la corriente sensada no sobrepase la corriente maxima establecida por el usuario
 */

void loop() {
  // Generar señal
  t_actual = micros(); // Se establece el valor de tiempo actual de ejecución 
  if (t_actual >= t_anterior + dt) { // Se crea la ventana de tiempo
    noInterrupts(); // Se suspenden las interrupciones
    t_anterior = micros(); // Se actualiza la variable con el tiempo actual de ejecución
    if (enabled_sig) { //Se verifica el estadod e la bandera
      t_anterior = micros(); // Se actualiza la variable con el tiempo actual de ejecución
      acum += dt; // Se le suma al acumulador dt
      y = m*(float(acum)/1000000) + b; // Se pasa acum a segundos
      y = constrain(y,0,Vmax); // se limita y entre 0 y vmax
      y = map(int(y*100),0,Vmax*100,0,255); // Se convierte float de 0.01 a 5 como entero de 1 a 500
      duty = (y/255)*100; // Se halla el porcentaje de dureza
      analogWrite(pinMotor,int(y)); // Se activa la señal 

      if(acum >= T/2 && rising == true){ // Se verifica si se llego a la mitad del periodo para cambiar la pendiente
        m = -m; //actualiza valor de pendiente
        b = Vmax-m*(float(T/2)/1000000); // intercepto
        rising = false; // bandera para verificar estimulacion en false
      }
      else if(acum >= T && rising == false){// Se verifica para volver a comenzar la señal
        acum = 0; //se reinicia la variable acum
        m = -m; // se actualiza el valor de m
        b = 0; // reinicia intercepto
        rising = true; // bandera para verificar estimulación en true
      }
    } 
    interrupts(); // Se activan las interrupciones
  }

  //Se imprimen y se guardan datos
  if(red_flag){ //bandera que se activa dentro de la función para calcular el valor de cada variable sensada con respecto a la frecuencia de muestreo y numero de muestras
    if(verb){ //VERBOSE ON
      Serial.print(" Corriente: ");
      Serial.print(measurement_i, 3);
      Serial.print(" A, ");
      Serial.print("RPM: ");
      Serial.print(measurement_vel);
      Serial.print(" rpm, ");
      Serial.print("Empuje: ");
      Serial.print(measurement_celda);
      Serial.print(" N, ");
      Serial.print("Temperatura: ");
      Serial.print(measurement_temp, 3);
      Serial.print(" °C, ");
      Serial.print("Duty: ");
      Serial.print(duty);
      Serial.println(" %");



    }
    else{
        save_data_MEM(measurement_i, measurement_vel, measurement_celda,measurement_temp, duty);
    }
    red_flag = false;

  }
  //Se verifica que la corriente sensada no sobrepase la corriente maxima establecida
  if(measurement_i > current_max){
    digitalWrite(pinMotor, LOW);
    enabled_sig = false;
    Serial.print(">> ");
    Serial.println("Se supera corriente máxima, se recomienda cambiar el MAXCURRENT");
    
  }
  
  // Ventana de tiempo para hallar RPM
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
    TIMSK1 |= B00000000; //Se desactiva el compare match
    String comando = Serial.readStringUntil('\n'); // Se verifiqcan que hayan valores por consola
    TIMSK1 |= B00000010; //Se activa el acompare match

    // Se verifica el comando START
    if(comando == "START"){
      enabled_sig = true;  // Se verifica que comando sea igual a START
      Serial.print(">> "); // Prompt
      Serial.println("STIMULUS ACTIVATED"); //Se indica que la señal se activó
    }

    // Se verifica el comando STOP
    else if(comando == "STOP"){ //Se verifica que comando sea igual a STOP
      digitalWrite(pinMotor, LOW); // Se desactiva la estimulación
      enabled_sig = false;  // Bandera que desactiva el desarrollo de la estimulación
      Serial.print(">> "); //prompt
      Serial.println("STIMULUS DISABLED"); // Se indica que la señal fue desactivada
    }

    //Se verifica el comando MODE STP
    else if(comando.startsWith("MODE STP ")){ //Se verifica que el comando empiece por "MODE STP "
      percentage = comando.substring(9).toInt(); // Se captura el valor del porcentaje de dureza
      String percentage_1 = comando.substring(9); // Se pasa dicho valor a un String para verificar que dicho valor no tenga letras
      bool percen = valor(percentage_1); // Se utiliza la función para verificar que no tenga letras
      Serial.print(">> "); // Se utiliza el prompt
      if(percen){ //Si tiene letras el valor es invalido
        Serial.println("Valor invalido");
      }
      else if(percentage <= 0){ // Si el valor es menor o igual a cero, el valor es invalido
        Serial.println("Valor invalido");
      }
      else if(percentage > 100){ // Si el valor es mayor a 100, el valor es invalido
        Serial.println("Valor invalido");
      }
      else{ // De lo contrario se activa la estimulación
        stm = true; // Bandera que activa estimulación STP
        T = periodo*1000; // Se agrega el periodo para la señal
        MODE = 0; // Modo de la estimulación en STEP
        dy = percentage; // Paso de % en STP
        modeInit(MODE); // Se activa el modo STP
        Serial.print("DONE STP "); //Se indica que se activó el modo STP
        Serial.println(percentage); // Se muestra el procentaje que se eligió 
      }
    }
    //Se verifica el comando MODE TRG
    else if(comando == "MODE TRG"){ // Se verifica que el comando sea igual a MODE TRG
        stm = false; // Bandera que indica que la estimulación TRG esta activa
        T = periodo*1000; // Se indica el periodo para la señal
        MODE = 1; // Modo de la estimulación TRG
        modeInit(MODE); // Se activa el modo TRG
        Serial.print(">> "); // prompt
        Serial.println("DONE TRG"); // Se indica que la señal fue activada
    }
    // Se verifica el comando
    else if(comando.startsWith("PERIOD ")){ //Se verifica el comando inicie con "PERIOD "
      periodo = comando.substring(7).toInt(); //Se captura el valor del periodo
      String periodo_1=comando.substring(7); //El valor del periodo se pasa a String para mirar si tiene letras
      bool per = valor(periodo_1); // Se verifica que no tenga letras el comando
      Serial.print(">> "); // prompt
      if(per){ // Si el valor tiene letras valor invalido
        Serial.println("Valor invalido");
      }
      else if(periodo <= 0){ // Si el valor es menor o igual a 0, valor invalido
        Serial.println("Valor invalido");
      }
      else{ // De lo contrario se activa el comando
        T = periodo*1000; // Se establece el nuevo periodo
        modeInit(MODE); // Se activa el modo en el que estaba con el nuevo periodo
        Serial.print("DONE "); // Se muestra en consola que el comando fue activado
        Serial.println(periodo); // Se muestra el valor
      }
    }
    // Se verifica el comando FSAMPLE
    else if(comando.startsWith("FSAMPLE ")){ // Se verifica que el comando inicie por "FSAMPLE "
      Fs = comando.substring(8).toInt(); //Se captura el valor de frecuencia de muestreo
      String Fs_1 = comando.substring(8); //Se pasa este valor a String
      bool fsamp = valor(Fs_1); // Se verifica que el valor tenga solo numeros
      Serial.print(">> "); //prompt
      if(fsamp){ // Si el valor tiene letras, el valor es invalido
        Serial.println("Valor invalido");
      }
      else if(Fs <= 0){ // Si el valor es menor o igual a 0, es invalido
        Serial.println("Valor invalido");
      }
      else if(Fs > 65535){ //Si el valor es mayor a la frecuencia preestablecida como maxima por el preescalador, el valor es invalido
        Serial.println("Escoja un valor menor de frecuencia de muestreo");
      }
      else{ // De lo contrario se activa el comando
        Serial.print("DONE ");  //Se muestra por serial que el comando fue activo
        Serial.println(Fs); // Se muestra el valor por consola
      }
    }
    // Se verifica el comando AVERAGE
    else if(comando.startsWith("AVERAGE ")){ //Se verifica que el comando inicie por "AVERAGE "
      n_samples = comando.substring(8).toInt(); //Se captura el numero de muestras ingresado
      String n_samples_1 = comando.substring(8); //Se pasa dicho a numero a un String
      bool aver = valor(n_samples_1); // Se verifica que el valor ingresado no tenga letras
      Serial.print(">> "); //prompt
      if(aver){ // Si el valor ingresado tiene letras , entonces es invalido
        Serial.println("Valor invalido");
      }
      else if(n_samples <= 0){ // Si el valor ingresado es menor o igual a 0, valor invalido
        Serial.println("Valor invalido");
      }
      else{ // De lo contrario se activa el comando 
        Serial.print("DONE "); //Se muestra por consola que el comando se activó
        Serial.println(n_samples); // Se muestra el valor por consola
      }
    }
    // Se verifica el comando MAXCURRENT
    else if(comando.startsWith("MAXCURRENT ")){ // Se verifica que el comando inicie por "MAXCURRENT "
      current_max = comando.substring(11).toInt(); // Se captura el valor de la corriente maxima
      String current_max_1 = comando.substring(11); // El valor se pasa a un String
      bool curr = valor(current_max_1); //Se verifica que el valor no tenga letras
      Serial.print(">> "); //prompt
      if(curr){ //Si el valor tiene letras, es un valor invalido
        Serial.println("Valor invalido");
      }
      else if(current_max <= 0){ // Si el valor es menor o igual a cero, valor invalido
        Serial.println("Valor invalido");
      }
      else{ // de lo contrario
        if(current_max == 1.00 ){ //Si la corriente es 1A, recomendar mas corriente
          Serial.print("DONE "); // Se muestra por consola que el comando se activó
          Serial.println(current_max); // Se muestra el valor de corriente maxima por consola
          Serial.print(">> "); //prompr
          Serial.println("Se recomienda un valor de 2 A"); // Se dice que se recomienda un valor de 2 A
        }
        else{// Si la corriente no es 1, se activa el comando 
          Serial.print("DONE "); // Se muestra por consoal que el comando se activó
          Serial.println(current_max); // Se muestra el valor por consola
        }
      }
    }
    // Se verifica el comando VERBOSE ON
    else if(comando == "VERBOSE ON"){ // Si el comando es igual a "VERBOSE ON"
      Serial.print(">> "); //prompt
      verb = true; // Bandera que dice que el VERBOSE ON esta activo
      Serial.println("DONE"); // Se imprime por serial que el comando se activó
    }
    // Se verifica el comando VERBOSE OFF
    else if(comando == "VERBOSE OFF"){ // Se verifica que el comando sea igual a "VERBOSE OFF"
      Serial.print(">> "); //Prompt
      verb = false; // Bandera que dice que el comando VERBOSE OFF esta activo
      Serial.println("DONE"); // Se muestra por consola que el comando se activó       
    }
    // Se verifica el comando STATE
    else if(comando == "STATE"){ //Se verifica que comando sea iguala "STATE"
      Serial.println("--------------------");  // Inicio de interfaz
      if(stm){ // Se verifica la bandera de la estimulación 
        Serial.print(">> "); //prompt
        Serial.println("MODE: STP"); //MODO STP
        
      }
      else{ //De lo contrario es modo TRG
        Serial.print(">> "); //Prompt
        Serial.println("MODE: TRG"); //Modo TRG
               
      }
      Serial.print(">> "); //promtp
      Serial.print("PERIOD: "); //Se muestra periodo
      Serial.println(periodo); //valor del periodo
      Serial.print(">> "); //prompt
      Serial.print("FRECUENCY: "); //frecuencia
      Serial.println(Fs); //Valor de frecuencia de muestreo
      Serial.print(">> "); //prompt
      Serial.print("AVERAGE: "); //muestras
      Serial.println(n_samples); //valor de muestras
      if(verb){ //Se verifica si esta en VERBOSE ON | OFF
        Serial.print(">> "); //Prompt
        Serial.println("VERBOSE: ON"); // Se muestra VERBOSE ON
      }
      else{ //De lo contrario
        Serial.print(">> "); //prompt
        Serial.println("VERBOSE: OFF"); // Se muestra VERBOSE OFF
      }
      Serial.print(">> "); //prompt
      Serial.print("MAXCURRENT: "); // Corriente maxima
      Serial.println(current_max); //valor de corriente maxima
      Serial.println("--------------------");
    }
    // Se verifica el comando FDATA
    else if(comando == "FDATA"){ // Se verifica que el comando sea igual a "FDATA"
      u=0; // Se reinicia variable que se usa para la extracción de la memoria
      Serial.println("DONE "); //Se muestra que el comando se activó
      noInterrupts(); //Se paran las interrupciones
      extract_data_MEM(); //Se invoca la función para extraer de la memoria
      interrupts(); //Se activan las interrupciones
    }
    //Se verifica el comando RESET
    else if(comando == "RESET"){ // Si comando es igual a "RESET"
      current_max = 1; // Se reinicia el valor de corriente maxima
      Fs = 200; // Se reinicia la frecuencia de muestreo
      n_samples = 1; // Se reinicia el average
      periodo = 500; // Se reinicia el periodo
      stm = true; // Se reinicia bandera que dice que el modo es STP
      verb = false; // Se reinicia bandera (VERBOSE OFF)
      percentage = 50; // Porcentaje por defecto de estimulación STP
      dy = percentage; //Se reinicia el paso % STP
      T = periodo*1000; // Se reinicia el periodo de la señal
      MODE = 0; // MODO STP
      modeInit(MODE); // Se graga dicho modo al que elige el modo de estimulación
    }
    //Si el comando no cumple ninguno de los anteriores es comando invalido
    else{
      Serial.print(">> "); //prompt
      Serial.println("Comando invalido"); //Se muestra por consola que el comando es invalido
    }

  }

}

/** @brief Funcion para calcular empuje
    - Se llama el objeto hx711 y se multiplica por 9.8 que es la gravedad para obtener así fuerza
    @return la funcion retorna el valor de empuje
 */
float sense_celda(){
  hx711.update();
  celda = abs(hx711.getUnits(10)-0.303)*(9.8);  // se calcula el valor de la celda con respecto a la escala y se multiplica por la gravedad 0.306
  return celda; // retorna valor de empuje
}

/** @brief Funcion para calcular temperatura
    - Se pasa el valor a voltaje
    - Posteriormente se pasa el valor a temperatura 1°C -> 10mV
    @return la funcion retorna el valor de temperatura
 */
float sense_temperature(){
  float voltage = analogRead(A4) * (5.0 / 1023.0); // Se pasa de valor analogico en voltaje
  float temp = offset_temp + voltage * 100; // se calcula la temperatura respecto al voltaje
  return temp; // retorna valor de temperatura
}
/** @brief Funcion para calcular corriente
    - Se pasa valor a voltaje
    - I = (voltaje -2.5)/sensibilidad
    @return la funcion retorna el valor de corriente
 */
float sense_current(){
  float voltaje = analogRead(A0) * (5.0 / 1023.0);  // se expresa valor analogico como voltaje
  float current = abs(voltaje - 2.5)/sens_current; // se calcula corriente
  current = abs(current-0.045); // desfase
  return current; // se retorna el valor de corriente
}
/** @brief Funcion para calcular las variables sensadas respecto a l numero de muestras y frecuencia de muestreo
    - Se calcula cada variable promediando con el numero de muestras
 */
void update_measurement(){
  if (count == n_samples){
      
      measurement_i = avg_current/n_samples; // se promedia la corriente
      measurement_vel = avg_vel/n_samples; // se promedia RPM
      measurement_temp = avg_temp/n_samples; // se promedia temperatura
      measurement_celda = avg_celda / n_samples; // se promedia valor de empuje
            
      red_flag = true;  //bandera que va a verificar VERBOSE ON

      avg_current = 0; // se reinicia el valor de corriente
      avg_vel = 0; // Se reinicia el valor de RPM
      avg_temp = 0; //Se reinicia el valor de temperatura
      avg_celda = 0; // Se reinicia el valor de la celda
      count = 0; // Se reinicia el contador que se compara con el numero de muestras

  }
  else { // Si no ha llegado a igualar el numero de muestras, se siguen calculando valores
    actual_current = sense_current(); // Se invoca la funcion para calcular corriente
    delayMicroseconds(104); // 13 ciclos de reloj para el ADC
    actual_vel = actual_vel_freq;  // Se sigue calculando RPM   
    actual_temp = sense_temperature(); //Se invoca la función para calcular temperatura
    delayMicroseconds(104); // 13 ciclos de reloj para el ADC 
    actual_celda = sense_celda(); // Se capturan valores de la celda
    // Se agrega valor capturado anteriormente para la variable a promediar 
    avg_celda += actual_celda;  // se suma valor captura de empuje
    avg_current += actual_current;  // se suma valor capturado de corriente  
    avg_vel += actual_vel; // se suma valor caputrado de velocidad
    avg_temp += actual_temp; // se suma valor capturado de temperatura
    count += 1;  // se suma 1 al contador que se compara con el numero de muestras   
  }
}
/** @brief Funcion que se activa cada que se realiza compara match 
    - Se reinicia el timer
    - Se llama la función para calcular la medida de las variables sensadas
 */
ISR(TIMER1_COMPA_vect){
  // Tiempo actual == 1/fs
  TCNT1 = 0; // se reinicia el timer
  update_measurement(); // se llama la funcion para captura la medida de las varibles sensadas
}
/** @brief Funcion para calcular RPM 
    - Se cuenta cada vez que sucede un flanco de subida en la interrupción en el pin 2
 */
void counter_function(){
  freq++; // se cuenta una unidad en variable 
}

/** @brief Funcion que dice que tipo de estimulacion esta activa 
    - Se actualizan datos para cada tipo de estimulacion 
 */
void modeInit(int modeInt){
  /// INICIALIZACION ///
  if (modeInt == 0) {                                               //STEP
    Vmax = 100; // 100%
    dt = T; // dt -> T (duracion de escalon)
    y_puntos = Vmax/dy;
    T = (2*y_puntos)*dt; // periodo total es T = (2*n%)*dt
  }
  else if (modeInt == 1) {                                          //TRIG
    Vmax = 5; // voltaje maximo 5V
    dt = T/1000; // T(us)/1000 | puntos NO SE PUEDE TENER UN PERIODO MENOR A 1 ms
  }
  
  m = (Vmax*1000000)/(T/2);
  b=0;
  rising = true; 
}
/** @brief Funcion para verifica que un numero no tiene letras
    - Se agarra el valor por el usuario en un String y se verifica que no tenga letras
    @return La funcion retorna un bool
 */
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
/** @brief Funcion para guardar en la memoria valores de la sensorica
    - en total son 15 Bytes que ocupa la sensorica
    - Se guarda en diferentes posiciones de memoria de tal forma que no se sobreescriban datos
    - Cada variable a medir, tiene su propia variable que se encarga de moverla por la memoria 
 */
void save_data_MEM(float corriente, uint16_t rpm, float celda, float temperatura, uint8_t pwm){

  EEPROM.put(x_, corriente); //Se guarda el valor de corriente
  EEPROM.put(y_, rpm); // Se guarda el valor de RPM
  EEPROM.put(z_, celda); //Se guarda el valor de empuje
  EEPROM.put(w_, temperatura); // Se guarda el valor de temperatura
  EEPROM.put(p_, pwm); // Se guarda el valor de procentaje de dureza
  x_ += 15; // se suma 15 posiciones para la corriente
  y_ += 15; // se suma 15 posiciones para RPM
  z_ += 15;// Se suma 15 posiciones para empuje
  w_ += 15; // se suman 15 posiciones para temperatura
  p_ += 15; // se suman 15 posiciones para porcentaje de dureza
  if(x_ == 1020){ // Cuando x llegue a 1020, se reinician los valores
    x_=0; //corriente
    y_=4; //rpm
    z_=6;  //empuje
    w_ = 10; //temperatura
    p_ = 14; //duty
  }

 
}
/** @brief Funcion para extraer valores de la memoria 
    - Se recorre la memoria, donde cada sensor tiene su propia variable que recorre la memoria y lo extrae
 */

void extract_data_MEM(){
  while(u<1020){

  Serial.print(EEPROM.get(x_, measurement_i)); // Se imprime valor de corriente en una posicion x
  Serial.print(", ");
  Serial.print(EEPROM.get(y_, measurement_vel)); // Se imprime valor de corriente en una posicion y
  Serial.print(", ");
  Serial.print(EEPROM.get(z_, measurement_celda)); // Se imprime valor de empuje en una posicion z
  Serial.print(", ");
  Serial.print(EEPROM.get(w_, measurement_temp)); // Se imprime valor de temperatura en una posicion w
  Serial.print(", ");
  Serial.println(EEPROM.get(p_, duty)); // Se imprime valor de porcentaje de dureza en una posicion p
  x_ += 15; //recorre para extraer valores de corriente
  y_ += 15; //recorre para extraer valores de RPM
  z_ += 15; //recorre para extraer valores de empuje
  w_ += 15; //recorre para extraer valores de temperatura
  p_ += 15; //recorre para extraer valores de procentaje de dureza
  if(x_ == 1020){ // cuando x sea 1020, se reinician las variables
    u = 1020;
    x_=0; //corriente
    y_=4; //rpm
    z_=6;  //empuje
    w_ = 10; //temperatura 
    p_ = 14; //duty 
  }
}

}

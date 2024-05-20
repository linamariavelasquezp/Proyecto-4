// Librerías
#include <Adafruit_NeoPixel.h>
#include <avr/pgmspace.h>
#ifdef __AVR__
#include <avr/power.h>  // Required for 16 MHz Adafruit Trinket
#endif
 
// Matriz de las BANDERAS
const static uint8_t R[][36] PROGMEM = {
  { 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60 },//COLOMBIA
  { 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60 },//JAVERIANA
  { 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63 },// RUSIA
  { 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48 },//ESPAÑA
 
};
 
// Estructuras de objetos
typedef struct {
  int pasado;
  int presente;
  double dt;
  double t1;
  double t2;
  double f;
  double delayNuestro;
} varVeli;
 
// Variables Globales
const double pi = 3.1416;
const int Elpin = 4;  // Pin del hall
unsigned long p = 0;
int indice = 0;
int j = 0;
int completions = 0;  // Contador para las tiras completadas
 
#define LED_COUNT 36
#define LED_PIN 5
#define LED_PIN2 6
#define LED_PIN3 7
#define LED_PIN4 8
 
// Objetos
varVeli veli;                                                         // Objeto de velocidad de la cicla
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);    // Declare our NeoPixel strip object:
Adafruit_NeoPixel strip2(LED_COUNT, LED_PIN2, NEO_GRB + NEO_KHZ800);  // Declare our NeoPixel strip object:
Adafruit_NeoPixel strip3(LED_COUNT, LED_PIN3, NEO_GRB + NEO_KHZ800);  // Declare our NeoPixel strip object:
Adafruit_NeoPixel strip4(LED_COUNT, LED_PIN4, NEO_GRB + NEO_KHZ800);  // Declare our NeoPixel strip object:
 
// Estados
enum stateVeli {
  VELI,
  ANOTHER
};
stateVeli stateV = VELI;
enum stateMatrix {
  FOR1,
  FOR2
};
stateMatrix imagen = FOR1;
enum delays {
  WAITING,
  FINISH
};
delays delayUs = WAITING;
enum indexes {
  waiting
};
indexes est_man = waiting;
 
// SETUP
void setup() {
  Serial.begin(2000000);
  p = micros();
  setUpVeli(Elpin, &veli);
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  strip.begin();             // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();              // Turn OFF all pixels ASAP
  strip.setBrightness(25);   // Set BRIGHTNESS (max = 255)
  strip2.begin();            // INITIALIZE NeoPixel strip object (REQUIRED)
  strip2.show();             // Turn OFF all pixels ASAP
  strip2.setBrightness(25);  // Set BRIGHTNESS (max = 255)
  strip3.begin();            // INITIALIZE NeoPixel strip object (REQUIRED)
  strip3.show();             // Turn OFF all pixels ASAP
  strip3.setBrightness(25);  // Set BRIGHTNESS (max = 255)
  strip4.begin();            // INITIALIZE NeoPixel strip object (REQUIRED)
  strip4.show();             // Turn OFF all pixels ASAP
  strip4.setBrightness(25);  // Set BRIGHTNESS (max = 255)
}
 
// LOOP
void loop() {
  velocidadInsta(&veli, Elpin);//MAQUINA DE ESTADOS CONCURRENTE PARA LA LECTURA DEL PERIODO
  mostrarImagen(&strip, indice);//MAQUINA DE ESTADOS PARA LA IMPRESION DE LOS NEOPIXELS EN LA TIRA 1
  mostrarImagen(&strip2, indice);//MAQUINA DE ESTADOS PARA LA IMPRESION DE LOS NEOPIXELS EN LA TIRA 2
  mostrarImagen(&strip3, indice);//MAQUINA DE ESTADOS PARA LA IMPRESION DE LOS NEOPIXELS EN LA TIRA 3
  mostrarImagen(&strip4, indice);//MAQUINA DE ESTADOS PARA LA IMPRESION DE LOS NEOPIXELS EN LA TIRA 4
  j++;
  if (completions == 4) {//ESTE CONDICIONAL ES PARA AUMENTAR LOS INDICES HASTA QUE LAS CUATRO TIRAS HALLAN MANDADO SUS RESPECTIVAS INSTRUCCIONES DE EL INDICE ACTUAL
    Delay(&veli);// DELAY EN MAQUINA DE ESTADOS
    manejoInd(indice); // MAQUINA DE ESTADOS PARA EL AUMENTO DE INDICES Y REINICIO DE INDICES
    completions = 0; // REINICIO DE LAS TIRAS COMPLETADAS
  }
}
 
/************************************************************************
*  void velocidadInsta(varVeli *veli, int Elpin)
*
*  Function Return Type: void
*
*  Purpose: Calculate the instantaneous speed taking into account the edges of the hall sensor reading
*
*  Plan: It is not necessary
*
*  Register of Revisions:
*
*  Date       Responsible                  Comment 
*  -----------------------------------------------------------------------
*  20/05/23  Lina María Velásquez, Samuel Niño Gonzalez, Miguel Ángel Medina y María Fernanda Tafur        
Function #2
*********************************************/
void velocidadInsta(varVeli *veli, int Elpin) {
  switch (stateV) {
    case VELI:
      if (veli->pasado == 0 && veli->presente == 1) {
        veli->t2 = micros();
        veli->dt = (veli->t2) - (veli->t1);
        veli->delayNuestro = (veli->dt) / 360;
        veli->t1 = veli->t2;
        veli->pasado = veli->presente;
        veli->presente = digitalRead(Elpin);  // Periodo
      } else {
        stateV = ANOTHER;
      }
      break;
    case ANOTHER:
      if (!(veli->pasado == 0 && veli->presente == 1)) {
        veli->pasado = veli->presente;
        veli->presente = digitalRead(Elpin);
      } else {
        stateV = VELI;
      }
  }
}

/************************************************************************
*  void setUpVeli(int Elpin, varVeli *veli) 
*
*  Function Return Type: void
*
*  Purpose: Hall Sensor Speed Object Constructor
*
*  Plan: It is not necessary
*
*  Register of Revisions:
*
*  Date       Responsible                  Comment 
*  -----------------------------------------------------------------------
*  20/05/23  Lina María Velásquez, Samuel Niño Gonzalez, Miguel Ángel Medina y María Fernanda Tafur        
Function #2
  *********************************************/
void setUpVeli(int Elpin, varVeli *veli) {
  veli->pasado = digitalRead(Elpin);
  veli->presente = 1;
  veli->dt = 0;
  veli->t1 = micros();
  veli->t2 = 0;
  veli->f = 0;
  veli->delayNuestro = 0;
}
 
/************************************************************************
*  void mostrarImagen(Adafruit_NeoPixel *tira, int &in)
*
*  Function Return Type: void
*
*  Purpose: FSM that transforms the 24-bit matrix R[][] into a 6-bit variable, taking into account the three RGB channels.

*  Plan: It is not necessary
*  
*  Register of Revisions:
*
*  Date       Responsible                  Comment 
*  -----------------------------------------------------------------------
*  20/05/23  Lina María Velásquez, Samuel Niño Gonzalez, Miguel Ángel Medina y María Fernanda Tafur        
Function #2
  *********************************************/
void mostrarImagen(Adafruit_NeoPixel *tira, int &in) {
  // Máquina de estado
  switch (imagen) {
    case FOR1://FOR 1 PARA MOVERSE EN LAS FILAS
      if (in < 4) {
        imagen = FOR2;
        p = micros();
      } else if (in > 4) {//SE SUPONE QUE ACA SE DEBE REINCIAR SI ESE VALOR ES MAS QUE 4 PERO NOS SE REALIZA DICHA ACCION
        in = 0;
      }
      break;
 
    case FOR2:
      if (j < 36) {
        uint8_t valor_6bits = pgm_read_byte(&(R[in][j]));
        // Serial.println(j);
        // Extraer los 2 bits de cada canal
        uint32_t canal_rojo_2bits = (valor_6bits >> 4) & 0b11;
        uint32_t canal_verde_2bits = (valor_6bits >> 2) & 0b11;
        uint32_t canal_azul_2bits = valor_6bits & 0b11;
        // Expandir cada canal a 8 bits
        uint32_t canal_rojo = (canal_rojo_2bits << 6) | (canal_rojo_2bits << 4) | (canal_rojo_2bits << 2) | canal_rojo_2bits;
        uint32_t canal_verde = (canal_verde_2bits << 6) | (canal_verde_2bits << 4) | (canal_verde_2bits << 2) | canal_verde_2bits;
        uint32_t canal_azul = (canal_azul_2bits << 6) | (canal_azul_2bits << 4) | (canal_azul_2bits << 2) | canal_azul_2bits;
        uint32_t valor_24bits = (canal_rojo << 16) | (canal_verde << 8) | canal_azul;
        //PASAR NUMERO DE 6 BITS EN 24 BITS
        tira->setPixelColor(j, valor_24bits); //IMPRESION DE LOS LEDS DE LAS COLUMAS DE ESA FILA
        tira->show();
      } else {
        j = 0; //REINICIO DE FOR
        completions++;  // Incrementar el contador cuando una tira complete su actualización
        imagen = FOR1;
      }
      break;
  }
}

/************************************************************************
*  void Delay(varVeli *veli)
*
*  Function Return Type: void
*
*  Purpose: FSM that generates a DELAY from the speed per revolution of the bicycle
*
*  Plan: It is not necessary
* 
*  Register of Revisions:
*
*  Date       Responsible                  Comment 
*  -----------------------------------------------------------------------
*  20/05/23  Lina María Velásquez, Samuel Niño Gonzalez, Miguel Ángel Medina y María Fernanda Tafur        
Function #2
  *********************************************/
void Delay(varVeli *veli) {
  delayUs = WAITING; //DELAY EN MAQUINA DE ESTADOS (SUPOSICION: CREEMOS QUE A PESAR DE PONER CUALQUIER VALOR EN EL PRIMER IF ÉL TIENE UN DELAY MINIMO POR TODAS LAS COSAS QUE HACE)
  switch (delayUs) {
    p = micros();
    case WAITING:
      if (micros() - p > 5000 /*veli->delayNuestro*/) {  //veli->delayNuestro
        p = micros();
 
        delayUs = FINISH;
      } else {
        delayUs = WAITING;
      }
      break;
    case FINISH:
      break;
  }
}

/************************************************************************
*  void manejoInd(int &indice)
*
*  Function Return Type: void
*
*  Purpose: FSM to handle both rising and restarting indices
*
*  Plan: It is not necessary
*
*  Register of Revisions:
*
*  Date       Responsible                  Comment 
*  -----------------------------------------------------------------------
*  20/05/23  Lina María Velásquez, Samuel Niño Gonzalez, Miguel Ángel Medina y María Fernanda Tafur        
Function #2
  *********************************************/
void manejoInd(int &indice) { //AUMENTO DE LOS INDICES Y REINICIO DE LOS MISMOS.
  switch (est_man) {
    case waiting:
      indice ++;
      if (indice >= 4) {
        indice = 0;
      }
      break;
  }
}
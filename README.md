# Hello-worldXDXD
XDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD
#include "ControlMotores.h"
#include "arduino.h"

#define PIN_DIRX   5
#define PIN_PASOX  2

#define PIN_DIRY   6
#define PIN_PASOY  3

#define PIN_DIRZ   7
#define PIN_PASOZ  4

#define X_PASO_HIGH   PORTD |= 0b000000100;
#define X_PASO_LOW    PORTD &= ~0B000000100;

#define Y_PASO_HIGH   PORTD|= 0b000001000;
#define Y_PASO_LOW    PORTD &= ~0B000001000;

#define Z_PASO_HIGH   PORTD|= 0b00010000;
#define Z_PASO_LOW    PORTD &= ~0B00010000;

#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);

#define TIMER2_INTERRUPTS_ON    TIMSK2 |=  (1 << OCIE2A);
#define TIMER2_INTERRUPTS_OFF    TIMSK2 &= ~(1 << OCIE2A);

#define NUMERO_MOTORES  3

volatile byte Bandera_pasos_restantes=0;
volatile byte Bandera_pasos_siguientes=0;
volatile int MOTORES_SIMULTANEOS=2;
volatile bool Motor_simultaneo=false;
volatile bool Movimiento_hecho = false;

    struct InfoPasos {
       float Intervalo_maximo_tiempo;
       volatile unsigned long Intervalo_minimo_paso;
       void (*dirFunc)(int);
       void (*pasoFunc)();
       unsigned c0;
       long Posicion;
       volatile int dir;
       volatile unsigned int Pasos_totales;
       volatile bool Movimiento_hecho = false;
       volatile unsigned int Contador_pasos_rampa;
       volatile unsigned long Pasos_requeridos_para_vmax;
       volatile unsigned long Tiempo_estimado_movimiento;
       volatile unsigned long Tiempo_pasos_rampa;
       volatile float Escala_velocidad;
       volatile unsigned int n;
       volatile float d;
       volatile unsigned long di;
       volatile unsigned int Contador_pasos;

   };
   volatile InfoPasos Escalas[NUMERO_MOTORES];


void xPASO(){
  X_PASO_HIGH
  X_PASO_LOW
}
void xDIR(int dir){
  digitalWrite(PIN_DIRX,dir);
}

void yPASO(){
  Y_PASO_HIGH
  Y_PASO_LOW
}
void yDIR(int dir){
  digitalWrite(PIN_DIRY,dir);
}

void zPASO(){
  Z_PASO_HIGH
  Z_PASO_LOW
}
void zDIR(int dir){
  digitalWrite(PIN_DIRZ,dir);
} 

  ControlVelocidadMotorPasos::ControlVelocidadMotorPasos(){
  pinMode(PIN_PASOX, OUTPUT);
  pinMode(PIN_DIRX, OUTPUT);

  pinMode(PIN_PASOY, OUTPUT);
  pinMode(PIN_DIRY, OUTPUT); 

  pinMode(PIN_PASOZ, OUTPUT);
  pinMode(PIN_DIRZ, OUTPUT);
  
  Escalas[0].Intervalo_maximo_tiempo = 2000;
  Escalas[0].Intervalo_minimo_paso = 110;

  Escalas[1].Intervalo_maximo_tiempo = 2000;
  Escalas[1].Intervalo_minimo_paso = 110;

  Escalas[2].Intervalo_maximo_tiempo = 240;
  Escalas[2].Intervalo_minimo_paso = 27;
 
  Escalas[0].dirFunc = xDIR;
  Escalas[0].pasoFunc = xPASO;


  Escalas[1].dirFunc = yDIR;
  Escalas[1].pasoFunc = yPASO;


  Escalas[2].dirFunc = zDIR;
  Escalas[2].pasoFunc = zPASO;
  }

void Resetear_Pasos(  volatile InfoPasos&si){
  si.c0 = si.Intervalo_maximo_tiempo*.676;
  si.d = si.c0;
  si.di = si.d;
  si.Contador_pasos = 0;
  si.n = 0;
  si.Contador_pasos_rampa = 0;
  Movimiento_hecho = false;
}

void ControlVelocidadMotorPasos::Configuracion_velocidad_max(float velocidadX,float velocidadY,float velocidadZ){
      velocidadX=(16000000/(64*((2200*velocidadX/100))))-1;
      velocidadY=(16000000/(64*(2200*(velocidadY/100))))-1;
      velocidadZ=(16000000/(256*(2200*(velocidadZ/100))))-1;
      Serial.print(velocidadZ);
      volatile unsigned long IntervaloX=velocidadX;
      volatile unsigned long IntervaloY=velocidadY;
      volatile unsigned long IntervaloZ=velocidadZ;
      Escalas[0].Intervalo_maximo_tiempo = 2000;
      Escalas[0].Intervalo_minimo_paso = IntervaloX;

      Escalas[1].Intervalo_maximo_tiempo = 2000;
      Escalas[1].Intervalo_minimo_paso = IntervaloY;

      Escalas[2].Intervalo_maximo_tiempo = 240;
      Escalas[2].Intervalo_minimo_paso = IntervaloZ;
   }


void ControlVelocidadMotorPasos::Preparar_Movimiento(int Numero_motor,long milimetros){
  volatile InfoPasos&si = Escalas[Numero_motor];
  float pasos=milimetros*25;
  si.dirFunc(pasos<0?HIGH:LOW);
  si.dir=pasos>0?1:-1;
  si.Pasos_totales=abs(pasos);
  Resetear_Pasos(si);
  Bandera_pasos_restantes|=(1<<Numero_motor);
  }

void ControlVelocidadMotorPasos::Preparar_Movimiento_Z(int Numero_Motor, long milimetros){
  volatile InfoPasos& si= Escalas[Numero_Motor];
  float Pasos=milimetros*25;
  si.dirFunc(Pasos<0?HIGH:LOW);
  si.dir=Pasos>0?1:-1;
  si.Pasos_totales=abs(Pasos);
  Resetear_Pasos(si);
  Motor_simultaneo=true;
  }


void Siguiente_conjunto_intervalo_interrupcion(){
  unsigned long x=65600;
  for(int i=0;i<MOTORES_SIMULTANEOS;i++){
    if(((1<<i)&Bandera_pasos_restantes)&&Escalas[i].di<x){
      x=Escalas[i].di;
    }
  }
  
  for(int i=0;i<MOTORES_SIMULTANEOS;i++){
    if(((1<<i)&Bandera_pasos_restantes)&&Escalas[i].di==x){
    Bandera_pasos_siguientes|=(1<<i);
    }
  }

  if(Bandera_pasos_restantes==0){
    TIMER1_INTERRUPTS_OFF
    OCR1A=65553;
  }
  OCR1A=x;
}

void Siguiente_conjunto_intervalo_interrupcion_Z(){
  unsigned long z=65553;
  volatile InfoPasos& s = Escalas[2];
      if(s.di<z){
      z=s.di;
    }
  if(Movimiento_hecho==true){
    TIMER2_INTERRUPTS_OFF
    OCR2A=65553;
  }
 OCR2A=z;
}



ISR(TIMER1_COMPA_vect){
  if(Motor_simultaneo){
    TIMER2_INTERRUPTS_ON
  }
  for(int i=0;i<MOTORES_SIMULTANEOS;i++){
volatile InfoPasos& s = Escalas[i];

  if (s.Contador_pasos<s.Pasos_totales){
    s.pasoFunc();
    s.Contador_pasos++;
    s.Posicion+=s.dir;
    if(s.Contador_pasos>=s.Pasos_totales){
      Bandera_pasos_restantes &=  ~(1 << i);
    }
  }

  if(s.Contador_pasos_rampa==0){
    s.n++;
    s.d=s.d-(2*s.d)/(4*s.n+1);

    if (s.d<=s.Intervalo_minimo_paso){
      s.d=s.Intervalo_minimo_paso;
      s.Contador_pasos_rampa=s.Contador_pasos;
    }

    if (s.Contador_pasos>=s.Pasos_totales/2){
      s.Contador_pasos_rampa=s.Contador_pasos;
    }
  }
  else if(s.Contador_pasos>=s.Pasos_totales-s.Contador_pasos_rampa){
    s.d=(s.d*(4*s.n+1))/(4*s.n+1-2);
    s.n--;
  }
  s.di=s.d;
}
Siguiente_conjunto_intervalo_interrupcion();
TCNT1=0;
}

ISR(TIMER2_COMPA_vect){
volatile InfoPasos& s = Escalas[2];

  if (s.Contador_pasos<s.Pasos_totales){
    s.pasoFunc();
    s.Contador_pasos++;
    s.Posicion+=s.dir;
    if(s.Contador_pasos>=s.Pasos_totales){
      Movimiento_hecho=true;
    }
  }

  if(s.Contador_pasos_rampa==0){
    s.n++;
    s.d=s.d-(2*s.d)/(4*s.n+1);

    if (s.d<=s.Intervalo_minimo_paso){
      s.d=s.Intervalo_minimo_paso;
      s.Contador_pasos_rampa=s.Contador_pasos;
    }

    if (s.Contador_pasos>=s.Pasos_totales/2){
      s.Contador_pasos_rampa=s.Contador_pasos;
    }
  }
  else if(s.Contador_pasos>=s.Pasos_totales-s.Contador_pasos_rampa){
    s.d=(s.d*(4*s.n+1))/(4*s.n+1-2);
    s.n--;
  }
  s.di=s.d;
Siguiente_conjunto_intervalo_interrupcion_Z();
TCNT2=0;
}

void ControlVelocidadMotorPasos::Ejecuta_y_espera(){
  Siguiente_conjunto_intervalo_interrupcion();
  Siguiente_conjunto_intervalo_interrupcion_Z();
  volatile InfoPasos&si = Escalas[0];
  if(si.Pasos_totales>0){
  TIMER1_INTERRUPTS_ON
  }
  else if(si.Pasos_totales<0){
    TIMER1_INTERRUPTS_ON
  }
  else if(si.Pasos_totales==0){
    TIMER2_INTERRUPTS_ON
  }
  while(Bandera_pasos_restantes);
  Bandera_pasos_restantes=0;
  Bandera_pasos_siguientes=0;
}

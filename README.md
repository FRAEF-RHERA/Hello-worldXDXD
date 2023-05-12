# Hello-worldXDXD
XDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD
#include "ControlMotores.h"
#include "arduino.h"
nfiguracion_velocidad_max(100,0,0);         
  FL.Preparar_Movimiento(0,500);                     
  FL.Ejecuta_y_espera();                           
  FL.Configuracion_velocidad_max(0,0,100);         
  FL.Preparar_Movimiento_Z(2,500);                 
  FL.Ejecuta_y_espera();                           
}

void loop(){
  FL.Configuracion_velocidad_max(0,0,100);
  FL.Preparar_Movimiento_Z(2,160);
  FL.Ejecuta_y_espera();      
  delay(3000);                            
  FL.Configuracion_velocidad_max(0,0,50);
  FL.Preparar_Movimiento_Z(2,100);
  FL.Ejecuta_y_espera();
  char Tecla = Teclado.getKey();             

















szifojsmn0pi9n0jmfspczsfz<s


el zeus esta aprendio al github       
  if (Tecla != NO_KEY){               
//Librerias necesarias
#include <SPI.h>
#include <LoRa.h>
#include <string.h>
#include <stdio.h>
//Pins necesarios para LoRa funcionar,  CS, RST , G0
int CS = 2 ;
int RST = 4 ; 
int G0 = 3 ;
String mensaje ;

char valores[20];

void setup() {
  Serial.begin(9600);
  while (!Serial);

  //Funcion para setear pines necesarios. 
  LoRa.setPins ( CS, RST, G0) ;

  // Comprobacion de que el LoRa esta bien conectado
  if (!LoRa.begin(915E6)) {
    Serial.println("Fallo el Lora, comprobar conexiones");
    while (1);
  }
    Serial.println("LoRa preparado para recibir");
}

void loop() {
  // Comprueba si hay informacion
  int packetSize = LoRa.parsePacket();
  if (packetSize) { //Si se recibio informacion, parsePacket tomara el valor de 1, por lo tanto sera true, sino tendra el valor de 0. 
    //Serial.print("Se recibio un paquete: ");

    // read packet
    while (LoRa.available()) {
      mensaje = LoRa.readString();
      char val [mensaje.length()];
      strcpy (val,mensaje.c_str());
      char *V1 = strtok (val , ":");
      Serial.print (V1) ;
      char *V2 = strtok (NULL , ",");
      Serial.print (",") ;
      Serial.print (V2) ;
      char *V3 = strtok (NULL , ",");
      Serial.print (",") ;
      Serial.print (V3) ;
      char *V4 = strtok (NULL , ",");
      Serial.print (",") ;
      Serial.print (V4) ;
      char *V5 = strtok (NULL , ",");
      Serial.print (",") ;
      Serial.print (V5) ;
      char *V6 = strtok (NULL , ",");
      Serial.print (",") ;
      Serial.print (V6) ;
      char *V7 = strtok (NULL , ",");
      Serial.print (",") ;
      Serial.print (V7) ;
      char *V8 = strtok (NULL , ",");
      Serial.print (",") ;
      Serial.print (V8) ;

    }

    // print RSSI of packet
    //Serial.print(" with RSSI ");
    Serial.print (",") ;
    Serial.print(LoRa.packetRssi());
    Serial.print (",") ;
    Serial.print(LoRa.packetSnr());
    Serial.print(",");
    Serial.println (LoRa.packetFrequencyError());
  }
}

#include <SPI.h>                       //include the SPI library:
#include "Lora_Library.h"              //include the LoRa Library file of constants variables and functions

void setup()
{
  Program_Setup();                     //do initial program setup
}

void loop()
{
  Serial.print("LoRa For Arduino RX");
  Serial.println();
  LED1Flash(4, 50, 200);		//Flash the LED
  lora_ResetDev();			//Reset the RFM98
  lora_Setup();				//Do the initial LoRa Setup
  lora_SetFreq(434.400);		//Set the RFM98 frequency
  
  Serial.print("Programmed Frequency ");
  Serial.print(lora_GetFreq());
  Serial.println(); 
  
  lora_Print();				//Print the LoRa registers
  lora_Tone(1000, 1000, 5);             //Transmit an FM tone
  lora_SetModem(lora_BW41_7, lora_SF8, lora_CR4_5, lora_Explicit, lora_LowDoptOFF);		//Setup the LoRa modem parameters
  lora_PrintModem();                    //Print the modem parameters
  Serial.println();

  while (1)
  {
    lora_Listen(5);                      //listen for a packet, no timeout
    
    if (lora_FRXOK == 1)                 //if no receive error, print packet contents as ASCII
    {
      lora_RXPKTInfo();                    //print the info on received packet
      lora_RXBuffPrint(lora_PrintASC);   //Print ASCII of packet if no CRC error
    }
    
    if (lora_FCRCerror == 1)                 //if no receive error, print packet contents as ASCII
    {
      lora_RXPKTInfo();                    //print the info on received packet
    }
       
    Serial.println();
  }
}




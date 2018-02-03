#include <SPI.h> // include the SPI library:
#include "Lora_Library.h"            // include the LoRa constants, variables and functions

void setup()
{
  Program_Setup();
}

void loop()
{
  Serial.print("LoRa For Arduino TX");
  Serial.println();
  LED1Flash(4, 50, 200);		//Flash the LED
  lora_ResetDev();			//Reset the device
  lora_Setup();				//Do the initial LoRa Setup
  lora_SetFreq(434.400);		//Set the LoRa
  
  Serial.print("Programmed Frequency ");
  Serial.print(lora_GetFreq());
  Serial.println(); 
  
  lora_Print();			        //Print the LoRa registers
  lora_Tone(1000, 1000, 5);             //Transmit an FM tone
  lora_SetModem(lora_BW41_7, lora_SF8, lora_CR4_5, lora_Explicit, lora_LowDoptOFF);		//Setup the LoRa modem parameters
  lora_PrintModem();                    //Print the modem parameters
  Serial.println();
  lora_FillTX();			//Fill TXbuff with test data
  lora_TXBuffPrint(0);			//Print the TX buffer as ASCII to screen
  while(1) 
  {
  lora_Send(lora_TXStart, lora_TXEnd, 32, 255, 1, 10, 2);	//send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
  lora_TXPKTInfo();			//print packet information
  Serial.println();
  delay(1000);
  }
}




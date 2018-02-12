void setup() {
  // put your setup code here, to run once:

  // Analog to digital conversion
  // ADMUX = 0x40 is for x-axis 0x41 is for y-axis
  ADMUX = 0b01000000;
  ADCSRA = 0b11100000;

  // UART Transmission
  UCSR0B = 0b00001000;
  UCSR0C = 0b00000111;  //asynch, no parity, 1-bit stop,9 bit data
  UBRR0L =0x67;        //0x67 sets baud rate to 9600 using 16 Mhz clock, for 5.6k its 0x10, 0x08 for 115200
  //Serial.begin(9600);
  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:

  //print analog to digital converter values
  Serial.println(ADC);
}

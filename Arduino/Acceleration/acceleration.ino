void setup() {
  // put your setup code here, to run once:
  // ADMUX = 0x40 is for x-axis 0x41 is for y-axis
  ADMUX = 0b01000000;
  ADCSRA = 0b11100000;

  UCSR0C = 0b00000111;  //asynch, no parity, 1-bit stop,9 bit data
  UBRR0L =0x67;        //sets baud rate to 9600 using 16 Mhz clock, for 5.6k its 0x10
  //Serial.begin(9600);
  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:

  //float scaled;

  //map to -3 to 3 because the accelerometer can measure
  //up to 3gs to keep the data in terms of m/s^2
  //scaled = map(ADC,0, 675, -3, 3); 
  Serial.println(ADC);
}

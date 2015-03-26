/*void SonarInit(){
  DDRB |= (1<<PB5);
  TCCR1A = (1<<WGM11)|(1<<COM1A1);
  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10);
  ICR1 = PERIOD_TRIG;  
  OCR1A = 2; 
  TIMSK1 |= (1<<OCIE1A);

  DDRB &= ~(1<<PB6);
  PORTK |= (1<<PB6);
  PCMSK0 |= 1<<PCINT6;
  PCICR |= 1<<0;

}




ISR(PCINT0_vect){
  if (((PINB & 1<<PB6)>>PB6) == 1){
    start = micros();

  }
  else{
    width = (micros() - start);
    //newPing = true;
    //DDRB |= (1<<PB5);
    if (width > 50){
      pingDistCentimeters = width  * 0.017543859;
      pingDistMeters = pingDistCentimeters * 0.01;
      //newPing = true;
      if (width < 17100){
        //if (width < 5700){
        pingDistCentimeters = width  * 0.017543859;
        pingDistMeters = pingDistCentimeters * 0.01;
        newPing = true;
      }
      //DDRB |= (1<<PB5);

    }
  }

}*/

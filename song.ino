 int song = 0;
 
void sing(int s) {
  // iterate over the notes of the melody:
  song = s;
  if (song == 2) {
  //  Serial.println(" 'Underworld Theme'");
    int size = sizeof(underworld_melody) / sizeof(int);
    for (int thisNote = 0; thisNote < size; thisNote++) {
 
      // to calculate the note duration, take one second
      // divided by the note type.
      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
      int noteDuration = 1000 / underworld_tempo[thisNote];
 
      buzz(Buzzer, underworld_melody[thisNote], noteDuration);
      digitalWrite(Lamp,HIGH);
      // to distinguish the notes, set a minimum time between them.
      // the note's duration + 30% seems to work well:
      int pauseBetweenNotes = noteDuration * 1.30;
      delay(pauseBetweenNotes);
 
      // stop the tone playing:
      buzz(Buzzer, 0, noteDuration);
      digitalWrite(Lamp,LOW);
 
    }
 
  } else {
 
    
    int size = sizeof(melody) / sizeof(int);

      int noteDuration = 1000 / tempo[thisNote];
      int pauseBetweenNotes = noteDuration * 1.30;
       buzz(Buzzer, melody[thisNote], noteDuration);
        digitalWrite(Lamp,HIGH);

      currentMillis = millis();
      if (currentMillis - previousMillis > (pauseBetweenNotes*10)) {
      previousMillis = currentMillis;
     
    //  Serial.println(" 'Mario Theme'");
       thisNote++;
      
      }

       
       buzz(Buzzer, 0, noteDuration);
      digitalWrite(Lamp,LOW); 
      
      // to distinguish the notes, set a minimum time between them.
      // the note's duration + 30% seems to work well:
      

      
      
      /*
      while (currentMillis - previousMillis < (pauseBetweenNotes*10)) {
      currentMillis = millis();
      previousMillis = currentMillis;
      }
      */
     // delay(pauseBetweenNotes);
      if(thisNote < size)
      {
      thisNote=0;
      }
      
      // stop the tone playing:

     
 
    
  }
}
 
void buzz(int targetPin, long frequency, long length) {
 // digitalWrite(A3, HIGH);
  long delayValue = 1000000 / frequency / 2; // calculate the delay value between transitions
  //// 1 second's worth of microseconds, divided by the frequency, then split in half since
  //// there are two phases to each cycle
  long numCycles = frequency * length / 1000; // calculate the number of cycles for proper timing
  //// multiply frequency, which is really cycles per second, by the number of seconds to
  //// get the total number of cycles to produce
  for (long i = 0; i < numCycles; i++) { // for the calculated length of time...
    digitalWrite(targetPin, HIGH); // write the buzzer pin high to push out the diaphram
    delayMicroseconds(delayValue); // wait for the calculated delay value
    digitalWrite(targetPin, LOW); // write the buzzer pin low to pull back the diaphram
    delayMicroseconds(delayValue); // wait again or the calculated delay value
  }
//  digitalWrite(A3, LOW);
 
}

void PlayMelody() {
   int size = sizeof(melody) / sizeof(int);
  if (outputTone) {
    // We are currently outputting a tone
    // Check if it's been long enough and turn off if so
 //   Serial.print(currentMillis);
 //   Serial.print(" | ");
 //   Serial.println(previousMillis);
    long int noteDuration = 1000 / tempo[thisNote];
    if (currentMillis - previousMillis >= noteDuration) {
      previousMillis = currentMillis;
      noTone(Buzzer);
      outputTone = false;
 //     Serial.println(noteDuration);
     
    }
     if (thisNote > size) {
        thisNote = 0;
      }
  } else {
    // We are currently in a pause
    // Check if it's been long enough and turn on if so
    
    int noteDuration = 1000 / tempo[thisNote];
      int pauseBetweenNotes = noteDuration * 1.30;
    if (currentMillis - previousMillis >= pauseBetweenNotes) {
      previousMillis = currentMillis;
      
      tone(Buzzer, melody[thisNote]);
      outputTone = true;
      //Update to play the next tone, next time
      thisNote = thisNote + 1;
      
      
 //     Serial.println(pauseBetweenNotes);
    }
  }
}

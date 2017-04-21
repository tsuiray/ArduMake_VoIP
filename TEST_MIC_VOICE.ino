#include "Voice.h"
#include "PCM.h"

#define VOICE_OUT_PWM 3
#define TEST_PIN 4
#define AUDIO_IN  A0
#define SPK_PIN 11

unsigned char Aud_Buf[480] = {0};//20ms has 160's sample in 8KHz sampling
unsigned char* Aud_Buf_Org = &Aud_Buf[0];
unsigned char* Aud_Buf_Cur = &Aud_Buf[0];
unsigned char* Aud_Buf_Read = &Aud_Buf[0];

void setup()
{
  Serial.begin(115200);
  /*
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 2000;            // compare match register 16MHz/8000Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS10);    // No prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
*/
  pinMode(VOICE_OUT_PWM,OUTPUT);
  pinMode(TEST_PIN,OUTPUT);
  pinMode(SPK_PIN,OUTPUT);
  pinMode(AUDIO_IN,INPUT);
  analogReference(DEFAULT);
  Serial.println("Sys Start:");

  //startPlayback(Voice,sizeof(Voice));
}
uint8_t aud_val =0;
void loop() {
  uint16_t i=0;
  uint16_t buf_size =0;
  // put your main code here, to run repeatedly:
  //analogWrite(VOICE_OUT_PWM,*Voice_cur++);
  //if Voice_cur
  #if 0
    startPlayback(Aud_Buf_Read,Aud_Buf_Cur); // 160 sample = 20ms
  
    Aud_Buf_Read += 160;
    if ((Aud_Buf_Read - Aud_Buf_Org) >= sizeof(Aud_Buf))
    {
        Aud_Buf_Read = Aud_Buf_Org;
    }
      Serial.println(*Aud_Buf_Cur);
  #else
    startPlayback(Voice,sizeof(Voice));
  #endif
  delay(500);
}

unsigned char const *sounddata_data=0;
int sounddata_length=0;
volatile uint16_t sample;
byte lastSample;
uint8_t tToggle=0;
// This is called at 8000 Hz to load the next sample.
ISR(TIMER1_COMPA_vect) {
  if (sample >= sounddata_length) {
    if (sample == sounddata_length + lastSample) {
      stopPlayback();
      digitalWrite(SPK_PIN,LOW);
    }
    else {
      // Ramp down to zero to reduce the click at the end of playback.
      OCR2A = sounddata_length + lastSample - sample;
    }
  }
  else {
    OCR2A = pgm_read_byte(&sounddata_data[sample]);
  }
  
  ++sample;
  //tToggle = !tToggle;
  //digitalWrite(TEST_PIN,tToggle);
  //*Aud_Buf_Cur++ = analogRead(AUDIO_IN);

}

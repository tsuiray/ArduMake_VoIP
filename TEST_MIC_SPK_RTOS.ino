#include <Arduino_FreeRTOS.h>
#include <timers.h>
#include "Voice.h"
#include "PCM.h"

// define two tasks for Blink & AnalogRead
void TaskBlink( void *pvParameters );
void TaskAudioGet( void *pvParameters );
void TaskTimer( void *pvParameters );
void TaskPlayVoice( void *pvParameters );

#define VOICE_OUT_PWM 3
#define TEST_PIN 4
#define AUDIO_IN  A0
#define SPK_PIN 11

unsigned char Aud_Buf[480] = {0};//20ms has 160's sample in 8KHz sampling
unsigned char* Aud_Buf_Org = &Aud_Buf[0];
unsigned char* Aud_Buf_Cur = &Aud_Buf[0];
unsigned char* Aud_Buf_Read = &Aud_Buf[0];

unsigned char const *sounddata_data=0;
int sounddata_length=0;
volatile uint16_t sample;
byte lastSample;
uint8_t tToggle=0;
uint8_t aud_buf =0;
uint8_t* Aud_Buf_Cur_2 = &aud_buf;
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
  tToggle = !tToggle;
  digitalWrite(TEST_PIN,tToggle);
  aud_buf = analogRead(AUDIO_IN)/4;
  *Aud_Buf_Cur_2 = aud_buf;
  //*Aud_Buf_Cur++;

}
// the setup function runs once when you press reset or power the board
void setup() {
  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  pinMode(VOICE_OUT_PWM,OUTPUT);
  pinMode(TEST_PIN,OUTPUT);
  pinMode(SPK_PIN,OUTPUT);
  pinMode(AUDIO_IN,INPUT);  
  analogReference(DEFAULT);
  Serial.println("Thread Start:");
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }
  
  // Now set up two tasks to run independently.
  xTaskCreate(
    TaskBlink
    ,  (const portCHAR *)"Blink"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskAudioGet
    ,  (const portCHAR *) "AnalogRead"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  xTaskCreate(
    TaskPlayVoice
    ,  (const portCHAR *) "PlayVoice"
    ,  256  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  NULL );

}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/
void TaskPlayVoice(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for(;;)
  {
      startPlayback(Voice,sizeof(Voice));
      vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
  }
}

void TaskBlink(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, LEONARDO, MEGA, and ZERO 
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN takes care 
  of use the correct LED pin whatever is the board used.
  
  The MICRO does not have a LED_BUILTIN available. For the MICRO board please substitute
  the LED_BUILTIN definition with either LED_BUILTIN_RX or LED_BUILTIN_TX.
  e.g. pinMode(LED_BUILTIN_RX, OUTPUT); etc.
  
  If you want to know what pin the on-board LED is connected to on your Arduino model, check
  the Technical Specs of your board  at https://www.arduino.cc/en/Main/Products
  
  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
  
  modified 2 Sep 2016
  by Arturo Guadalupi
*/

  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  
  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
  }
}

void TaskAudioGet(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  pinMode(TEST_PIN,OUTPUT);
/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.
*/

  for (;;)
  {
    // read the input on analog pin 0:
    //int sensorValue = analogRead(A1);
    // print out the value you read:
    Serial.println(aud_buf);
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

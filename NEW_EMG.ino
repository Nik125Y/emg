

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "Servo.h"
#include "EMGFilters.h"


#define _DEBUG      0

#define SerialToUSB Serial

Servo servo1; //(пин 9)

//переменные для хранения пороговых значений (для определения сотояния мышцы - расслаблена/напряжена)
int threshold1 = 20; 

// Analog input pins
int SensorInputPins[] = {A5};

#define ARR_SIZE(a) (sizeof(a) / sizeof(a[0]))

// Cycle buffer
typedef struct
{
  uint8_t index;
  uint16_t buf[64]; /* Buffer for rectified AC value */
  uint32_t sum;     /* Sum for fast caculation of mean value */
} CycleBuf_t;

// Append to cycle buffer
#define CYCLE_BUF_ADD(cb, val)                    \
  {                                               \
    cb.sum -= cb.buf[cb.index];                   \
    cb.sum += (val);                              \
    cb.buf[cb.index] = (val);                     \
    cb.index = (cb.index + 1) % ARR_SIZE(cb.buf); \
  }

/* Get mean value of cycle buffer */
#define CYCLE_BUF_MEAN(cb) (cb.sum / ARR_SIZE(cb.buf))

CycleBuf_t rectifiedAcBuf[ARR_SIZE(SensorInputPins)];





EMGFilters myFilter[ARR_SIZE(SensorInputPins)];

// Set the input frequency.
//
// The filters work only with fixed sample frequency of
// `SAMPLE_FREQ_500HZ` or `SAMPLE_FREQ_1000HZ`.
// Inputs at other sample rates will bypass
SAMPLE_FREQUENCY sampleRate = SAMPLE_FREQ_500HZ;

// Time interval for processing the input signal. 
unsigned long long interval = 1000000ul / sampleRate;

// Set the frequency of power line hum to filter out.
//
// For countries with 60Hz power line, change to "NOTCH_FREQ_60HZ"
NOTCH_FREQUENCY humFreq = NOTCH_FREQ_50HZ;


void setup() {
  /* add setup code here */
  
  //initialization
  for (int i = 0; i < ARR_SIZE(SensorInputPins); i++) {
    myFilter[i].init(sampleRate, humFreq, true, true, true);
    
    
    rectifiedAcBuf[i].sum = 0;
    rectifiedAcBuf[i].index = 0;

    for (int j = 0; j < ARR_SIZE(rectifiedAcBuf[i].buf); j++)
    {
      rectifiedAcBuf[i].buf[j] = 0;
    }
  }

  // open serial
  SerialToUSB.begin(9600);
  servo1.attach(9);
  servo1.write(120);
}


void loop() {
  // Note: `micros()` will overflow and reset every about 70 minutes.
  unsigned long long timeStamp = micros();

  // filter processing
  int data = 0, dataAfterFilter = 0;

  uint16_t envelope;

  for (int i = 0; i <  ARR_SIZE(SensorInputPins); i++) {
    data = analogRead(SensorInputPins[i]);
    dataAfterFilter = myFilter[i].update(data);

    // Rectification
    CYCLE_BUF_ADD(rectifiedAcBuf[i], abs(dataAfterFilter));

    // Simple envelope calculation, use 2 * rectified value
    envelope = CYCLE_BUF_MEAN(rectifiedAcBuf[i]) * 2;
    // if (i+1 == ARR_SIZE(SensorInputPins)) value = envelope;
  }
  unsigned long timeElapsed = micros() - timeStamp;

  SerialToUSB.println(envelope);
  if(envelope >threshold1 )
  {
    servo1.write(0);
  }
  else 
  {
    servo1.write(120);
  }
  delay(10);
  
}


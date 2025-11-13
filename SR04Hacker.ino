#include <ADC.h>
#include <ADC_util.h>

#define TRIG_PIN 11
#define ECHO_PIN 12
#define ANALOG_PIN A9

//#define COMPUTER_SAMPLE_DEBUG

// Sets the display mode for the onboard OLED
#define DISPLAY_MODE_TEXT
ADC *adc = new ADC();

// maximum number of samples in buffer
#define MAXCOUNT 2000
// buffer variable is global so it can be accessed from the interrupt
uint16_t adcvals[MAXCOUNT];
unsigned int count = 0;  // buffer position

void setup() {
  Serial.begin(115200);

  Serial.println("init");
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(ANALOG_PIN, INPUT);


  //adc->adc0->setReference(ADC_REFERENCE::REF_3V3);

  adc->adc0->setAveraging(1); // disable averaging
  adc->adc0->setResolution(16); // set bits of resolution

  // can be: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED_16BITS, HIGH_SPEED or VERY_HIGH_SPEED
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); // change the conversion speed to high speed 16 bit
  // it can be any of the ADC_MED_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // fastest sampling

  // Enable interrupts so we can easily record the data. DMA would be better but can be implemented later
  adc->adc0->enableInterrupts(adc0_isr);

  Serial.println("SR-04 Hacker Initalized");
}

void loop() {
  // put your main code here, to run repeatedly:
  while (digitalRead(ECHO_PIN));  // wait for sensor to be ready (not sending a pulse)

  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(500);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(TRIG_PIN, LOW);
  // Ping is sent on falling edge of trigger
  // Wait a little so we avoid the internal echos from the sensor
  // This causes a blind spot for about 2-3 inches in front of the sensor, but makes for nicer data
  delayMicroseconds(1100);
  // Start sampling
  adc->adc0->startContinuous(ANALOG_PIN);
  count = 0;

  // Wait for sampling to finish
  while (count < MAXCOUNT) {
    // The ADC interrupt handler seems to sometimes get conufsed if this loop is empty
    delay(1);
  }
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(LED_BUILTIN, LOW);
  adc->adc0->stopContinuous();

  // Send sensor data to computer if computer is listening
#ifdef COMPUTER_SAMPLE_DEBUG
  if (Serial) {
    for (int i = 0; i < MAXCOUNT; i++) {
      uint16_t val = adcvals[i];
      Serial.print(val);
      Serial.print(" ");
      if (i % 256 == 0) Serial.flush(); // Wait for each chunk to send so slower computers can keep up
    }
    Serial.println();
  }
#endif

  // Calculate min/max/average of sensor data
  int minval = adc->adc0->getMaxValue();
  int maxval = 0;
  uint64_t sum = 0;

  for (int i = 0; i < MAXCOUNT; i++) {
    uint16_t val = adcvals[i];
    if (val < minval) minval = val;
    if (val > maxval) maxval = val;
    sum += val;
  }
  float meanval = sum / float(MAXCOUNT);

  // Display mode 1
  // Information about ping and detections



  int ping_index, ping_index_end;
  int thresh = meanval + (maxval - meanval) * 0.5;
  //display.println(maxval-meanval);

  bool found_ping = false;
  int window_size = 11 * 2;

  for (int i = 0; i < MAXCOUNT - window_size; i++) {
    thresh = 1400;
    uint32_t mean = 0;
    uint32_t rms = 0;
    for (int j = i; j < i + window_size; j++) {
      mean +=  adcvals[j];
    }

    mean /= window_size;
    for (int j = i; j < i + window_size; j++) {
      int diff = adcvals[j] - mean;
      diff /= 32;
      rms +=  diff * diff;
    }

    rms /= window_size;

    thresh = (thresh - (i / 10));
    if (rms > thresh && !found_ping) {
      float distance = i * 0.7684 / 2;
      Serial.print(distance);
      
      found_ping = true;
      ping_index = i;
      i += 300; // jump forward 11 samples since 40khz period is ~11 samples at this sample rate
    } else if (rms < thresh && found_ping) {
      ping_index_end = i;
      found_ping = false;
      Serial.print(" ");
     
    }
  }
  Serial.println();

}


// ADC read interrupt
void adc0_isr(void) {
  // Record data if we have space in buffer else throw away the data (but read it to clear flag)
  if (count < MAXCOUNT)
    adcvals[count++] = adc->adc0->analogReadContinuous();
  else adc->adc0->analogReadContinuous();
}

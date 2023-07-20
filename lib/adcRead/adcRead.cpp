#include "adcRead.h"
#include <Arduino.h>

adcRead::adcRead() {}
adcRead::~adcRead() {}

adcRead::adcRead(const float *resistorInput, const float resistorFeedback, const float resistorGround): rf(resistorFeedback), rg(resistorGround) {
    for(int i = 0; i < 3; i++) ri[i] = *(resistorInput + i);

    ads.begin();
    ads.setDataRate(RATE_ADS1115_860SPS);
    ads.setGain(GAIN_TWO);

    fftSample = 32;
    fftFrequency = 80;
    idealFrquency = 10;
    freqBin = idealFrquency* fftSample/ fftFrequency;
    realFFTPlan = fft_init(fftSample, FFT_REAL, FFT_FORWARD, NULL, NULL);
}

void adcRead::sample() {
    u_int64_t timeNow = esp_timer_get_time();
    // int timeNow = esp_timer_get_time();
    for(int sample = 0; sample < fftSample; sample++) {

        while((esp_timer_get_time() - timeNow) < 1e6/fftFrequency) {}
        timeNow = esp_timer_get_time();
        // Serial.print(timeNow);
        // Serial.print(" ");
        for(int pin = 0; pin < 3; pin++) {
            signalVoltage[pin][sample] = ads.computeVolts(ads.readADC_SingleEnded(pin));
        }
    }
}

void adcRead::fft() {
    
    float mag = 0;

    for(int pin = 0; pin < 3; pin++) {
        // Serial.printf("signal(%d, :) = [", pin+1);
        for(int sample = 0; sample < fftSample; sample++) {
            realFFTPlan->input[sample] = signalVoltage[pin][sample];
            // Serial.printf("%f, ", realFFTPlan->input[sample]);
            // if(i == 32) Serial.println("...");
        }
        // Serial.println("];");
        fft_execute(realFFTPlan);
        mag = sqrt(pow(realFFTPlan->output[2* freqBin], 2) + pow(realFFTPlan->output[2* freqBin+ 1], 2));
        
        voltage[pin] = mag*1e3*PI/fftSample;       // 2*Amplifier = 2* (PI/4)* (2* mag/ N)
    }
}

void adcRead::dft() {
    float real, img, mag;
    for(int pin = 0; pin < 3; pin++) {
        real = 0;
        img = 0;
        for(int n = 0; n < fftSample; n++) {
            real += signalVoltage[pin][n]* cos(-2* PI* idealFrquency/ fftFrequency* n);
            img += signalVoltage[pin][n]* sin(-2* PI* idealFrquency/ fftFrequency* n);
        }
        mag = 2* sqrt(pow(real,2)+ pow(img,2))/ fftSample;      // mag = 2* norm(real, img)/ N
        voltage[pin] = 2* PI/ 4* mag* 1e3;       // 2*Amplifier = 2* (PI/4)* mag* 1e3[V -> mV]
    }
}
void adcRead::getSignal(float (&signalVolt)[32]) {
    
    // int timeNow = esp_timer_get_time();
    
    for(int sample = 0; sample < fftSample; sample++) {

        // while((esp_timer_get_time() - timeNow) < 1e6/fftFrequency) {}
        // timeNow = esp_timer_get_time();
        // signalVolt[sample] = ads.computeVolts(ads.readADC_SingleEnded(0))*1e3;
        
        signalVolt[sample] = signalVoltage[0][sample]*1e3;
    }
}

void adcRead::getCurrent(float (&current)[3]) {
    float gain = 1+(rf/rg);
    sample();
    dft();
    // fft();
    for(int pin = 0; pin < 3; pin++) {
        // voltage[pin] = ads.computeVolts(ads.readADC_SingleEnded(pin))* 1e3;
        current[pin] = voltage[pin]* 1e3/ (gain* ri[pin]);        // μA
    }
}

void adcRead::getVolatge(float (&voltage)[3]) {
    fft();
    for(int pin = 0; pin < 3; pin++) {
        voltage[pin] = this->voltage[pin];        // mV
        // voltage[pin] = ads.readADC_SingleEnded(pin)* mVConverter;
        // Serial.printf("%f, ", ads.readADC_SingleEnded(pin)* mVConverter);
    }
}

// float adcRead::current(float inputOffsetVoltage) {
//     fft();
//     float gain = 1+(rf/rg);

//     // float volt = voltMode*1e3/gain - inputOffsetVoltage;   // μV, voltage of input resistor
//     // float volt = voltFFT*1e3/gain - inputOffsetVoltage;   // μV, voltage of input resistor
//     // float curr = volt/ri;        // μA

//     // Serial.printf("volt = %.3f mV, curr = %.3f μA\n", volt, curr/11);
    
//     // return curr;
//     // return voltFFT;
//     // getVoltRead();
//     // return voltRead;
// }
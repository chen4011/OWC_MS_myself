#ifndef REMOTEFETCHDATA_ADCREAD_H_
#define REMOTEFETCHDATA_ADCREAD_H_

#include <esp_adc_cal.h>
#include "fft.h"
#include "Adafruit_ADS1X15.h"

class adcRead
{
private:
    Adafruit_ADS1115 ads;
    float voltage[3];
    float signalVoltage[3][32];
    float ri[3], rf, rg;

    int fftSample;
    int fftFrequency;
    int idealFrquency;
    int freqBin;
    fft_config_t *realFFTPlan;
    
    void sample();
    void fft();
    void dft();
public:
    adcRead();
    ~adcRead();
    
    adcRead(const float *resistorInput, const float resistorFeedback, const float resistorGround);
    
    // float current(float inputOffsetVoltage);       // input offset volatge in μV
    void getCurrent(float (&current)[3]);
    void getVolatge(float (&voltage)[3]);
    void getSignal(float (&signalVolt)[32]);
};

#endif
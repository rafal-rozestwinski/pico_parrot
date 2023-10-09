#include <stdio.h>
#include "pico/stdlib.h"   // stdlib 
#include "hardware/irq.h"  // interrupts
#include "hardware/pwm.h"  // pwm 
#include "hardware/adc.h"
#include "hardware/sync.h" // wait for interrupt 
 
//#define RR_DEBUG 1

// Audio PIN is to match some of the design guide shields. 
#define AUDIO_PIN 28  // you can change this to whatever you like

// -> MIC_PIN 26 // GP26, ADC0
#define ADC_NUM 0
#define ADC_PIN (26 + ADC_NUM)
#define ADC_VREF 3.3
#define ADC_RANGE (1 << 12)
#define ADC_CONVERT (ADC_VREF / (ADC_RANGE - 1))


#define BUFFER_SIZE_IN_SECONDS 15
#define ADC_SAMPLE_RATE 8000
#define SAMPLE_SIZE (sizeof(uint8_t))
#define MIC_SAMPLES_BUFFER_SIZE (BUFFER_SIZE_IN_SECONDS*ADC_SAMPLE_RATE*SAMPLE_SIZE)
#define SLEEP_TIME_US (unsigned)(((float)1/ADC_SAMPLE_RATE)*1000000)
uint8_t MIC_SAMPLES[MIC_SAMPLES_BUFFER_SIZE];
uint32_t MIC_SAMPLES_POS = 0;

volatile uint8_t g_last_sample;

#define VOX_DELTA 30
#define HIGH_VOX_VALUE (125+VOX_DELTA)
#define LOW_VOX_VALUE (125-VOX_DELTA)

/* 
 * This include brings in static arrays which contain audio samples. 
 * if you want to know how to make these please see the python code
 * for converting audio samples into static arrays. 
 */
#include "sample.h"
int wav_position = 0;

/*
 * PWM Interrupt Handler which outputs PWM level and advances the 
 * current sample. 
 * 
 * We repeat the same value for 8 cycles this means sample rate etc
 * adjust by factor of 8   (this is what bitshifting <<3 is doing)
 * 
 */

#define PLAYBACK_ADC 1
void pwm_interrupt_handler() {
    pwm_clear_irq(pwm_gpio_to_slice_num(AUDIO_PIN));    
    #if PLAYBACK_ADC
        pwm_set_gpio_level(AUDIO_PIN, g_last_sample);  
    #else
    if (wav_position < (WAV_DATA_LENGTH<<3) - 1) { 
        // set pwm level 
        // allow the pwm value to repeat for 8 cycles this is >>3 
        pwm_set_gpio_level(AUDIO_PIN, WAV_DATA[wav_position>>3]);  
        wav_position++;
    } else {
        // reset to start
        wav_position = 0;
    }
    #endif
}

int main(void) {
    /* Overclocking for fun but then also so the system clock is a 
     * multiple of typical audio sampling rates.
     */
    stdio_init_all();
    printf("Starting...");


    // AUDIO OUTPUT START

    set_sys_clock_khz(176000, true); 
    gpio_set_function(AUDIO_PIN, GPIO_FUNC_PWM);

    int audio_pin_slice = pwm_gpio_to_slice_num(AUDIO_PIN);

    // Setup PWM interrupt to fire when PWM cycle is complete
    pwm_clear_irq(audio_pin_slice);
    pwm_set_irq_enabled(audio_pin_slice, true);
    // set the handle function above
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_interrupt_handler); 
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // Setup PWM for audio output
    pwm_config config = pwm_get_default_config();
    /* Base clock 176,000,000 Hz divide by wrap 250 then the clock divider further divides
     * to set the interrupt rate. 
     * 
     * 11 KHz is fine for speech. Phone lines generally sample at 8 KHz
     * 
     * 
     * So clkdiv should be as follows for given sample rate
     * ((176000000/11)/250)/8000 = 8.0   (8.0 = it lets pwm to run for 8 samples.)
     *              ^11.0f for 8kHz
     *  8.0f for 11 KHz
     *  4.0f for 22 KHz
     *  2.0f for 44 KHz etc
     */
    pwm_config_set_clkdiv(&config, 11.0f); 
    //pwm_config_set_clkdiv(&config, 8.0f); 
    pwm_config_set_wrap(&config, 250); 
    pwm_init(audio_pin_slice, &config, true);

    pwm_set_gpio_level(AUDIO_PIN, 0);


    // AUDIO OUTPUT STUFF END


    // AUDIO INPUT STUFF BEGIN

    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input( ADC_NUM);
    // Wiring up the device requires 3 jumpers, to connect VCC (3.3v), GND, and AOUT. The example here uses ADC0, which is GP26. Power is supplied from the 3.3V pin.
    // NOTE: uint8_t unsigned samples. Ensure bias of 0.5VCC at the Mic pin!
    uint adc_raw;
    
    bool recording = false;
    while(1) {
        //__wfi(); // Wait for Interrupt
        adc_raw = adc_read(); // raw voltage from ADC
        g_last_sample = adc_raw >> 4; // shift to save only 8 bits
        #ifdef RR_DEBUG
            float adc_f = adc_raw * ADC_CONVERT;
            printf("%.2f %u %u\n", adc_f, (unsigned)adc_raw, (unsigned)g_last_sample);
        #endif
        if(adc_raw > HIGH_VOX_VALUE || adc_raw < LOW_VOX_VALUE) {
            recording = true;
        }
        sleep_us(SLEEP_TIME_US);
    }
}

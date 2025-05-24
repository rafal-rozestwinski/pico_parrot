#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"   // stdlib 
#include "hardware/irq.h"  // interrupts
#include "hardware/pwm.h"  // pwm 
#include "hardware/adc.h"
#include "hardware/sync.h" // wait for interrupt 
 #include "pico/time.h"

//#define RR_DEBUG 1

#define PTT_PIN 21  // you can change this to whatever you like

// Audio PIN is to match some of the design guide shields. 
#define AUDIO_PIN 22  // you can change this to whatever you like

// -> MIC_PIN 26 // GP26, ADC0
#define ADC_NUM 0
#define ADC_PIN (26 + ADC_NUM)
#define ADC_VREF 3.3
#define ADC_RANGE (1 << 12)
#define ADC_CONVERT (ADC_VREF / (ADC_RANGE - 1))


#define BUFFER_SIZE_IN_SECONDS 10
#define ADC_SAMPLE_RATE 11000
//#define ADC_SAMPLE_RATE 8000
//#define ADC_SAMPLE_RATE 44000
#define SAMPLE_SIZE (sizeof(uint8_t))
#define MIC_SAMPLES_BUFFER_SIZE (BUFFER_SIZE_IN_SECONDS*ADC_SAMPLE_RATE*SAMPLE_SIZE)
#define SLEEP_TIME_US (unsigned)(((float)1/ADC_SAMPLE_RATE)*1000000)
volatile uint8_t MIC_SAMPLES[MIC_SAMPLES_BUFFER_SIZE];
volatile uint32_t MIC_SAMPLES_POS = 0;

volatile uint8_t g_last_sample;


uint32_t msSinceBoot;

#define VOX_DELTA 40
#define HIGH_VOX_VALUE (125+VOX_DELTA)
#define LOW_VOX_VALUE (125-VOX_DELTA)

#define STOP_ON_NO_AUDIO 1
#define SECONDS_WITHOUT_AUDIO_TO_STOP_RECORDING 2
#define AVG_SAMPLE_COUNT_TO_STOP_REC (ADC_SAMPLE_RATE*SECONDS_WITHOUT_AUDIO_TO_STOP_RECORDING)
#define MAX_DELTA_TO_INDICATE_STOP 4


#ifndef PICO_DEFAULT_LED_PIN
#error blink example requires a board with a regular LED
#endif
#define LED_PIN PICO_DEFAULT_LED_PIN

#include "sample.h"
int wav_position = 0;
int beep_position = 0;

volatile bool recording = false;
volatile bool playback = false;

uint8_t* play_buffer = NULL;
uint32_t play_max_size = 0;
uint32_t play_pos = 0;

void pwm_interrupt_handler() {
    pwm_clear_irq(pwm_gpio_to_slice_num(AUDIO_PIN));    
    #if PLAYBACK_ADC
        pwm_set_gpio_level(AUDIO_PIN, g_last_sample);  
        return;
    #else
    if(playback) {
        if(play_buffer == NULL) {
            return;
        }
        if (play_pos < (play_max_size<<3) - 1) { 
            // set pwm level 
            // allow the pwm value to repeat for 8 cycles this is >>3 
            pwm_set_gpio_level(AUDIO_PIN, play_buffer[play_pos>>3]);  
            play_pos++;
        } else {
            play_pos = 0;
            pwm_set_gpio_level(AUDIO_PIN, 0);  
            playback = false;
        }
    }
    #endif
}

volatile bool recording_finished = false;
volatile bool finished_playback = true;
bool adc_sample_irq_callback(repeating_timer_t *rt) {
    // Start ADC conversion (assumes channel already selected)
    uint16_t result = adc_read();  // 12-bit result (0-4095)
    g_last_sample = result >> 4;      // Convert to 8-bit (0-255)
    if(recording) {
        gpio_put(LED_PIN, 1);
        if(MIC_SAMPLES_POS > (MIC_SAMPLES_BUFFER_SIZE-10)) {
            // buffer full, force playback sooner than vox.
            recording_finished = true;
            recording = false;
            
        } else {
            MIC_SAMPLES[MIC_SAMPLES_POS] = g_last_sample;
            MIC_SAMPLES_POS++;
        }

    } else {
        if(finished_playback) { // what was recorded, was played back already:
            gpio_put(LED_PIN, 0);
            if((g_last_sample > HIGH_VOX_VALUE || g_last_sample < LOW_VOX_VALUE)) {
                MIC_SAMPLES_POS = 0;
                finished_playback = false;
                recording_finished = false;
                recording = true;
                MIC_SAMPLES[MIC_SAMPLES_POS] = g_last_sample;
                MIC_SAMPLES_POS++;
            }
        }
    }
    return true; // keep repeating
}

void play_sound(volatile uint8_t* buffer, uint32_t size) {
    printf("play_sound buffer=0x%p size=%u...\n", buffer, (unsigned)size);
    sleep_ms(100);
    play_buffer = (uint8_t*)buffer;
    play_max_size = size-1;
    play_pos = 0;
    playback = true;
    while(playback) {
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        sleep_ms(100);
    }
    printf("play_sound complete.\n");
    sleep_ms(100);
}

void playback_sequence() {
    printf("Playback began.\n");
    gpio_put(LED_PIN, 0);
    printf("Press PTT!\n");
    gpio_put(PTT_PIN, 1);
    sleep_ms(100);
    play_sound(WAV_DATA, WAV_DATA_LENGTH/4);
    sleep_ms(100);
    play_sound(MIC_SAMPLES, MIC_SAMPLES_POS);
    sleep_ms(100);
    play_sound(WAV_DATA, WAV_DATA_LENGTH/4);
    sleep_ms(100);
    gpio_put(PTT_PIN, 0);
    printf("Release PTT!\n");
    sleep_ms(100);
    finished_playback = true;
    recording_finished = false;
    MIC_SAMPLES_POS = 0;
}

bool should_stop_recording() {
    #ifdef STOP_ON_NO_AUDIO
    if(recording) {
        int current_pos = MIC_SAMPLES_POS;
        if(current_pos > AVG_SAMPLE_COUNT_TO_STOP_REC) {
            float min_sample =  1000000.0f;
            float max_sample = -1000000.0f;
            for(int i=0; i<AVG_SAMPLE_COUNT_TO_STOP_REC -1 ; i++) {
                float sample = MIC_SAMPLES[current_pos - i - 1];
                if(sample < min_sample) {
                    min_sample = sample;
                }
                if(sample > max_sample) {
                    max_sample = sample;
                }
            }
            float delta = abs(max_sample - min_sample);
            //printf("min %f max %f delta %f\n", min_sample, max_sample, delta);
            if(delta < MAX_DELTA_TO_INDICATE_STOP) {
                printf("min %f max %f delta %f\n", min_sample, max_sample, delta);
                printf("Max delta %f lower then MAX_DELTA_TO_INDICATE_STOP(%f) - stop recording.\n", delta, MAX_DELTA_TO_INDICATE_STOP);
                return true;
            } else {
                //printf("Max delta %f\n", delta);
                return false;
            }
        }
    }
    return false;
    #else
    return false;
    #endif
}

int main(void) {
    stdio_init_all();
    printf("Starting...");
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    // AUDIO OUTPUT START
    set_sys_clock_khz(176000, true); 
    gpio_set_function(AUDIO_PIN, GPIO_FUNC_PWM);
    gpio_init(PTT_PIN);
    gpio_set_dir(PTT_PIN, GPIO_OUT);
    gpio_put(PTT_PIN, 0);
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
    //pwm_config_set_clkdiv(&config, 2.0f); // 44kHz
    //pwm_config_set_clkdiv(&config, 11.0f);  //8kHz
    pwm_config_set_clkdiv(&config, 8.0f); // 11kHz
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
    for(int i=0; i<4; i++) {
        printf("waiting... ");
        gpio_put(LED_PIN, 1);
        sleep_ms(1000);
        gpio_put(LED_PIN, 0);
        sleep_ms(1000);
    }

    static repeating_timer_t timer;
    // 11 kHz sampling -> 1e6 / 11000 = ~91 μs
    add_repeating_timer_us(-91, adc_sample_irq_callback, NULL, &timer);
    printf("Started.\n");
    while(1) {
        msSinceBoot = to_ms_since_boot(get_absolute_time());
        printf("%d ", msSinceBoot);
        printf("STATUS recording_finished=%i recording=%i finished_playback=%i MIC_SAMPLES_POS= %i / %i\n", recording_finished ? 1 : 0, recording ? 1 : 0, finished_playback ? 1 : 0, MIC_SAMPLES_POS, MIC_SAMPLES_BUFFER_SIZE);
        if(!recording) {
            gpio_put(LED_PIN, 1);
            sleep_ms(100);
            gpio_put(LED_PIN, 0);
        }
        if(recording_finished) {
            printf("recording finished\n");
            playback_sequence();
        }
        if(recording) {
            if(should_stop_recording()) {
                printf("should_stop_recording!\n");
                recording_finished = true;
                recording = false;
            }
        }
        sleep_ms(1000);
    }
}

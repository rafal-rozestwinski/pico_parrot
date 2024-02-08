#include <stdio.h>
#include "pico/stdlib.h"   // stdlib 
#include "hardware/irq.h"  // interrupts
#include "hardware/pwm.h"  // pwm 
#include "hardware/adc.h"
#include "hardware/sync.h" // wait for interrupt 
 
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

//#define PLAYBACK_ADC 1

#define BUFFER_SIZE_IN_SECONDS 10
#define ADC_SAMPLE_RATE 11000
//#define ADC_SAMPLE_RATE 8000
//#define ADC_SAMPLE_RATE 44000
#define SAMPLE_SIZE (sizeof(uint8_t))
#define MIC_SAMPLES_BUFFER_SIZE (BUFFER_SIZE_IN_SECONDS*ADC_SAMPLE_RATE*SAMPLE_SIZE)
#define SLEEP_TIME_US (unsigned)(((float)1/ADC_SAMPLE_RATE)*1000000)
uint8_t MIC_SAMPLES[MIC_SAMPLES_BUFFER_SIZE];
uint32_t MIC_SAMPLES_POS = 0;

volatile uint8_t g_last_sample;

#define VOX_DELTA 40
#define HIGH_VOX_VALUE (125+VOX_DELTA)
#define LOW_VOX_VALUE (125-VOX_DELTA)


#ifndef PICO_DEFAULT_LED_PIN
#error blink example requires a board with a regular LED
#endif
#define LED_PIN PICO_DEFAULT_LED_PIN



#include "fs.h"

lfs_t lfs;
lfs_file_t file;


int filesystem_ops() {
    // mount the filesystem
    int err = lfs_mount(&lfs, &PICO_FLASH_CFG);

    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
    if (err) {
        lfs_format(&lfs, &PICO_FLASH_CFG);
        lfs_mount(&lfs, &PICO_FLASH_CFG);
    }

    // read current count
    uint32_t boot_count = 0;
    lfs_file_open(&lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&lfs, &file, &boot_count, sizeof(boot_count));

    // update boot count
    boot_count += 1;
    lfs_file_rewind(&lfs, &file);
    lfs_file_write(&lfs, &file, &boot_count, sizeof(boot_count));

    // remember the storage is not updated until the file is closed successfully
    lfs_file_close(&lfs, &file);

    // release any resources we were using
    lfs_unmount(&lfs);

    // print the boot count
    printf("boot_count: %d\n", boot_count);
    return 0;
}


/* 
 * This include brings in static arrays which contain audio samples. 
 * if you want to know how to make these please see the python code
 * for converting audio samples into static arrays. 
 */
#include "sample.h"
int wav_position = 0;
int beep_position = 0;


bool recording = false;
bool playback = false;

//uint8_t MIC_SAMPLES[MIC_SAMPLES_BUFFER_SIZE];
//uint32_t MIC_SAMPLES_POS = 0;

uint8_t* play_buffer = NULL;
uint32_t play_max_size = 0;
uint32_t play_pos = 0;

/*
 * PWM Interrupt Handler which outputs PWM level and advances the 
 * current sample. 
 * 
 * We repeat the same value for 8 cycles this means sample rate etc
 * adjust by factor of 8   (this is what bitshifting <<3 is doing)
 * 
 */
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

void play_sound(uint8_t* buffer, uint32_t size) {
    printf("play_sound buffer=0x%p size=%u...\n", buffer, (unsigned)size);
    sleep_ms(100);
    play_buffer = buffer;
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


int main(void) {
    /* Overclocking for fun but then also so the system clock is a 
     * multiple of typical audio sampling rates.
     */
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
    uint adc_raw;
    // END OF AUDIO INPUT CONFIGURATION

    for(int i=0; i<4; i++) {
        printf("waiting... ");
        gpio_put(LED_PIN, 1);
        sleep_ms(1000);
        gpio_put(LED_PIN, 0);
        sleep_ms(1000);
    }
    //printf("filesystem_ops()...\n");
    //filesystem_ops();

    printf("Started.\n");
    while(1) {
        //__wfi(); // Wait for Interrupt
        adc_raw = adc_read(); // raw voltage from ADC
        g_last_sample = adc_raw >> 4; // shift to save only 8 bits
        #ifdef RR_DEBUG
            float adc_f = adc_raw * ADC_CONVERT;
            printf("%.2f %u %u\n", adc_f, (unsigned)adc_raw, (unsigned)g_last_sample);
            continue;
        #endif
        //printf("recording? %u\n", (unsigned)recording);
        if((g_last_sample > HIGH_VOX_VALUE || g_last_sample < LOW_VOX_VALUE) && !recording) {
            printf("Recording, sample value: %u\n", (unsigned)g_last_sample);
            recording = true;
            MIC_SAMPLES_POS = 0;
        }
        if(recording) {
            gpio_put(LED_PIN, 1);
            if(MIC_SAMPLES_POS > (MIC_SAMPLES_BUFFER_SIZE-10)) {
                recording = false;
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
                continue;
            }
            MIC_SAMPLES[MIC_SAMPLES_POS] = g_last_sample;
            MIC_SAMPLES_POS++;
            //printf("pos %u \n", MIC_SAMPLES_POS);

        } else {
            //printf("not recording!\n");
        }
        sleep_us(SLEEP_TIME_US);
    }
}

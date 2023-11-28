#include <stdio.h>
#include <string.h>
#include <cmath>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include "WS2812.hpp"
#include <queue>
#include "clocked_input.pio.h"
#include "hardware/structs/bus_ctrl.h"

#define delay sleep_ms
#define LED_PIN_ONE 0
#define LED_PIN_TWO 1
#define LED_LENGTH_ONE 750
#define LED_LENGTH_TWO 750
#define PIO_INPUT_PIN_BASE 26

uint8_t TX_DONE = 22;

bool DRAW_FLAG_ONE = false;
bool DMA_DONE_ONE = false;

PIO pio = pio1;
dma_channel_config c = dma_channel_get_default_config(0);
uint offset = pio_add_program(pio, &clocked_input_program);
uint sm = pio_claim_unused_sm(pio, true);

uint32_t BUFFER[4500];

WS2812 *strip_one;

WS2812 *strip_two;

void displayManagerOne(){

    bool state = true;

    while(true){

        sleep_us(1);

        if(DRAW_FLAG_ONE && DMA_DONE_ONE){

            uint32_t BUFFER_POINTER = 0;

            for (uint16_t led = 0; led < LED_LENGTH_ONE; led++){

                strip_one->setPixelColor(led, 

                    (uint8_t) BUFFER[(LED_LENGTH_ONE*3)+(BUFFER_POINTER++)], 
                    (uint8_t) BUFFER[(LED_LENGTH_ONE*3)+(BUFFER_POINTER++)], 
                    (uint8_t) BUFFER[(LED_LENGTH_ONE*3)+(BUFFER_POINTER++)]
                    
                    );

            }

            strip_one->show();
            printf("Draw string one\r\n");
            DMA_DONE_ONE = false;
            DRAW_FLAG_ONE = false;

        }
    }

}

void displayManagerTwo(){
    uint32_t BUFFER_POINTER = 0;

    for (uint16_t led = 0; led < LED_LENGTH_TWO; led++){

        strip_two->setPixelColor(led, 
        
            (uint8_t) BUFFER[BUFFER_POINTER++], 
            (uint8_t) BUFFER[BUFFER_POINTER++], 
            (uint8_t) BUFFER[BUFFER_POINTER++]

        );

    }

    strip_two->show();
    printf("Draw string two\r\n");
}

void init(){

    stdio_init_all();

    gpio_init(2);
    gpio_set_dir(2, GPIO_OUT);
    gpio_disable_pulls(2);

    gpio_init(16);
    gpio_set_dir(16, GPIO_IN);
    gpio_pull_down(16);

    gpio_init(17);
    gpio_set_dir(17, GPIO_IN);
    gpio_disable_pulls(17);

    gpio_init(18);
    gpio_set_dir(18, GPIO_IN);
    gpio_disable_pulls(18);

    strip_one = new WS2812(
        LED_PIN_ONE,            // Data line is connected to pin 0. (GP0)
        LED_LENGTH_ONE,         // Strip is 6 LEDs long.
        pio0,               // Use PIO 0 for creating the state machine.
        0,                  // Index of the state machine that will be created for controlling the LED strip
                            // You can have 4 state machines per PIO-Block up to 8 overall.
                            // See Chapter 3 in: https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf
        WS2812::FORMAT_GRB  // Pixel format used by the LED strip
    );

    strip_two = new WS2812(
        LED_PIN_TWO,            // Data line is connected to pin 0. (GP0)
        LED_LENGTH_TWO,         // Strip is 6 LEDs long.
        pio0,               // Use PIO 0 for creating the state machine.
        1,                  // Index of the state machine that will be created for controlling the LED strip
                            // You can have 4 state machines per PIO-Block up to 8 overall.
                            // See Chapter 3 in: https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf
        WS2812::FORMAT_GRB  // Pixel format used by the LED strip
    );

}

void messageDone(){

    printf("Message Done\r\n");

    dma_channel_wait_for_finish_blocking(0);
    DMA_DONE_ONE = true;
    DRAW_FLAG_ONE = true;
    dma_channel_configure(0, &c,
            BUFFER,        // Destination pointer
            &pio->rxf[sm],      // Source pointer
            4500, // Number of transfers
            true                // Start immediately
        );

    displayManagerTwo();

    printf("parsed\n");

}

void gpio_callback(uint gpio, uint32_t events) {
    if(gpio==TX_DONE) messageDone();
    //if(gpio==serial_clock) parallelRead();
}

int main() {

    init();

    delay(2000);
    printf("Initializing christmas display...\r\n");

    multicore_launch_core1(displayManagerOne);

    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));

    dma_channel_configure(0, &c,
        BUFFER,        // Destination pointer
        &pio->rxf[sm],      // Source pointer
        4500, // Number of transfers
        true                // Start immediately
    );

    pio_sm_clear_fifos(pio, sm);
    clocked_input_program_init(pio, sm, offset, PIO_INPUT_PIN_BASE);

    gpio_set_irq_enabled_with_callback(TX_DONE, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    //printf("Ready!\r\n");

    while(true) sleep_ms(4);

/*
    printf("Reading from FIFO\r\n");
    for (int i = 0; i < 18; ++i) {
        uint8_t rxdata = pio_sm_get_blocking(pio, sm);
        printf("%02x %s\n", rxdata);
    }
    printf("Done\r\n");
*/

    
    //gpio_set_irq_enabled(serial_latch, GPIO_IRQ_EDGE_RISE, true);

    //while(true);

    /*LED_PIXEL test = {.RED = 128, .GREEN = 255, .BLUE = 128};

    LED_BUFFER.push(test);
    LED_BUFFER.push(test);

    printf("Buffer size: %d\r\n", LED_BUFFER.size());

    LED_PIXEL found = LED_BUFFER.front();
    LED_BUFFER.pop();

    printf("Buffer size: %d\r\n", LED_BUFFER.size());
    printf("Values: %d %d %d\r\n", found.RED, found.GREEN, found.BLUE);

    clearBuffer();
    */

    //while(true) draw(strip_one, strip_two);

}

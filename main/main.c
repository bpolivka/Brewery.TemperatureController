#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define NUM_SENSORS 5

SemaphoreHandle_t conversionDoneSemaphore;

float VoltsPerTick = 2.5 / 4096.0;

void handleGpioInterrupt(void* data)
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    int gpio = *((int*)data);

    xSemaphoreGiveFromISR(conversionDoneSemaphore, &higherPriorityTaskWoken);

    if (higherPriorityTaskWoken == pdTRUE)
        portYIELD_FROM_ISR();
}

void app_main()
{
    conversionDoneSemaphore = xSemaphoreCreateBinary();
    
    esp_err_t ret;
    spi_device_handle_t spi;
    
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE; 
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = 1 << 25;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    
    gpio_install_isr_service(0);

    gpio_isr_handler_add(25, handleGpioInterrupt, (void*)25);

    spi_bus_config_t buscfg = {
        .miso_io_num   = PIN_NUM_MISO,
        .mosi_io_num   = PIN_NUM_MOSI,
        .sclk_io_num   = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1*1000*1000,          // Clock out at 1 MHz (TODO: make 10)
        .mode           = 0,                    // SPI mode 0
        .spics_io_num   = PIN_NUM_CS,           // CS pin
        .queue_size     = 1,                    // We want to be able to queue 1 transaction at a time
    };

    // Initialize the SPI bus
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    assert(ret == ESP_OK);
    
    //Attach the LCD to the SPI bus
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    assert(ret == ESP_OK);

    spi_transaction_t tx;
    memset(&tx, 0, sizeof(tx));

    // Reset
    tx.tx_data[0] = 0x10;
    tx.length = 8;
    tx.flags = SPI_TRANS_USE_TXDATA;
    ret = spi_device_transmit(spi, &tx);
    assert(ret == ESP_OK);

    // Averaging
    tx.tx_data[0] = 0x3c;  // average of 32 converions; AVGON=1, NAVG1=1, NAVG0=1, NSCAN1=0, NSCAN0=0
    tx.length = 8;
    tx.flags = SPI_TRANS_USE_TXDATA;
    ret = spi_device_transmit(spi, &tx);
    assert(ret == ESP_OK);

    uint8_t tx_buffer[NUM_SENSORS * 2];
    uint8_t rx_buffer[NUM_SENSORS * 2];

    memset(&tx_buffer, 0, sizeof(tx_buffer));

    while(1)
    {
        // Reset FIFO
        memset(&tx, 0, sizeof(tx));
        tx.tx_data[0] = 0x18;
        tx.length = 8;
        tx.flags = SPI_TRANS_USE_TXDATA;
        ret = spi_device_transmit(spi, &tx);
        assert(ret == ESP_OK);
    
        // Conversion
        memset(&tx, 0, sizeof(tx));
        tx.tx_data[0] = 0xa0;
        tx.length = 8;
        tx.flags = SPI_TRANS_USE_TXDATA;
        ret = spi_device_transmit(spi, &tx);
        assert(ret == ESP_OK);

        xSemaphoreTake(conversionDoneSemaphore, portMAX_DELAY);
        
        memset(&tx, 0, sizeof(tx));
        tx.length = NUM_SENSORS * 16;
        tx.rxlength = NUM_SENSORS * 16;
        tx.tx_buffer = tx_buffer;
        tx.rx_buffer = rx_buffer;
        ret = spi_device_transmit(spi, &tx);
        assert(ret == ESP_OK);

        printf("----------------------------------------\r\n");
        
        for (int i = 0 ; i < NUM_SENSORS ; ++i)
        {
            uint16_t value = (((uint16_t)rx_buffer[i * 2]) << 8) | ((uint16_t)rx_buffer[(i * 2)+1]);
            float volts = (float)value * VoltsPerTick;
            float temp = volts * 100.0;    
            printf("%d : %04d - %f\r\n", i, value, temp);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

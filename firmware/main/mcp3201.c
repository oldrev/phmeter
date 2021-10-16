#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp32/rom/ets_sys.h"
#include <q/q.h>

#define TAG "MCP3201"

// Código para comunicação SPI entre ESP32 e MCP3201
// Será utilizado a SPI3 que possui a seguinte pinagem:
// MISO -> 19
// MOSI -> 23 (Não será utilizado)
// SCLK -> 18
// CS0+ -> 5

// Para o ADC, F_clock = 16 * F_amostra
// Logo, para uma taxa de amostragem de 10K(2x F_Nyquist)
#define SPI_CLOCK 1600000
#define SPI_MISO 19
#define SPI_SCLK 18
#define SPI_CS0 5
#define HOST_ID SPI3_HOST
#define TAM_PACOTE_ADC 15 // tamanho do pacote em bits

// Handler para o MCP_3201
spi_device_handle_t mcp_3201;
// fila para envio de dados
QueueHandle_t send_data_q;
static void SPI_recv_task();

uint8_t buffer_to_send[100004];

static q_t _vstep;

// Inicia a comunicação SPI
void mcp3201_init()
{
    _vstep = qdiv(qint(2500), qint(4095));

    esp_err_t err;
    // Sinais não utilizados devem ser setados em -1
    timer_config_t timer_cfg = {
        .alarm_en = TIMER_ALARM_DIS,
        .counter_en = TIMER_START,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_DIS,
        .divider = 2,
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &timer_cfg);

    spi_bus_config_t config = {
        .mosi_io_num = -1,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    // Configura interface com dispositivo
    spi_device_interface_config_t dev_config = {
        .command_bits = 0,
        .address_bits = 0,
        .spics_io_num = SPI_CS0,
        .dummy_bits = 0,
        .mode = 0,
        .queue_size = 10,            // Limita o numero de transações na fila a 10
        .clock_speed_hz = SPI_CLOCK, //Define clock de comunicação como 400Khz
        // .pre_cb = SPI_start_mcp
    };
    // Inicia o barramento da SPI3
    err = spi_bus_initialize(HOST_ID, &config, 0);
    // Checa por erro
    if (err == ESP_OK)
    {
        ESP_LOGI("SPI_TAG", "barramento SPI inicializado!");
    }
    else
    {
        ESP_LOGE("SPI_TAG", "Erro: %s", esp_err_to_name(err));
    }
    // Adiciona MCP3201 ao barramento
    err = spi_bus_add_device(HOST_ID, &dev_config, &mcp_3201);
    ESP_ERROR_CHECK(err);

    if (err == ESP_OK)
    {
        ESP_LOGI("SPI_TAG", "MCP3201 adicionado ao barramento!");
    }
    else
    {
        ESP_LOGE("SPI_TAG", "Erro: %s", esp_err_to_name(err));
    }

    //Cria task para comunicação SPI com ADC
    /*
    err = xTaskCreate(SPI_recv_task, "SPI_recv", 4096, NULL, 5, NULL);
    assert(err);

    //Cria fila para envio de dados
    send_data_q = xQueueCreate(10, sizeof(uint8_t *));
    */
    gpio_set_direction(SPI_CS0, GPIO_MODE_OUTPUT);
    gpio_set_level(SPI_CS0, 1);

}
// Função para ler o adc
q_t read_adc_mcp3201()
{
    esp_err_t ret;
    uint16_t read_value;
    spi_transaction_t trans_recv_adc; //Descritor da transação
    memset(&trans_recv_adc, 0, sizeof(trans_recv_adc));
    trans_recv_adc.tx_buffer = NULL,
    trans_recv_adc.flags = SPI_TRANS_USE_RXDATA;
    trans_recv_adc.length = 16;
    /* Antes de iniciar a leitura, coloca em nivel baixo o pino SPI_CS0 para indicar
     * ao MCP3201 que será iniciada uma transação
     */
    gpio_set_level(SPI_CS0, 0);
    ret = spi_device_polling_transmit(mcp_3201, &trans_recv_adc); //Inicia a transação SPI
    /* Após terminar a transação, coloca em nível alto o pino SPI_CS0 para indicar
     * o fim da transação.
     */
    gpio_set_level(SPI_CS0, 1);
    assert(ret == ESP_OK);

    /* O byte mais significativo é recebido no primeiro byte de rx_data
     * O byte menos significativo é recebido no segundo byte de rx_data
     * O valor lido é concatenado na variável de 16 bits read_value
     * O valor então é deslocado 3 casas para direita e 4 casas para esquerda
     * para limpar os bits com lixo
     */
    read_value = ((trans_recv_adc.rx_data[0] << 8) | (trans_recv_adc.rx_data[1]));
    read_value = read_value << 3;
    read_value = read_value >> 4;
    // printf("read value: %d MSB: %d LSB: %d\n", read_value, trans_recv_adc.rx_data[0], trans_recv_adc.rx_data[1]);
    return qmul(qint(read_value), _vstep);
}

static void SPI_recv_task()
{
    uint16_t rec_data;
    int index = 0;
    int32_t time_total = 0;
    double time_init, time_end;
    gpio_set_direction(SPI_CS0, GPIO_MODE_OUTPUT);
    gpio_set_level(SPI_CS0, 1);
    timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &time_init);
    while (1)
    {
        rec_data = read_adc_mcp3201();
        ets_delay_us(75);
        buffer_to_send[index] = (rec_data >> 8); //MSB
        index++;
        buffer_to_send[index] = (rec_data); //LSB
        index++;
        if (index == 100000)
        {
            timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &time_end);
            time_total = (time_end - time_init) * 1000000;
            buffer_to_send[100000] = time_total >> 24;
            buffer_to_send[100001] = time_total >> 16;
            buffer_to_send[100002] = time_total >> 8;
            buffer_to_send[100003] = time_total;
            uint8_t *pvBuffer_to_send = buffer_to_send;
            xQueueSend(send_data_q, &pvBuffer_to_send, portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(5000));
            index = 0;
            timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &time_init);
        }
    }
}

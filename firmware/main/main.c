/* When building using VSCode tooling, the necessary
 variables are not injected in build time.
*/
#include "../build/config/sdkconfig.h"
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include <q/q.h>

#include "mcp3201.h"

#define GPIO_CS GPIO_NUM_5
#define GPIO_SCLK GPIO_NUM_18
#define GPIO_MISO GPIO_NUM_19
#define GPIO_MOSI GPIO_NUM_23

void app_main(void)
{
    mcp3201_init();
    const q_t OPAMP_GAIN = qdiv(qint(191), qint(100));
    const q_t PH_MV_PER_KELVIN = qdiv(198, 1000);
    q_t temp_kelvin = qint(273 + 19);
    q_t MV_PER_PH = qmul(PH_MV_PER_KELVIN, temp_kelvin);

    // int qconv(q_t * q, const char *s);
    char ph_text[32];

    while (true)
    {
        q_t adc_raw_mv = read_adc_mcp3201();
        int iadc_mv = qtoi(adc_raw_mv);

        // 探头输入的信号经过了反向放大了两倍，并且上移了 625 mV
        q_t ph_mv = qdiv(adc_raw_mv, OPAMP_GAIN);
        ph_mv = qnegate(qsub(ph_mv, qint(625)));

        q_t ph = qabs(qadd(qint(-7), qdiv(ph_mv, MV_PER_PH)));
        memset(ph_text, 0, sizeof(ph_text));
        qsprint(ph, ph_text, sizeof(ph_text));
        ESP_LOGI("main", "ADC MCP3201 ADC_Raw=%d mV,\t\tProbe=%d mV\t\tpH=%s", iadc_mv, qtoi(ph_mv), ph_text);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#define BLYNK_PRINT Serial

#include <time.h>
#include <sys/time.h>
#include "esp32/ulp.h"
#include "esp_deep_sleep.h"
#include "driver/rtc_io.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

static RTC_DATA_ATTR struct timeval sleep_enter_time;

// Wi-Fi point
char ssid[] = "MGBot";
char pass[] = "Terminator812";
IPAddress blynk_ip(139, 59, 206, 133);
WiFiClient client;

// Blynk
char auth[] = "ab6ca5b76e9c470396f750999f3d3ddc";

#define WAKE_UP_TIME 10   // Wake up time (s)
#define WORKING_TIME 10   // Working time (s)

long waterCount = 0;     // Суммарные данные со счетчика воды
long esp_water = 0;      // Данные, посчитанные основным процессором
long ulp_water = 0;      // Данные, посчитанные ULP сопроцессором

// Вход импульсов (бит 25 в RTC_GPIO_IN_REG)
int ulp_rtc_io_number = 11;
gpio_num_t gpio_num = GPIO_NUM_0;

// Код для ULP процессора
const ulp_insn_t program[] = {
  I_MOVI(R0, 0),
  I_MOVI(R1, 0),
  I_MOVI(R3, 0),
  M_LABEL(1),
  I_RD_REG(RTC_GPIO_IN_REG, 25, 25),
  I_SUBR(R2, R1, R0),
  M_BXZ(2),
  I_ADDI(R3, R3, 1),
  M_LABEL(2),
  I_MOVR(R1, R0),
  I_MOVI(R2, 16),
  I_ST(R3, R2, 0),
  M_BX(1),
  I_HALT()
};

void setup() {
  // Инициализация UART
  Serial.begin(115200);
  delay(500);

  // Инициализация входа
  gpio_num_t gpio_num = GPIO_NUM_0;
  rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pulldown_dis(gpio_num);
  rtc_gpio_pullup_dis(gpio_num);
  rtc_gpio_pullup_en(gpio_num);
  rtc_gpio_hold_en(gpio_num);

  // Измерение времени сна
  struct timeval now;
  gettimeofday(&now, NULL);
  int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

  // Определение, по какому событию проснулись
  switch (esp_deep_sleep_get_wakeup_cause()) {
    case ESP_DEEP_SLEEP_WAKEUP_TIMER: {
        Serial.println();
        Serial.print("Wake up from timer. Time spent in deep sleep: ");
        Serial.println(sleep_time_ms);
        break;
      }
    case ESP_DEEP_SLEEP_WAKEUP_UNDEFINED:
    default: {
        Serial.println("Not a deep sleep reset");
        memset(RTC_SLOW_MEM, 0, CONFIG_ULP_COPROC_RESERVE_MEM);
      }
  }

  // Подключение к Wi-Fi и серверу Blynk
  /*    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
    delay(500);
    Serial.print(".");
    }  */
  //  Blynk.begin(auth, ssid, pass, blynk_ip, 8442);
  //  Serial.println("");
  //  Serial.println("WiFi connected");
  //  Serial.print("IP address: ");
  //  Serial.println(WiFi.localIP());
  //  Serial.println();

  // Считывание показаний из ULP сопроцессора
  ulp_water = RTC_SLOW_MEM[16] & 0xffff;

  // Отправка данных на сервер Blynk
  waterCount = ulp_water / 2;
  Serial.print("Water counter: ");
  Serial.println(waterCount);
  //  Blynk.virtualWrite(V0, waterCount);
  //  Blynk.run();
  //  Blynk.run();
  //  Blynk.run();
  //  Serial.println("Data has been sent");
  //  Serial.println();

  // Задержка перед уходом в спящий режим (только для тестов)
  //delay(WORKING_TIME * 1000);

  // Очистка памяти RTC для ULP сопроцессора
  memset(RTC_SLOW_MEM, 0, CONFIG_ULP_COPROC_RESERVE_MEM);

  // Запуск выполнения программы в ULP сопроцессоре
  size_t load_addr = 0;
  size_t size = sizeof(program) / sizeof(ulp_insn_t);
  ulp_process_macros_and_load(load_addr, program, &size);
  ulp_run(load_addr);

  // Установка таймера сна
  Serial.print("Enabling timer wakeup: ");
  Serial.println(WAKE_UP_TIME);
  esp_deep_sleep_enable_timer_wakeup(WAKE_UP_TIME * 1000000);

  // Входим в режим сна
  Serial.println("Entering deep sleep");
  gettimeofday(&sleep_enter_time, NULL);
  esp_deep_sleep_start();
}

void loop() {

}

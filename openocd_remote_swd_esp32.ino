// ESP32 implementation of the OpenOCD remote_swd driver.
// Allows a cheap ESP32 to flash an ARM microcontroller using SWD over TCP/IP.
// remote_swd is much faster than the OpenOCD remote_bitbang driver.
//
// Tested using ESP32-C6-MINI:
// - flashing a 48KB ELF firmware image to STM32G0x1 takes 10 seconds.
//
// Prerequisite libraries:
//   - https://github.com/contrem/arduino-timer

#include <arduino-timer.h>
#include <driver/gpio.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <esp_mac.h>
#include <HTTPClient.h>
#include <lwip/sockets.h>
#include <lwipopts.h>
#include <lwip/stats.h>
#include <soc/gpio_struct.h>
#include <WiFi.h>

#include "remote_swd.h"
#include "wifi_password.h"

#if !defined(WIFI_SSID) || !defined(WIFI_PASSWORD)
#error "WIFI_SSID and WIFI_PASSWORD must be defined! Add them to wifi_password.h"
#endif

// Define if we should wait for USB serial port to be opened at boot.
#undef WAIT_FOR_USB_SERIAL_PORT

// Hardware version, for reporting to OpenOCD.
#define HW_VERSION_MAJOR    1
#define HW_VERSION_MINOR    0

// GPIO pins used with the ESP32-C6-MINI board.
// Change these if necessary to match your hardware.
#define GPIO_SWCLK          GPIO_NUM_21
#define GPIO_SWDIO          GPIO_NUM_22
#define GPIO_SWDIO_UNUSED   GPIO_NUM_23

// Activity LED pin
#define LED_PIN             GPIO_NUM_7

static Timer<2> timer;
static bool led_state;
static int wifi_drop_count;
static uint8_t mac_addr[6];
static char mac_addr_str[16];

static void print_wifi_status()
{
  Serial.print("SSID:        ");
  Serial.println(WiFi.SSID());
  Serial.print("RSSI:        ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
  Serial.print("IP Address:  ");
  Serial.println(WiFi.localIP());
}

static void connect_wifi()
{
  // Attempt to connect to Wifi network:
  Serial.print("Attempting to connect to SSID '");
  Serial.print(WIFI_SSID);
  Serial.println("'");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  WiFi.mode(WIFI_MODE_STA);

  // Disable power save to improve WiFi performance.
  // https://github.com/espressif/arduino-esp32/issues/1484
  esp_wifi_set_ps (WIFI_PS_NONE);

  Serial.println();
  Serial.println("Connected to WiFi:");
  print_wifi_status();
}

static bool check_wifi_callback(void* arg)
{
  (void)arg;

  if(WiFi.status() != WL_CONNECTED) {
    if(++wifi_drop_count >= 6) {
      // If we are disconnected for awhile, reboot the ESP32.
      Serial.println("WiFi disconnected. Restarting ESP32...");
      ESP.restart();    // does not return
    }
    else {
      // Otherwise, attempt a reconnect.
      Serial.println("Reconnecting to WiFi...");
      WiFi.disconnect();
      connect_wifi();
    }
  }
  return true;
}

static bool led_off_callback(void* arg)
{
  (void)arg;
  set_led(false);
  return false; // false to stop
}

static inline void set_led(bool enable)
{
  if (enable)
    digitalWrite(LED_PIN, HIGH);
  else
    digitalWrite(LED_PIN, LOW);
}

static inline void blink_led()
{
  // Pulse the LED on for 25 msec.
  set_led(true);
  timer.in(25, led_off_callback);
}

// ----------------------------------------------------------------------------
// SWD callback functions to read/write the SWDIO and SWCLK pins.

static int swdio_swclk_init(void)
{
  // Use the slower gpio_ functions for the initial setup. Later, use direct
  // register access later for toggling the pins quickly.
  gpio_reset_pin(GPIO_SWCLK);
  gpio_reset_pin(GPIO_SWDIO);
  gpio_reset_pin(GPIO_SWDIO_UNUSED);

  // SWCLK as output low.
  gpio_set_level(GPIO_SWCLK, 0);
  gpio_set_direction(GPIO_SWCLK, GPIO_MODE_OUTPUT);

  // SWD as input.
  gpio_set_direction(GPIO_SWDIO, GPIO_MODE_INPUT);
  gpio_set_direction(GPIO_SWDIO_UNUSED, GPIO_MODE_INPUT);

  // SWD as input.
  GPIO.enable_w1tc.enable_w1tc = (1<<GPIO_SWDIO) | (1<<GPIO_SWDIO_UNUSED);

  // SWCLK as output low.
  GPIO.out_w1tc.val = 1<<GPIO_SWCLK;
  GPIO.enable_w1ts.enable_w1ts = 1<<GPIO_SWCLK;
  return 0;
}

static int swdio_input(void)
{
  // Set SWD as input.
  GPIO.enable_w1tc.enable_w1tc = 1<<GPIO_SWDIO;
  return 0;
}

static int swdio_output(void)
{
  // All SWD transfers start as outputs, so this will blink the activity LED
  // for each transfer.
  blink_led();

  // Output SWD low.
  GPIO.out_w1tc.out_w1tc = 1<<GPIO_SWDIO;
  GPIO.enable_w1ts.enable_w1ts = 1<<GPIO_SWDIO;
  return 0;
}

static bool swdio_read(void)
{
  return (GPIO.in.in_data_next & (1<<GPIO_SWDIO));
}

static int swdio_write(bool bit)
{
  if(bit)
    GPIO.out_w1ts.val = 1<<GPIO_SWDIO;
  else
    GPIO.out_w1tc.val = 1<<GPIO_SWDIO;
  return 0;
}

// Issue a short active-high pulse on SWCLK.
static int swclk_send_pulse(void)
{
  // At 80 MHz CPU and 40 MHz flash on ESP32-C6,
  // this results in a pulse appx 280 ns wide, for appx 1 MHz clock rate.
  GPIO.out_w1ts.val = 1<<GPIO_SWCLK;
  asm volatile("nop\n");
  GPIO.out_w1tc.val = 1<<GPIO_SWCLK;
  return 0;
}

// ----------------------------------------------------------------------------

void setup()
{
  // Init LED.
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize USB serial port for debugging.
  Serial.begin(115200);
#ifdef WAIT_FOR_USB_SERIAL_PORT
  while (!Serial);
#endif
  Serial.print("\nESP32 remote_swd booting (HW version ");
  Serial.print(HW_VERSION_MAJOR);
  Serial.print(".");
  Serial.print(HW_VERSION_MINOR);
  Serial.print(", SW version 0x");
  Serial.print(REMOTE_SWD_SW_VERSION, HEX);
  Serial.println(") ...");

  // MAC address is unique for every ESP32 device. Use it as a UID
  // when reporing data.
  esp_read_mac(mac_addr, ESP_MAC_WIFI_STA);
  snprintf(mac_addr_str, sizeof(mac_addr_str), "%02X%02X%02X%02X%02X%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4],
           mac_addr[5]);
  uint32_t serial_number = 0;
  serial_number |= mac_addr[2]; serial_number <<= 8;
  serial_number |= mac_addr[3]; serial_number <<= 8;
  serial_number |= mac_addr[4]; serial_number <<= 8;
  serial_number |= mac_addr[5];

  // Connect to WiFi and monitor it every 10 seconds.
  connect_wifi();
  timer.every(10000, check_wifi_callback);

  // Initialize the SWD library.
  uint16_t hw_version = ((HW_VERSION_MAJOR & 0xFF) << 8) |
                         (HW_VERSION_MINOR & 0xFF);
  int ret = remote_swd_server_init(
              serial_number, hw_version, REMOTE_SWD_TCP_PORT,
              swdio_swclk_init, swdio_input, swdio_output,
              swdio_write, swdio_read, swclk_send_pulse);
  if(ret < 0)
    fprintf(stderr, "Failed initializing remote_swd server.\n");
}

void loop()
{
  // Run the SWD server.
  remote_swd_server_process();

  // Update the timer.
  timer.tick();
}

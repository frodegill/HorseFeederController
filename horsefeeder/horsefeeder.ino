//#define GPIO_DEBUG

#ifdef GPIO_DEBUG
# include <gpio_viewer.h>
  GPIOViewer gpio_viewer;
#endif

#include <Servo.h>
#include <WiFi.h>


unsigned int clock_rollover = 0;
unsigned long previous_time_ms = 0L;

constexpr uint8_t SERVO_COUNT = 4;
Servo servos[SERVO_COUNT];
constexpr int SERVO_CLOSED_DEG = 0;
constexpr int SERVO_OPEN_DEG = 180;

constexpr uint8_t active_feeders_led_pins[SERVO_COUNT] = {19,21,23,22};
constexpr uint8_t hour_led_pins[6] = {32,33,25,26,27,14};
constexpr uint8_t servo_control_pins[SERVO_COUNT] = {18,5,17,16};
constexpr uint8_t button_pin = 13;

constexpr uint8_t LED_OFF = 0;
constexpr uint8_t LED_ON = 255;

bool button_was_pressed = false;
unsigned long previous_buttonpress_ds = 0L;
constexpr unsigned long buttonpress_grace_period_ds = 2L;
constexpr unsigned long ONE_HOUR_IN_DS = 60*60*10L;

volatile uint8_t active_feeders = 0;

int previous_hours_left = 0;


unsigned long getDecisecondsSinceBoot() {
  unsigned long current_time_ms = millis();
  if (current_time_ms < previous_time_ms) {
    clock_rollover++;
  }
  previous_time_ms = current_time_ms;

  return (static_cast<unsigned long>(-1)/100.0)*clock_rollover + current_time_ms/100.0; 
}

void updateActiveFeedersLeds() {
  for (auto i=0; i<sizeof(active_feeders_led_pins)/sizeof(active_feeders_led_pins[0]); i++)
    analogWrite(active_feeders_led_pins[i], i<active_feeders ? LED_ON : LED_OFF);
}

void updateHourLeds() {
  for (auto i=0; i<sizeof(hour_led_pins)/sizeof(hour_led_pins[0]); i++)
    analogWrite(hour_led_pins[i], i<previous_hours_left ? LED_ON : LED_OFF);
}

void activateFeeder() {
  if (active_feeders > 0) {
    servos[active_feeders-1].write(SERVO_OPEN_DEG);
    delay(2000);
    servos[active_feeders-1].write(SERVO_CLOSED_DEG);
    active_feeders--;

    updateActiveFeedersLeds();

    previous_buttonpress_ds = getDecisecondsSinceBoot(); //Easiest way to reset timer..
    previous_hours_left = active_feeders==0 ? 0 : 6;
    updateHourLeds();
  }
}


void setup() {
#ifdef GPIO_DEBUG
  Serial.begin(115200);
#else
  Serial.end();
#endif

  //set pin modes
  for (auto pin : active_feeders_led_pins)
    pinMode(pin, OUTPUT);

  for (auto pin : hour_led_pins)
    pinMode(pin, OUTPUT);

  for (auto pin : servo_control_pins)
    pinMode(pin, OUTPUT);

  pinMode(button_pin, INPUT_PULLDOWN);

  for (auto i=0; i<SERVO_COUNT; i++)
    servos[i].attach(servo_control_pins[i]);

  //Save some power
  btStop(); //Disable bluetooth
#ifndef GPIO_DEBUG
  WiFi.mode(WIFI_OFF); //Disable WiFi
#endif
  setCpuFrequencyMhz(80); //Reduce CPU speed

  updateActiveFeedersLeds();
  updateHourLeds();

#ifdef GPIO_DEBUG
  gpio_viewer.connectToWifi("<ssid>", "<password>");
  gpio_viewer.begin();
#endif
}

void loop() {
  unsigned long now_ds = getDecisecondsSinceBoot();

  bool button_currently_pressed = digitalRead(button_pin);
  if (button_was_pressed && !button_currently_pressed) {
    button_was_pressed = false;
  } else if (!button_was_pressed && button_currently_pressed && (previous_buttonpress_ds+buttonpress_grace_period_ds) < now_ds) {
    button_was_pressed = true;
    previous_buttonpress_ds = now_ds;
    active_feeders++;
    if (active_feeders > SERVO_COUNT) {
      active_feeders = 0;
    }
    updateActiveFeedersLeds();
  }

  if (active_feeders > 0) {
    int hours_left = 6 - (now_ds-previous_buttonpress_ds) / ONE_HOUR_IN_DS;
    if (hours_left != previous_hours_left) {
      previous_hours_left = hours_left;
      if (hours_left == 0) {
        activateFeeder();
      } else {
        updateHourLeds();
      }
    }
  }

  delay(20);
}

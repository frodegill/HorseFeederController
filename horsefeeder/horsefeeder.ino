//#define GPIO_DEBUG
//#define SERIAL_DEBUG

#ifdef GPIO_DEBUG
# include <gpio_viewer.h>
  GPIOViewer gpio_viewer;
#endif

#include <WiFi.h>

/** Start code taken from https://github.com/pablomarquez76/AnalogWrite_ESP32 ******************************************/
#define NUM_CHANNELS (16)
#define SERVO_FREQUENCY (50)
#define SERVO_RESOLUTION (12)

#define SERVO_MIN_PULSE_WIDTH (400)
#define SERVO_MAX_PULSE_WIDTH (2500)
#define SERVO_CLOSED_PULSE_WIDTH (SERVO_MIN_PULSE_WIDTH+100)
#define SERVO_OPEN_PULSE_WIDTH (SERVO_MAX_PULSE_WIDTH-100)

#define SERVO_MIN_TICKS 102
#define SERVO_MAX_TICKS 512
#define SERVO_OVERFLOW_PULSE_WIDTH 4000

typedef struct analog_write_channel {
  int8_t pin;
  uint32_t frequency;
  uint8_t resolution;
} analog_write_channel_t;

analog_write_channel_t _analog_write_channels[NUM_CHANNELS] = {
  { -1, 5000, 8 },
  { -1, 5000, 8 },
  { -1, 5000, 8 },
  { -1, 5000, 8 },
  { -1, 5000, 8 },
  { -1, 5000, 8 },
  { -1, 5000, 8 },
  { -1, 5000, 8 },
  { -1, 5000, 8 },
  { -1, 5000, 8 },
  { -1, 5000, 8 },
  { -1, 5000, 8 },
  { -1, 5000, 8 },
  { -1, 5000, 8 },
  { -1, 5000, 8 },
  { -1, 5000, 8 }
};

uint8_t analogWriteChannel(int8_t pin) {
  uint8_t channel = NUM_CHANNELS;
  // Check if pin already attached to a channel
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    if (_analog_write_channels[i].pin == pin) {
      channel = i;
      break;
    }
  }
  // If not, attach it to a free channel
  if (channel == NUM_CHANNELS) {
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
      if (_analog_write_channels[i].pin == -1) {
        _analog_write_channels[i].pin = pin;
        channel = i;
        ledcSetup(channel, _analog_write_channels[i].frequency, _analog_write_channels[i].resolution);
        ledcAttachPin(pin, channel);
        break;
      }
    }
  }
  return channel;
}

void analogServo(int8_t pin, uint32_t value) {
  // Get channel
  uint8_t channel = analogWriteChannel(pin);
  // Make sure the pin was attached to a channel, if not do nothing
  if (channel < NUM_CHANNELS) {
    // Set frequency and resolution
    if (_analog_write_channels[channel].frequency != SERVO_FREQUENCY || _analog_write_channels[channel].resolution != SERVO_RESOLUTION) {
      _analog_write_channels[channel].frequency = SERVO_FREQUENCY;
      _analog_write_channels[channel].resolution = SERVO_RESOLUTION;
      ledcSetup(channel, _analog_write_channels[channel].frequency, _analog_write_channels[channel].resolution);
    }

    if (value < SERVO_OVERFLOW_PULSE_WIDTH) {
      ledcWrite(channel, map(value, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH, SERVO_MIN_TICKS, SERVO_MAX_TICKS));  // map ms to ticks
    } else {
      ledcWrite(channel, 0);
    }
  }
}
/** End code taken from https://github.com/pablomarquez76/AnalogWrite_ESP32 ******************************************/



unsigned int clock_rollover = 0;
unsigned long previous_time_ms = 0L;

constexpr uint8_t SERVO_COUNT = 4;

constexpr uint8_t active_feeders_led_pins[SERVO_COUNT] = {19,21,23,22};
constexpr uint8_t hour_led_pins[6] = {32,33,25,26,27,14};
constexpr uint8_t servo_control_pins[SERVO_COUNT] = {18,5,17,16};
constexpr uint8_t button_pin = 13;

constexpr uint8_t LED_OFF = 0;
constexpr uint8_t LED_ON = 255;

bool button_was_pressed = false;
unsigned long previous_buttonpress_ds = 0L;
constexpr unsigned long buttonpress_grace_period_ds = 2L; //0.2 sec between each press (to avoid debouncing)
constexpr unsigned long buttonpress_testmode_ds = 20L; //2.0 sec to toggle test mode and release first feeder
constexpr unsigned long ONE_HOUR_IN_DS = 60*60*10L;

uint8_t active_feeders = 0;
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
  for (auto i=0; i<sizeof(active_feeders_led_pins)/sizeof(active_feeders_led_pins[0]); i++) {
    analogWrite(active_feeders_led_pins[i], i<active_feeders ? LED_ON : LED_OFF);
  }
}

void updateHourLeds() {
  for (auto i=0; i<sizeof(hour_led_pins)/sizeof(hour_led_pins[0]); i++) {
    analogWrite(hour_led_pins[i], i<previous_hours_left ? LED_ON : LED_OFF);
  }
}

void activateFeeder() {
  if (active_feeders > 0) {
    analogServo(servo_control_pins[active_feeders-1], SERVO_OPEN_PULSE_WIDTH);
    delay(2000);
    analogServo(servo_control_pins[active_feeders-1], SERVO_CLOSED_PULSE_WIDTH);
    delay(2000);
    analogServo(servo_control_pins[active_feeders-1], SERVO_OVERFLOW_PULSE_WIDTH);  // detach

    active_feeders--;
    updateActiveFeedersLeds();

    previous_buttonpress_ds = getDecisecondsSinceBoot(); //Easiest way to reset timer..
  } else {
    active_feeders = 0;
  }

  previous_hours_left = (active_feeders > 0) ? 6 : 0;
  updateHourLeds();
}


void setup() {
#if defined(GPIO_DEBUG) || defined(SERIAL_DEBUG)
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
    pinMode(servo_control_pins[i], OUTPUT);

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

#ifdef SERIAL_DEBUG
  Serial.println("Starting horsefeeder");
  Serial.flush();
#endif

  //Reset servers
  for (uint8_t feeder=0;feeder<SERVO_COUNT; feeder++) {
    analogServo(servo_control_pins[feeder], SERVO_CLOSED_PULSE_WIDTH);
  }
}

void loop() {
  unsigned long now_ds = getDecisecondsSinceBoot();

  bool button_currently_pressed = digitalRead(button_pin);
  if (button_was_pressed && !button_currently_pressed) {
    button_was_pressed = false;
    if ((previous_buttonpress_ds+buttonpress_testmode_ds) < now_ds) { //Enable test mode if button pressed for long enough
      for (int i=4; i>0; i--) {
        active_feeders = i;
        updateActiveFeedersLeds();
        activateFeeder();
      }
    }
  } else if (!button_was_pressed && button_currently_pressed && (previous_buttonpress_ds+buttonpress_grace_period_ds) < now_ds) {
    button_was_pressed = true;
    previous_buttonpress_ds = now_ds;
    active_feeders++;
    if (active_feeders > SERVO_COUNT) {
      active_feeders = 0;
      previous_hours_left = 0;
      updateHourLeds();
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

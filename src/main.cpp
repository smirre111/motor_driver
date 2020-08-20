
#include <Arduino.h>
#include <EasyButton.h>
#include <queue>

//#include "driver/ledc.h"
#include "C:\Users\Reinhold\.platformio\packages\framework-espidf\components\driver\include\driver\ledc.h"

// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ 5000

#define LEDC_HS_TIMER LEDC_TIMER_0
#define LEDC_HS_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_CHANNEL LEDC_CHANNEL_0
#define LEDC_HS_CH1_CHANNEL LEDC_CHANNEL_1

#define LEDC_TEST_CH_NUM (2)
#define LEDC_TEST_DUTY (4000)
#define LEDC_TEST_FADE_TIME (3000)

// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#ifdef NODEMCU_32S
const int motorEnPin = 4;
const int motorPwmUpPin = 5;
const int motorPwmDownPin = 19;
const int ctrlButtonUpPin = 25;
const int ctrlButtonDownPin = 26;
#else
const int motorEnPin = 2;
const int motorPwmUpPin = 34;
const int motorPwmDownPin = 35;
const int ctrlButtonUpPin = 12;
const int ctrlButtonDownPin = 13;
#endif

const int motorRunTime = 8000000; //Runtime in uS

int buttonUpState = 0;
int buttonDownState = 0;

enum blinds_state_t
{
  OFF,
  BLINDS_UP,
  BLINDS_DOWN,
  BLINDS_CMD_UP,
  BLINDS_CMD_DOWN,
  STOP
};


enum blinds_motcmd_t
{
  MOTCMD_IDLE,
  CMD_UP,
  CMD_DOWN
};

std::queue<blinds_motcmd_t> motorCmdQueue;

blinds_state_t blindsCurState = OFF;
blinds_state_t blindsNextState = OFF;

int brightness = 50; // how bright the LED is
int fadeAmount = 5;  // how many points to fade the LED by

int cur_time = 0;
int last_time = 0;
int buttonUpPressed = 0;
int buttonDownPressed = 0;

int motorUp = 0;
int motorDown = 0;

volatile int timerElapsed = 0;

int totalInterruptCounter;

hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE buttonMux = portMUX_INITIALIZER_UNLOCKED;

int duration = 2000;
EasyButton buttonUp(ctrlButtonUpPin, 35, false, false);
EasyButton buttonDown(ctrlButtonDownPin, 35, false, false);

/*
  * Prepare and set configuration of timers
  * that will be used by LED Controller
  */
ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_HS_MODE,           // timer mode
    .duty_resolution = LEDC_TIMER_13_BIT, //, // resolution of PWM duty
    .timer_num = LEDC_HS_TIMER,           // timer index
    .freq_hz = LEDC_BASE_FREQ,            // frequency of PWM signal
    .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
};

// Prepare individual configuration
// for each channel of LED Controller
// by selecting:
// - controller's channel number
// - output duty cycle, set initially to 0
// - GPIO number where LED is connected to
// - speed mode, either high or low
// - timer servicing selected channel
//   Note: if different channels use one timer,
//         then frequency and bit_num of these channels
//         will be the same
ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {

    {.gpio_num = motorPwmUpPin,
     .speed_mode = LEDC_HS_MODE,
     .channel = ledc_channel_t(LEDC_HS_CH0_CHANNEL),
     .intr_type = ledc_intr_type_t(0),
     .timer_sel = ledc_timer_t(LEDC_HS_TIMER),
     .duty = 0,
     .hpoint = 0},
    {.gpio_num = motorPwmDownPin,
     .speed_mode = LEDC_HS_MODE,
     .channel = ledc_channel_t(LEDC_HS_CH1_CHANNEL),
     .intr_type = ledc_intr_type_t(0),
     .timer_sel = ledc_timer_t(LEDC_HS_TIMER),
     .duty = 0,
     .hpoint = 0}};

void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);
  timerElapsed = 1;
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  Serial.println("Timer elapsed!");
}

// Callback.
void onPressedForDurationButtonUp()
{
  Serial.println("Button up has been pressed for the given duration!");
  portENTER_CRITICAL(&buttonMux);
  buttonUpPressed = 1;
  motorCmdQueue.push(CMD_UP);
  portEXIT_CRITICAL(&buttonMux);
}

// Callback.
void onPressedForDurationButtonDown()
{
  Serial.println("Button down has been pressed for the given duration!");
  portENTER_CRITICAL(&buttonMux);
  buttonDownPressed = 1;
  motorCmdQueue.push(CMD_DOWN);
  portEXIT_CRITICAL(&buttonMux);
}

void motorUpFade()
{
  Serial.println("Call: Motor up ");
  const int ch = 0;
  ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
                          ledc_channel[ch].channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
  ledc_fade_start(ledc_channel[ch].speed_mode,
                  ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
  timerRestart(timer);
  timerAlarmWrite(timer, motorRunTime, false);
  timerAlarmEnable(timer);
}

void motorDownFade()
{
  Serial.println("Call: Motor down ");
  const int ch = 1;
  ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
                          ledc_channel[ch].channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
  ledc_fade_start(ledc_channel[ch].speed_mode,
                  ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
  timerRestart(timer);
  timerAlarmWrite(timer, motorRunTime, false);
  timerAlarmEnable(timer);
}

void motorStop()
{
  Serial.println("Call: Motor stop ");
  for (int ch = 0; ch < LEDC_TEST_CH_NUM; ch++)
  {
    ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, 0);
    ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
  }
}

#if 0
// Arduino like analogWrite
// value has to be between 0 and valueMax
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / valueMax) * min(value, valueMax);

  // write duty to LEDC
  ledcWrite(channel, duty);
}
#endif

void setupPwmChannels()
{

  // Set configuration of timer0 for high speed channels
  ledc_timer_config(&ledc_timer);

  // Set LED Controller with previously prepared configuration
  for (int ch = 0; ch < LEDC_TEST_CH_NUM; ch++)
  {
    ledc_channel_config(&ledc_channel[ch]);
  }

  // Initialize fade service.
  ledc_fade_func_install(0);
}

void setup()
{
  // Setup timer and attach timer to a led pin

  pinMode(motorEnPin, OUTPUT);
  digitalWrite(motorEnPin, HIGH);

  pinMode(ctrlButtonUpPin, INPUT);
  pinMode(ctrlButtonDownPin, INPUT);

  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  timer = timerBegin(3, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  // timerAlarmWrite(timer, 8000000, false);
  // timerAlarmEnable(timer);

  Serial.begin(115200);

  // Initialize the button.
  buttonUp.begin();
  buttonDown.begin();

  // Attach callback.
  buttonUp.onPressedFor(duration, onPressedForDurationButtonUp);
  buttonDown.onPressedFor(duration, onPressedForDurationButtonDown);

  setupPwmChannels();

  Serial.println("End of setup!");
}

void fsmProcess()
{
  blinds_motcmd_t cmd = MOTCMD_IDLE;
  portENTER_CRITICAL(&buttonMux);
  if (!motorCmdQueue.empty()) {
    blinds_motcmd_t cmd = motorCmdQueue.front();
    motorCmdQueue.pop();
  }
  portEXIT_CRITICAL(&buttonMux);

 
  switch (blindsCurState)
  {
  case OFF:
    //if (buttonUpPressed == 1)
    if (cmd == CMD_UP)
    {
      blindsNextState = BLINDS_CMD_UP;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Button Up pressed; BlindsUp");
      }
      motorUpFade();
    }
    //else if (buttonDownPressed == 1)
    else if (cmd == CMD_DOWN)
    {
      blindsNextState = BLINDS_CMD_DOWN;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Button Down pressed; BlindsDown");
      }
      motorDownFade();
    }
    if (blindsCurState != blindsNextState)
    {
      //motorStop();
    }
    break;
  case BLINDS_CMD_UP:
    //if (buttonDownPressed == 1)
    if (cmd == CMD_DOWN)
    {
      blindsNextState = OFF;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Button Down pressed during BlindsUp");
      }
      motorStop();
    }
    else
    {
      if (timerElapsed == 1)
      {
        portENTER_CRITICAL(&timerMux);
        timerElapsed = 0;
        portEXIT_CRITICAL(&timerMux);
        blindsNextState = BLINDS_UP;
        motorStop();
      }
    }
    if (blindsCurState != blindsNextState)
    {
      //motorUpFade();
    }

    break;

  case BLINDS_CMD_DOWN:
    //if (buttonUpPressed == 1)
    if (cmd == CMD_UP)
    {
      blindsNextState = OFF;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Button Up pressed during BlindsDown");
      }
      motorStop();
    }
    else
    {
      if (timerElapsed == 1)
      {
        portENTER_CRITICAL(&timerMux);
        timerElapsed = 0;
        portEXIT_CRITICAL(&timerMux);
        blindsNextState = BLINDS_DOWN;
        motorStop();
      }
    }
    if (blindsCurState != blindsNextState)
    {
      //motorDownFade();
    }
    break;

  case BLINDS_DOWN:
    //if (buttonUpPressed == 1)
    if (cmd == CMD_UP)
    {
      blindsNextState = BLINDS_CMD_UP;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Blinds are down; button Up pressed");
      }
      motorUpFade();
    }
    if (blindsCurState != blindsNextState)
    {
      //motorStop();
    }
    break;

  case BLINDS_UP:
    //if (buttonDownPressed == 1)
    if (cmd == CMD_DOWN)
    {
      blindsNextState = BLINDS_CMD_DOWN;
      if (blindsCurState != blindsNextState)
      {
        Serial.println("Blinds are up; button Down pressed");
      }
      motorDownFade();
    }
    if (blindsCurState != blindsNextState)
    {
      //motorStop();
    }
    break;
  case STOP:
    blindsNextState = OFF;
    if (blindsCurState != blindsNextState)
    {
      motorStop();
    }
    break;
  }

  if (blindsCurState != blindsNextState)
  {

    Serial.print("blindsCurState: ");
    Serial.print(blindsCurState);
    Serial.print(" blindsNextState: ");
    Serial.println(blindsNextState);
  }

  // if (timerElapsed > 0)
  // {
  //   portENTER_CRITICAL(&timerMux);
  //   timerElapsed = 0;
  //   portEXIT_CRITICAL(&timerMux);
  // }

  portENTER_CRITICAL(&buttonMux);
  buttonDownPressed = 0;
  buttonUpPressed = 0;
  portEXIT_CRITICAL(&buttonMux);

  blindsCurState = blindsNextState;
}

void loop()
{
  buttonUp.read();
  buttonDown.read();
  fsmProcess();
}

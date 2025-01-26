#include <Arduino.h>
#include <ArduinoOTA.h>
#include <TelnetStream.h>

/* Custom Libraries */
#include <global_command_manager.h>
#include <wireless_driver.h>
#include <tof_driver.h>
#include <motor_driver.h>
#include <motor_function.h>
#include <encoder_driver.h>
#include <encoder_turn.h>

/* Algorithm */
#include <wall_follower.h>
#include <flood_fill.h>

/* Encoder Variables */
volatile int leftEncoderValue = 0;
volatile int rightEncoderValue = 0;
volatile bool encoderDataReady = false;

/* Wall PID Parameters */
volatile float lastWallError = 0;
volatile float wallIntegral = 0;

/* Encoder PID Parameters */
volatile float lastEncoderError = 0;
volatile float encoderIntegral = 0;

/* Sensor Correction Factor */
volatile int leftFactor = 0;
volatile int rightFactor = 0;

/* Flood Fill */
volatile bool isFastRun = false;

/* WiFi Flag */
volatile bool manualFlag = false;

void stop_command(void)
{
  brake();
  disable_motor();
  lastWallError = 0;
  wallIntegral = 0;
  manualFlag = false;
  commandManager.stop_execution();
  TelnetStream.println("");
}

void start_command(void)
{
  enable_motor();
  while (!commandManager.should_stop())
  {
    wait_for_command();
    flood_fill();
  }
}

void wall_command(void)
{
  enable_motor();
  while (!commandManager.should_stop())
  {
    wait_for_command();
    wall_follower();
  }
}

void debug_command(void)
{
  while (!commandManager.should_stop())
  {
    wait_for_command();
    tof_read(true);
  }
}

void fast_command(void)
{
  isFastRun = true;
}

void manual_command(void)
{
  if (manualFlag)
  {
    std::array<uint16_t, 4> readings = tof_read(false);

    int leftDiagonal = readings[2];
    int rightDiagonal = readings[1];

    if (leftDiagonal < 100 && rightDiagonal < 100)
    {
      delay(100);
      readings = tof_read(false);

      leftDiagonal = readings[2];
      rightDiagonal = readings[1];

      if (leftDiagonal < 100 && rightDiagonal < 100)
      {
        manualFlag = true;
      }
    }
  }

  else
  {
    enable_motor();
    flood_fill();
  }
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  ota_setup("connor", "password"); // Argument: ssid, password
  tof_setup();
  motor_setup();
  encoder_setup();

  digitalWrite(LED_BUILTIN, HIGH); // Turn on onboard LED if successful
  disable_motor();

  /* Commands */
  commandManager.add_command("START", start_command);
  commandManager.add_command("DEBUG", debug_command);
  commandManager.add_command("STOP", stop_command);
  // commandManager.add_command("TURN", encoder_turn_back);
  // commandManager.add_command("LEFT", encoder_turn_left);
  // commandManager.add_command("RIGHT", encoder_turn_right);
  commandManager.add_command("CALI", calibrate_tof_front_threshold);
  commandManager.add_command("FAST", fast_command);
  commandManager.add_command("WALL", wall_command);
}

void loop()
{
  //encoder_debug();
  ArduinoOTA.handle();
  wait_for_command();
  delay(10);
}

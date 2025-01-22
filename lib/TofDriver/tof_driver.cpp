#include <tof_driver.h>
#include <motor_function.h>
#include <motor_driver.h>

VL53L0X tof_sensors[4];

const int XSHUT_PINS[4] = {XSHUT_1, XSHUT_2, XSHUT_4, XSHUT_5};
const uint8_t TOF_SENSOR_ADDRESSES[4] = {tofAddress1, tofAddress2, tofAddress4, tofAddress5};

void tof_setup(void)
{
    Wire.setClock(400000); // Set to 400 kHz
    Wire.begin(SDA_PIN, SCL_PIN);

    // Reset and initialize sensors
    for (int i = 0; i < sensorNum; i++)
    {
        pinMode(XSHUT_PINS[i], OUTPUT);
        digitalWrite(XSHUT_PINS[i], LOW);
        delay(10);
    }

    // I²C Address assignment
    for (int i = 0; i < sensorNum; i++)
    {
        digitalWrite(XSHUT_PINS[i], HIGH);
        delay(10);

        if (!tof_sensors[i].init(true))
        {
            TelnetStream.print("Init Failed: Sensor ");
            TelnetStream.println(i);

            continue;
        }

        tof_sensors[i].setAddress(TOF_SENSOR_ADDRESSES[i]);
        tof_sensors[i].setTimeout(500);
        tof_sensors[i].startContinuous(); // Start continuous back-to-back mode (take readings as fast as possible).
    }
}

void calibrate_tof_front_threshold(void)
{
    int totalLeft = 0;
    int totalRight = 0;

    TelnetStream.println("");
    TelnetStream.println("*** Calibrating ***");

    for (int i = 0; i < 10; i++)
    {
        std::array<uint16_t, 4> readings = tof_read(false);

        int leftDiagonal = readings[2];
        int rightDiagonal = readings[1];

        totalLeft += leftDiagonal;
        totalRight += rightDiagonal;
    }

    int avgLeft = totalLeft / 10;
    int avgRight = totalRight / 10;
    int correctionFactor = abs(avgLeft - avgRight);

    TelnetStream.println("");
    TelnetStream.print("Average Left: ");
    TelnetStream.print(avgLeft);
    TelnetStream.print(" Average Right: ");
    TelnetStream.print(avgRight);
    TelnetStream.print(" Correction Factor: ");
    TelnetStream.print(correctionFactor);
    TelnetStream.println("");

    if (avgLeft > avgRight)
    {
        leftFactor = correctionFactor;
    }
    else
    {
        rightFactor = correctionFactor;
    }

    TelnetStream.println("");
    TelnetStream.print("Left Factor: ");
    TelnetStream.print(leftFactor);
    TelnetStream.print(" Right Factor: ");
    TelnetStream.print(rightFactor);
    TelnetStream.println("");
}

std::array<uint16_t, sensorNum> tof_read(bool debug)
{
    std::array<uint16_t, sensorNum> readings;

    for (int i = 0; i < sensorNum; i++)
    {
        uint16_t distance = tof_sensors[i].readRangeContinuousMillimeters();
        delay(10);

        if (tof_sensors[i].timeoutOccurred() || distance == 65535)
        {
            TelnetStream.println("Time Out Error!");
            brake();
            disable_motor();
        }
        else
        {
            readings[i] = distance;
        }
    }

    readings[1] -= rightFactor;
    readings[2] -= leftFactor;

    if (debug)
    {
        TelnetStream.print(readings[3]);
        TelnetStream.print(",  ");
        TelnetStream.print(readings[2]);
        TelnetStream.print(",  ");
        TelnetStream.print(readings[1]);
        TelnetStream.print(",  ");
        TelnetStream.print(readings[0]);
        TelnetStream.println();
    }

    return readings;
}
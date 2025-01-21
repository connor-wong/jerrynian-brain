#include <encoder_driver.h>

void encoder_setup(void)
{
    pinMode(L_ENCODER_A, INPUT_PULLUP);
    pinMode(L_ENCODER_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(L_ENCODER_A), encoder_left_isr, CHANGE);

    pinMode(R_ENCODER_A, INPUT_PULLUP);
    pinMode(R_ENCODER_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(R_ENCODER_A), encoder_right_isr, CHANGE);
}

void encoder_left_isr(void)
{
    bool stateA = digitalRead(L_ENCODER_A);
    bool stateB = digitalRead(L_ENCODER_B);

    // Update count based on direction
    if (stateA == stateB)
    {
        leftEncoderValue++; // Forward
    }
    else
    {
        leftEncoderValue--; // Backward
    }

    encoderDataReady = true;
}

void encoder_right_isr(void)
{
    bool stateA = digitalRead(R_ENCODER_A);
    bool stateB = digitalRead(R_ENCODER_B);

    // Update count based on direction
    if (stateA == stateB)
    {
        rightEncoderValue--; // Forward
    }
    else
    {
        rightEncoderValue++; // Backward
    }

    encoderDataReady = true;
}

void encoder_debug(void)
{
    if (encoderDataReady)
    {
        TelnetStream.print("Left Encoder: ");
        TelnetStream.print(leftEncoderValue);
        TelnetStream.print(" Right Encoder: ");
        TelnetStream.println(rightEncoderValue);
        encoderDataReady = false;
    }
}
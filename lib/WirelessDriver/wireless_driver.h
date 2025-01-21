#ifndef WIRELESS_DRIVER_H
#define WIRELESS_DRIVER_H

#include <Arduino.h>
#include <WiFi.h>
#include <TelnetStream.h>
#include <ArduinoOTA.h>

#define WIFI_SCAN_DISABLED // Disable Wi-Fi scan if not needed

void ota_setup(const char *ssid, const char *password);
String received_command(bool echo);
String trim(String str);
void wait_for_command(void);

#endif
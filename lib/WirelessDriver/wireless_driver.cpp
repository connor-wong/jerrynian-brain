#include <wireless_driver.h>
#include <global_command_manager.h>

/* WiFi Config */
IPAddress local_IP(192, 168, 137, 100); // Fixed static IP
IPAddress gateway(192, 168, 137, 1);    // Hotspot gateway IP
IPAddress subnet(255, 255, 255, 0);     // Subnet mask
IPAddress primaryDNS(8, 8, 8, 8);       // Primary DNS (Google)
IPAddress secondaryDNS(8, 8, 4, 4);     // Secondary DNS (Google)

void ota_setup(const char *ssid, const char *password)
{
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);

    // Set static IP
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS))
    {
        delay(500);
        ESP.restart();
    }

    WiFi.begin(ssid, password); // Connect to local network

    while (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
        for (int i = 0; i < 3; i++)
        {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(250);
            digitalWrite(LED_BUILTIN, LOW);
            delay(250);
        }

        ESP.restart();
    }

    ArduinoOTA.begin();
    TelnetStream.begin();

    delay(100);
}

String received_command(bool echo)
{
    String receivedMessage = ""; // Static to persist across calls

    // Check if there's data available in TelnetStream
    while (TelnetStream.available())
    {
        int c = TelnetStream.read(); // Read one character

        if (c == '\n') // End of message (newline character)
        {
            receivedMessage = trim(receivedMessage); // Trim any extra characters
            receivedMessage.toUpperCase();           // Convert to uppercase

            if (echo)
            {
                TelnetStream.println("");
                TelnetStream.println("Received Command: " + receivedMessage);
            }

            return receivedMessage; // Return the trimmed command
        }

        if (c != -1)
        {
            receivedMessage += (char)c; // Append character to the string
        }
    }

    return ""; // Return an empty string if no complete command is received
}

String trim(String str)
{
    str.trim();            // Removes leading and trailing whitespaces
    str.replace("\r", ""); // Remove any carriage return characters
    return str;
}

void wait_for_command(void)
{
    String command = received_command(true);

    if (command.length() > 0)
    {
        commandManager.execute_command(command);
    }
}
#include <memory_driver.h>

void memory_setup(void)
{
    EEPROM.begin(EEPROM_SIZE);
}

void memory_store(int address, int data)
{
    EEPROM.write(address, data);
    EEPROM.commit();
}

int memory_read(int address)
{
    int data = EEPROM.read(address);

    return data;
}

void memory_reset(int address)
{
    EEPROM.write(address, 0);
    EEPROM.commit();
}
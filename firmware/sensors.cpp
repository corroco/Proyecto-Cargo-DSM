#include "sensors.h"
float tara = 0.0;
long load_cell_reading = 0.0;
float load_cell_value = 0.0;
// objeto celda
HX711 celda;
void sensors_setup(void)
{
    celda.begin(DOUT_PIN, SCK_PIN);
    if (celda.is_ready())
    {
        celda.set_scale();
        // Serial.println("COLOCAR MASA CONOCIDA");
        // delay(10000);
        // long reading = celda.read_average(10);
        // Serial.print("VALOR A UTILIZAR PARA FACTOR");
        // Serial.println(reading);
        // delay(5000);
    }
    else
    {
        Serial.println(0);
    }
    delay(1000);
}

float read_load_cell(void)
{
    load_cell_reading = 16.00 * (celda.read_average(5) + tara) / 147768.00;
    // convert long to float load cell reading
    load_cell_value = (float)load_cell_reading;

    return load_cell_value;
}

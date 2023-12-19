#include "motor.h"
#include "sensors.h"

const int switchKey = 40;

// Parámetros del PID (ajustar según sea necesario)
float kp = 2;  // Ganancia proporcional
float ki = 0;  // Ganancia integral
float kd = 0; // Ganancia derivativa

// Estado deseado de la celda de carga
int targetLoadCell = -186;

// Estado actual de la celda de carga
// int baseLoadCellReading = 180;

// Duración del loop principal
int loopTime = 30;

// Variables para el controlador PID
float error = 0;
float integral = 0;
float derivative = 0;
float lastError = 0;

bool motorsActive = false;

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("----");
  Serial.println("READY BIKE TRAILER");
  Serial.println("----");
  delay(200);
  sensors_setup();
  pinMode(switchKey, OUTPUT);
  digitalWrite(switchKey, HIGH);
  // baseLoadCellReading = read_load_cell();
}

void loop()
{
  static unsigned long lastLoopTime = 0;
  unsigned long currentMillis = millis();

  // Realiza el loop principal cada loopTime milisegundos
  if (currentMillis - lastLoopTime >= loopTime)
  {
    int loadCellReading = read_load_cell();

    // Calcular el error
    error = targetLoadCell - loadCellReading;

    // Calcular término integral
    integral += error;

    // Calcular término derivativo
    derivative = error - lastError;

    // Calcular salida del PID
    float output = kp * error + ki * integral + kd * derivative + 38;

    // Limitar la salida entre valores máximos y mínimos de PWM
    output = constrain(output, 38, 120);

    // Verificar si los motores deben activarse
    if (!motorsActive || loadCellReading < targetLoadCell)
    {
      motor_cmd_vel(output, output);
      motorsActive = true;
    }
    else
    {
      motor_stop();
      motorsActive = false;
    }

    // Almacenar el error actual para el siguiente ciclo
    lastError = error;

    // Imprimir la lectura de la celda de carga y la salida del PID
    Serial.print("Load Cell Reading: ");
    Serial.print(loadCellReading);
    Serial.print("  PID Output: ");
    Serial.println(output);

    // Actualizar el tiempo del último bucle
    lastLoopTime = currentMillis;
  } 
}

void serialEvent()
{
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    Serial.print("Received: ");
    Serial.println(inChar);

    if (inChar == 'O')
    {
      char nextChar = (char)Serial.read(); // Lee el siguiente carácter
      Serial.print("Next Char: ");
      Serial.println(nextChar);

      if (nextChar == 'N' && Serial.read() == '\n')
      {
        Serial.println("Turning ON");
        digitalWrite(switchKey, LOW);
      }
      else if (nextChar == 'F')
      {
        char thirdChar = (char)Serial.read(); // Lee el tercer carácter
        Serial.print("Third Char: ");
        Serial.println(thirdChar);

        if (thirdChar == 'F' && Serial.read() == '\n')
        {
          Serial.println("Turning OFF");
          digitalWrite(switchKey, HIGH);
        }
      }
    }
  }
}

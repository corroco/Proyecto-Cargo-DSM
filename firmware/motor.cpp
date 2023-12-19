#include "motor.h"

int vx_pwm = 0;
int vy_pwm = 0;
// float vx_abs = 0.0;
// float vy_abs = 0.0;
// float max_speed = MAX_SPEED;
// float min_speed = MIN_SPEED;
bool dir_a = HIGH;
bool dir_b = HIGH;

void motors_setup(void)
{
  pinMode(PWM_A, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  // Set the direction of the motors to forward
  digitalWrite(DIR_A, HIGH);
  digitalWrite(DIR_B, HIGH);
}

// cmd_vel que toma 2 valores PWM
void motor_cmd_vel(int pwm_a, int pwm_b)
{
  Serial.println("CMDVEL");
  Serial.print("pwm_a: ");
  Serial.println(pwm_a);
  Serial.print("pwm_b: ");
  Serial.println(pwm_b);

  // Toma la dirección de los motores del signo de pwm_a y pwm_b
  dir_a = (pwm_a >= 0) ? HIGH : LOW;
  dir_b = (pwm_b >= 0) ? HIGH : LOW;

  // Toma el valor absoluto de pwm_a y pwm_b
  vx_pwm = abs(pwm_a);
  vy_pwm = abs(pwm_b);

  // Verifica si la velocidad está fuera de rango
  if (vx_pwm > MAX_SPEED_PWM)
  {
    vx_pwm = MAX_SPEED_PWM;
  }
  if (vy_pwm > MAX_SPEED_PWM)
  {
    vy_pwm = MAX_SPEED_PWM;
  }

  // Mueve los motores
  digitalWrite(DIR_A, dir_a);
  digitalWrite(DIR_B, dir_b);

  // Establece la velocidad de los motores usando una rampa
  unsigned long startTime = millis();
  unsigned long rampDuration = 25;  // Ajusta la duración de la rampa según sea necesario

  while (millis() - startTime < rampDuration)
  {
    float fraction = float(millis() - startTime) / rampDuration; // Normaliza el tiempo de la rampa

    // Calcular el valor de PWM proporcional a la fracción
    int currentPWM_A = int(fraction * vx_pwm);
    int currentPWM_B = int(fraction * vy_pwm);

    // Establecer la velocidad de los motores
    analogWrite(PWM_A, currentPWM_A);
    analogWrite(PWM_B, currentPWM_B);
  }

  // Establecer la velocidad final después de la rampa
  analogWrite(PWM_A, vx_pwm);
  analogWrite(PWM_B, vy_pwm);

  Serial.print("vx_pwm: ");
  Serial.println(vx_pwm);
  Serial.print("vy_pwm: ");
  Serial.println(vy_pwm);
}


void motor_stop(void)
{
  // Set the speed of the motors to 0
  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);
}

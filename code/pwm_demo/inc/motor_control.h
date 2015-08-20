#ifndef MOTOR_CONTROL_H_INCLUDED
#define MOTOR_CONTROL_H_INCLUDED

#define MOTOR1      1
#define MOTOR2      2

#define FORWARD     1
#define BACKWARD    2
#define BRAKE       0

void configMotors(void);
void setMotorDirection(uint8_t motor, uint8_t direction);
void setMotorSpeed(uint8_t motor, uint8_t speed);

#endif /* MOTOR_CONTROL_H_INCLUDED */

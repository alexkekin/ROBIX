/*
**
**                           Main.c
**
**
**********************************************************************/
/*
   Last committed:     $Revision: 00 $
   Last changed by:    $Author: $
   Last changed date:  $Date:  $
   ID:                 $Id:  $

**********************************************************************/
#include "stm32f10x_conf.h"
#include "motor_control.h"





int main(void) {

    SystemInit();
    configMotors();

    setMotorDirection(MOTOR1, BRAKE);
    setMotorDirection(MOTOR2, BRAKE);


    while(1) {

    }
}

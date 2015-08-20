#include "stm32f10x.h"
#include "motor_control.h"

void configMotors(void) {

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_Write(GPIOC, 0x0);

    // ������������ �������
    TIM_TimeBaseInitTypeDef TIM_BaseConfig;
    // ������������ ������ �������
    TIM_OCInitTypeDef TIM_OCConfig;

    // ��������� ������ �� �������� ������� � 2400 kHz
    TIM_BaseConfig.TIM_Prescaler = (uint16_t) (SystemCoreClock / 2400000) - 1;
    // ������ - 256 ������ =>  9.375kHz
    TIM_BaseConfig.TIM_Period = 255;
    TIM_BaseConfig.TIM_ClockDivision = 0;
    // ������ �� ���� �� TIM_Period
    TIM_BaseConfig.TIM_CounterMode = TIM_CounterMode_Up;
    // �������������� ������ �3 (��� ������ ��� ��� �� ����� C)
    TIM_TimeBaseInit(TIM3, &TIM_BaseConfig);
    // ������������� ����� �������, ����� - PWM1
    TIM_OCConfig.TIM_OCMode = TIM_OCMode_PWM1;
    // ���������� - ����� �������
    TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
    // ����� ������� 127 ������ => 127/255 = 50%
    TIM_OCConfig.TIM_Pulse = 0;
    // ���������� => ����� - ��� ������� (+3.3V)
    TIM_OCConfig.TIM_OCPolarity = TIM_OCPolarity_High;
    // �������������� 3 ����� ������� �4 (PB8)
    TIM_OC1Init(TIM3, &TIM_OCConfig);
    TIM_OC2Init(TIM3, &TIM_OCConfig);
    TIM_OC3Init(TIM3, &TIM_OCConfig);
    TIM_OC4Init(TIM3, &TIM_OCConfig);
    // �������� ������
    TIM_Cmd(TIM3, ENABLE);
}

void setMotorDirection(uint8_t motor, uint8_t direction) {

    GPIO_InitTypeDef gpio;

    setMotorSpeed(motor, 0);
    switch(motor) {
        case MOTOR1:
            GPIO_ResetBits(GPIOC, GPIO_Pin_6);
            GPIO_ResetBits(GPIOC, GPIO_Pin_7);

            switch(direction) {
                case BRAKE:
                    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
                    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
                    gpio.GPIO_Speed = GPIO_Speed_2MHz;
                    GPIO_Init(GPIOC, &gpio);
                    break;

                case FORWARD:
                    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
                    gpio.GPIO_Pin = GPIO_Pin_7;
                    gpio.GPIO_Speed = GPIO_Speed_2MHz;
                    GPIO_Init(GPIOC, &gpio);

                    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
                    gpio.GPIO_Pin = GPIO_Pin_6;
                    gpio.GPIO_Speed = GPIO_Speed_2MHz;
                    GPIO_Init(GPIOC, &gpio);
                    break;

                case BACKWARD:
                    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
                    gpio.GPIO_Pin = GPIO_Pin_6;
                    gpio.GPIO_Speed = GPIO_Speed_2MHz;
                    GPIO_Init(GPIOC, &gpio);
                    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
                    gpio.GPIO_Pin = GPIO_Pin_7;
                    gpio.GPIO_Speed = GPIO_Speed_2MHz;
                    GPIO_Init(GPIOC, &gpio);
                    break;
            }
            break;

        case MOTOR2:
            GPIO_ResetBits(GPIOC, GPIO_Pin_8);
            GPIO_ResetBits(GPIOC, GPIO_Pin_9);

            switch(direction) {
                case BRAKE:
                    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
                    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
                    gpio.GPIO_Speed = GPIO_Speed_2MHz;
                    GPIO_Init(GPIOC, &gpio);
                    break;

                case FORWARD:
                    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
                    gpio.GPIO_Pin = GPIO_Pin_9;
                    gpio.GPIO_Speed = GPIO_Speed_2MHz;
                    GPIO_Init(GPIOC, &gpio);
                    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
                    gpio.GPIO_Pin = GPIO_Pin_8;
                    gpio.GPIO_Speed = GPIO_Speed_2MHz;
                    GPIO_Init(GPIOC, &gpio);
                    break;

                case BACKWARD:
                    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
                    gpio.GPIO_Pin = GPIO_Pin_8;
                    gpio.GPIO_Speed = GPIO_Speed_2MHz;
                    GPIO_Init(GPIOC, &gpio);
                    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
                    gpio.GPIO_Pin = GPIO_Pin_9;
                    gpio.GPIO_Speed = GPIO_Speed_2MHz;
                    GPIO_Init(GPIOC, &gpio);
                    break;
            }
            break;
    }

}

void setMotorSpeed(uint8_t motor, uint8_t speed) {

    switch(motor) {
        case MOTOR1:
            TIM_SetCompare1(TIM3, speed);
            TIM_SetCompare2(TIM3, speed);
            break;
        case MOTOR2:
            TIM_SetCompare3(TIM3, speed);
            TIM_SetCompare4(TIM3, speed);
            break;
    }
}

//
// Created by Ray on 2023/11/25.
//

#ifndef INC_2024RC_B_R1_STEPPER_H
#define INC_2024RC_B_R1_STEPPER_H

#include "main.h"

#define CW 0    //˳ʱ��
#define CCW 1   //��ʱ��

//һЩ���ݵļ���
#define SPR         3200                //���������Ȧ����
#define MICRO_STEP  16                  //�������������ϸ����

#define R2L         12                  //�������תһȦ��˿���ƶ�12mm
#define LPD         R2L / 360           //�������ÿתһ�ȣ�˿���ƶ�����
#define LPS         R2L / SPR           //ÿ���� ˿������·��
#define SSpd        60                  //˿���ٶ� mmÿ��
#define MSpd        SSpd / R2L          //����ٶ� Ȧÿ��
#define FPWM        (MSpd * SPR)        //�����ٶ���Ҫ��pwmƵ�� 16000  ����5250 2625

extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;

typedef struct Screws
{
    uint8_t  run_state;    // �����ת״̬ 0 ��ʾ���� 1 ��ʾ��������
    uint8_t  dir;          // �����ת���� CW CCW
    int32_t  target_step;   // ����������      ���ݵ��ϸ��������õ�
    int32_t  step_count;    // �Ʋ� �жϼ���ֵ

    int dir_cal;            // ���������� ˳ʱ����1 ��ʱ����-1
    float angle;            // �Ƕ�
    float pos;              //��ǰ˿��λ�ã�0Ϊ��ͣ�100Ϊ���
}Screws;

#define Step1_ENABLE    HAL_GPIO_WritePin(step1ena_GPIO_Port, step1ena_Pin, GPIO_PIN_RESET);
#define Step1_DISABLE   HAL_GPIO_WritePin(step1ena_GPIO_Port, step1ena_Pin, GPIO_PIN_SET);

#define Step2_ENABLE    HAL_GPIO_WritePin(step2ena_GPIO_Port, step2ena_Pin, GPIO_PIN_RESET);
#define Step2_DISABLE   HAL_GPIO_WritePin(step2ena_GPIO_Port, step2ena_Pin, GPIO_PIN_SET);

#define Step1_CW    HAL_GPIO_WritePin(step1dir_GPIO_Port, step1dir_Pin, GPIO_PIN_RESET);
#define Step1_CCW   HAL_GPIO_WritePin(step1dir_GPIO_Port, step1dir_Pin, GPIO_PIN_SET);

#define Step2_CW    HAL_GPIO_WritePin(step2dir_GPIO_Port, step2dir_Pin, GPIO_PIN_RESET);
#define Step2_CCW   HAL_GPIO_WritePin(step2dir_GPIO_Port, step2dir_Pin, GPIO_PIN_SET);

extern Screws LIFT;        //��ʱ��10
extern Screws RIGHT;       //��ʱ��11

void screw_init(void);

void screws_move(Screws* screws, float targe_distance);

void screws_pos(Screws* screws, float targe_pos);

#endif //INC_2024RC_B_R1_STEPPER_H

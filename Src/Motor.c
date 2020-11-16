/*
 * Motor.c
 *
 *  Created on: 2020. 8. 25.
 *      Author: ghhan
 */
#include "Motor.h"
#include "tim.h"

uint32_t Now_period;

void Var_Init()
{
	TX_buf_Tail=0;
	TX_buf_Haed=0;
	TX_str_leng=0;
	Motor_flag=0;
	Now_period=period_val;
}

int Change_period()
{
	int Th, Hun, Ten, One = 0;

	for(int i=0; i<=3;i++)
		{
			for(int j=48; j<=57;j++)
				{
					if(TX_str_leng==4)
						{
							if(TX_str[0]==j){Th=(j-48)*1000;}
							if(TX_str[1]==j){Hun=(j-48)*100;}
							if(TX_str[2]==j){Ten=(j-48)*10;}
							if(TX_str[3]==j){One=(j-48)*1;}
						}
					else if(TX_str_leng==3)
						{
							if(TX_str[0]==j){Hun=(j-48)*100;}
							if(TX_str[1]==j){Ten=(j-48)*10;}
							if(TX_str[2]==j){One=(j-48)*1;}
						}
					else if(TX_str_leng==2)
						{
							if(TX_str[0]==j){Ten=(j-48)*10;}
							if(TX_str[1]==j){One=(j-48)*1;}
						}
					else if(TX_str_leng==1)
						{
							if(TX_str[0]==j){One=(j-48)*1;}
						}
				}
		}

	TX_str_leng=0;

	return Th + Hun + Ten + One;
}

void Motor_Speed_Value(uint8_t num)
{
	HAL_TIM_OC_Stop(&htim12, TIM_CHANNEL_1);

	switch(num)
	{
		case select_val:
			Now_period = Change_period();
			htim12.Init.Period = Change_period();
			break;
		case speed_up:
			Now_period = (Now_period + 100);
			htim12.Init.Period = Now_period;
			break;
		case speed_down:
			Now_period = (Now_period - 100);
			htim12.Init.Period = Now_period;
			break;
		default:
			break;
	}

	if (HAL_TIM_OC_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }

	HAL_TIM_OC_Start(&htim12, TIM_CHANNEL_1);
}


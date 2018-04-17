#include "control.h"	

u8 filter_method = 2;  //ѡ�񿨶����˲�

int Motor_Close = 0;

int Encoder_Left = 0, Encoder_Right = 0;
float Angle_Balance, Gyro_Balance, Gyro_Turn; //ƽ����� ƽ�������� ת��������
float Acceleration_Z;                         //Z����ٶȼ�
int Balance_Pwm = 0, Velocity_Pwm = 0, Turn_Pwm = 0;
int Moto1, Moto2;                             //���PWM���� Ӧ��Motor�� ��Moto�¾�
float Balance_Kp = 300, Balance_Kd = 1;
float Velocity_Kp = 80, Velocity_Ki = 0.4;
/**************************************************************************
�������ܣ����еĿ��ƴ���
		 5ms��ʱ�ж���MPU6050��INT���Ŵ���
		 �ϸ�֤���������ݴ����ʱ��ͬ��
**************************************************************************/
int EXTI15_10_IRQHandler(void)
{
	if (PAin(12) == 0)
	{
		EXTI_ClearITPendingBit(EXTI_Line12);  	 //���LINE12�ϵ��жϱ�־λ   

		if ((USART_RX_STA & 0x8000) == 0x8000)
		{
			if (USART_RX_BUF[0] == 'W' && USART_RX_BUF[1] == ':') //У���ͷ
			{
				sscanf((const char *)(USART_RX_BUF + 2), "%d,%f,%f,%f,%f", &Motor_Close, &Balance_Kp, &Balance_Kd, &Velocity_Kp, &Velocity_Ki);
				printf("Write:%d,%f,%f,%f,%f\r\n", Motor_Close, Balance_Kp, Balance_Kd, Velocity_Kp, Velocity_Ki);
			}
			if (USART_RX_BUF[0] == 'R' && USART_RX_BUF[1] == ':')
			{
				printf("Read:%f,%f,%f,%f\r\n", Angle_Balance, Gyro_Balance, Gyro_Turn, Acceleration_Z);
			}
			if (USART_RX_BUF[0] == 'D' && USART_RX_BUF[1] == ':')
			{
				printf("Data:%d,%f\r\n", Read_Temperature(), Get_battery_volt());
			}
			memset(USART_RX_BUF, 0, USART_REV_LEN);
			USART_RX_STA = 0;

		}

		Encoder_Left = -Read_Encoder(2);                //===��ȡ��������ֵ
		Encoder_Right = Read_Encoder(4);                //===��ȡ��������ֵ
		Get_Angle(filter_method);                       //===������̬
		SafeCheck();
		Balance_Pwm = balance(Angle_Balance, Gyro_Balance);        //===ƽ��PID����	
		Velocity_Pwm = velocity(Encoder_Left, Encoder_Right);      //===�ٶȻ�PID����
		Moto1 = Balance_Pwm - Velocity_Pwm + Turn_Pwm;             //===�������ֵ������PWM
		Moto2 = Balance_Pwm - Velocity_Pwm - Turn_Pwm;             //===�������ֵ������PWM
		setResultPwm(Moto1, Moto2);
	}
	return 0;
}

int EXTI9_5_IRQHandler(void)
{
	delay_ms(10);	//����
	if (PAin(5) == 0)
	{
		if (Motor_Close)
		{
			Motor_Close = 0;
			printf("Turn ON car.\r\n");
		}
		else
		{
			Motor_Close = 1;
			setResultPwm(0, 0);
			printf("Turn OFF car.\r\n");
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line5);
	return 0;
}

/**************************************************************************
�������ܣ���ȡ�Ƕ�
��ڲ�������ȡ�Ƕȵ��㷨 1��DMP  2�������� 3�������˲�
����  ֵ����
**************************************************************************/
void Get_Angle(u8 way)
{
	float Accel_Y, Accel_X, Accel_Z, Gyro_Y, Gyro_Z;
	Gyro_Y = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_L);    //��ȡY��������
	Gyro_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_L);    //��ȡZ��������
	Accel_X = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_L); //��ȡX����ٶȼ�
	Accel_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_L); //��ȡZ����ٶȼ�
	if (Gyro_Y > 32768)  Gyro_Y -= 65536;                    //��������ת��  
	if (Gyro_Z > 32768)  Gyro_Z -= 65536;                    //��������ת��
	if (Accel_X > 32768) Accel_X -= 65536;                   //��������ת��
	if (Accel_Z > 32768) Accel_Z -= 65536;                   //��������ת��
	Gyro_Balance = -Gyro_Y;                                  //����ƽ����ٶ�
	Accel_Y = atan2(Accel_X, Accel_Z) * 180 / PI;            //�������	
	Gyro_Y = Gyro_Y / 16.4;                                  //����������ת��	
	Kalman_Filter(Accel_Y, -Gyro_Y);						 //�������˲�	
	Angle_Balance = angle;                                   //����ƽ�����
	Gyro_Turn = Gyro_Z;                                      //����ת����ٶ�
	Acceleration_Z = Accel_Z;                                //===����Z����ٶȼ�	
}

int balance(float Angle, float Gyro)
{
	float Bias;
	int balance;
	Bias = Angle - MIDDLE;       //===���ƽ��ĽǶ���ֵ �ͻ�е���
	balance = Balance_Kp*Bias + Gyro*Balance_Kd;   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	return balance;
}

int velocity(int encoder_left, int encoder_right)
{
	static float Velocity, Encoder_Least, Encoder, Movement;
	static float Encoder_Integral, Target_Velocity;
	Target_Velocity = 45;
	Movement = Target_Velocity / 2;
	Encoder_Least = (Encoder_Left + Encoder_Right) - 0;                 //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
	Encoder *= 0.8;		                                                //===һ�׵�ͨ�˲���       
	Encoder += Encoder_Least*0.2;	                                    //===һ�׵�ͨ�˲���    
	Encoder_Integral += Encoder;                                        //===���ֳ�λ�� ����ʱ�䣺10ms
	Encoder_Integral = Encoder_Integral - Movement;                     //===����ң�������ݣ�����ǰ������
	if (Encoder_Integral > 10000)  	Encoder_Integral = 10000;           //===�����޷�
	if (Encoder_Integral < -10000)	Encoder_Integral = -10000;          //===�����޷�	
	Velocity = Encoder*Velocity_Kp + Encoder_Integral*Velocity_Ki;      //===�ٶȿ���	
	return Velocity;
}

void setResultPwm(int moto1, int moto2)
{
	if (Motor_Close)
	{
		moto1 = 0;
		moto2 = 0;
		AIN1 = 0;
		AIN2 = 0;
		BIN1 = 0;
		BIN2 = 0;
		PAout(4) = 1;
	}
	else
	{
		if (moto1 < -MAX_SPEED) moto1 = -MAX_SPEED;
		if (moto1 > MAX_SPEED)  moto1 = MAX_SPEED;
		if (moto2 < -MAX_SPEED) moto2 = -MAX_SPEED;
		if (moto2 > MAX_SPEED)  moto2 = MAX_SPEED;
		PAout(4) = 0;
	}
	Set_Pwm(moto1, moto2);
}

void SafeCheck(void)
{
	if (Angle_Balance > 8.0 && Motor_Close == 0)
	{
		Motor_Close = 1;
		PAout(4) = 1;
		setResultPwm(0, 0);
		printf("Turn OFF car.\r\n");
	}
}

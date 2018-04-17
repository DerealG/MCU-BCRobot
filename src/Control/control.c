#include "control.h"	

u8 MPU6050_DMP_DATA = 0; // 1ΪDMP��0Ϊԭʼ����
u8 filter_method = 2;  //1.�������˲� 2.һ�׻����˲�

int Motor_Close = 0;

int Temperature = 0;
float battery_volt = 0;
int Encoder_Left = 0, Encoder_Right = 0;
float Angle_Balance, Gyro_Balance, Gyro_Turn; //ƽ����� ƽ�������� ת��������
float Acceleration_Z;                         //Z����ٶȼ�
int Balance_Pwm = 0, Velocity_Pwm = 0, Turn_Pwm = 0;
int Moto1, Moto2;                             //���PWM����

float middle = -3.5;
float Balance_Kp = 250, Balance_Kd = 1;
float Velocity_Kp = 3, Velocity_Ki = 0.02;
float Turn_Kp = 0, Turn_Kd = 0;

//2018-4-17 19:20 {-3.5,400,1,2,0}
//2018-4-17 20:20 {-3.5,250,1,3,0.02}
/**************************************************************************
�������ܣ����еĿ��ƴ���
		 5ms��ʱ�ж���MPU6050��INT���Ŵ���
		 �ϸ�֤���������ݴ����ʱ��ͬ��
**************************************************************************/
int EXTI15_10_IRQHandler(void)
{
	if (MPU6050_INT == 0)
	{
		EXTI_ClearITPendingBit(EXTI_Line12);  	 //���LINE12�ϵ��жϱ�־λ   

		Temperature = Read_Temperature();      					 //��ȡMPU6050�����¶ȴ���������
		battery_volt = Get_battery_volt();
		Encoder_Left = Read_Encoder(2);                //===��ȡ��������ֵ
		Encoder_Right = Read_Encoder(4);               //===��ȡ��������ֵ
		Get_Angle(MPU6050_DMP_DATA);                 //===������̬
		SafeCheck();
		Balance_Pwm = balance(Angle_Balance, Gyro_Balance);        //===ƽ��PID����
		Velocity_Pwm = velocity(Encoder_Left, Encoder_Right);      //===�ٶȻ�PID����
		//Turn_Pwm = turn(Encoder_Left, Encoder_Right, Gyro_Turn);   //===ת��PID����
		Moto1 = Balance_Pwm - Velocity_Pwm + Turn_Pwm;             //===�������ֵ������PWM
		Moto2 = Balance_Pwm - Velocity_Pwm - Turn_Pwm;             //===�������ֵ������PWM
		setResultPwm(Moto1, Moto2);

		//Data
		if ((USART_RX_STA & 0x8000) == 0x8000)
		{
			if (USART_RX_BUF[0] == 'W' && USART_RX_BUF[1] == ':') //У���ͷ
			{
				sscanf((const char *)(USART_RX_BUF + 2), "%d,%f,%f,%f,%f,%f", &Motor_Close, &middle, &Balance_Kp, &Balance_Kd, &Velocity_Kp, &Velocity_Ki);
				printf("Write:%d,%f,%f,%f,%f,%f\r\n", Motor_Close, middle, Balance_Kp, Balance_Kd, Velocity_Kp, Velocity_Ki);
			}
			if (USART_RX_BUF[0] == 'R' && USART_RX_BUF[1] == ':')
			{
				printf("Read:%f,%f,%f,%f,%f\r\n", middle, Angle_Balance, Gyro_Balance, Gyro_Turn, Acceleration_Z);
			}
			if (USART_RX_BUF[0] == 'D' && USART_RX_BUF[1] == ':')
			{
				printf("Data:%d,%f\r\n", Temperature, battery_volt);
			}
			memset(USART_RX_BUF, 0, USART_REV_LEN);
			USART_RX_STA = 0;

		}

		//DataSend();
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
void Get_Angle(u8 dataSource)
{
	float Accel_Y, Accel_Angle, Accel_Z, Gyro_X, Gyro_Z;
	if (dataSource == 1)                           //===DMP�Ķ�ȡ�����ݲɼ��ж϶�ȡ���ϸ���ѭʱ��Ҫ��
	{
		Read_DMP();                      //===��ȡ���ٶȡ����ٶȡ����
		Angle_Balance = -Roll;             //===����ƽ�����
		Gyro_Balance = -gyro[0];            //===����ƽ����ٶ�
		Gyro_Turn = gyro[2];               //===����ת����ٶ�
		Acceleration_Z = accel[2];         //===����Z����ٶȼ�
	}
	else
	{
		Gyro_X = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_XOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_XOUT_L);    //��ȡY��������
		Gyro_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_L);    //��ȡZ��������
		Accel_Y = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_L); //��ȡX����ٶȼ�
		Accel_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_L); //��ȡZ����ٶȼ�
		if (Gyro_X > 32768)  Gyro_X -= 65536;                       //��������ת��  Ҳ��ͨ��shortǿ������ת��
		if (Gyro_Z > 32768)  Gyro_Z -= 65536;                       //��������ת��
		if (Accel_Y > 32768) Accel_Y -= 65536;                      //��������ת��
		if (Accel_Z > 32768) Accel_Z -= 65536;                      //��������ת��
		Gyro_Balance = Gyro_X;                                   //����ƽ����ٶ�
		Accel_Angle = atan2(Accel_Y, Accel_Z) * 180 / PI;        //�������
		Gyro_X = Gyro_X / 16.4;                                  //����������ת��
		if (filter_method == 1)		  	Kalman_Filter(Accel_Angle, Gyro_X);//�������˲�
		else if (filter_method == 2)   First_Filter(Accel_Angle, Gyro_X);//�����˲�
		Angle_Balance = angle;                                   //����ƽ�����
		Gyro_Turn = Gyro_Z;                                      //����ת����ٶ�
		Acceleration_Z = Accel_Z;                                //===����Z����ٶȼ�
	}
}

/**************************************************************************
�������ܣ�ֱ��PD����
��ڲ������Ƕȡ����ٶ�
����  ֵ��ֱ������PWM
**************************************************************************/
int balance(float Angle, float Gyro)
{
	float Bias;
	int balance;
	Bias = Angle - middle;       //===���ƽ��ĽǶ���ֵ �ͻ�е���
	balance = Balance_Kp*Bias + Gyro*Balance_Kd;   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	return balance;
}

/**************************************************************************
�������ܣ��ٶ�PI���� �޸�ǰ�������ٶ�
��ڲ��������ֱ����������ֱ�����
����  ֵ���ٶȿ���PWM
**************************************************************************/
int velocity(int encoder_left, int encoder_right)
{
	static float Velocity = 0, Encoder_Least = 0, Encoder = 0;
	static float Encoder_Integral = 0;
	Encoder_Least = (Encoder_Left + Encoder_Right) - 0;                 //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
	Encoder *= 0.8;		                                                  //===һ�׵�ͨ�˲���
	Encoder += Encoder_Least*0.2;	                                      //===һ�׵�ͨ�˲���
	Encoder_Integral += Encoder;                                        //===���ֳ�λ�� ����ʱ�䣺10ms
	Encoder_Integral = Encoder_Integral;                     //===����ң�������ݣ�����ǰ������
	if (Encoder_Integral > 10000)  	Encoder_Integral = 10000;           //===�����޷�
	if (Encoder_Integral < -10000)	Encoder_Integral = -10000;          //===�����޷�
	Velocity = Encoder*Velocity_Kp + Encoder_Integral*Velocity_Ki;      //===�ٶȿ���
	return Velocity;
}

/**************************************************************************
�������ܣ�ת�����
��ڲ��������ֱ����������ֱ�������Z��������
����  ֵ��ת�����PWM
**************************************************************************/
int turn(int encoder_left, int encoder_right, float gyro)//ת�����
{
	static float Turn_Target = 0, Turn = 0;
	Turn = -Turn_Target*Turn_Kp - gyro*Turn_Kd;  //===���Z�������ǽ���PD����
	return Turn;
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
	DataScope_Get_Channel_Data(moto1, 2);
	DataScope_Get_Channel_Data(moto2, 3);
	Set_Pwm(moto1, moto2);
}

void SafeCheck(void)
{
	if ((Angle_Balance > 35.0 || Angle_Balance < -35.0) && Motor_Close == 0)
	{
		Motor_Close = 1;
		PAout(4) = 1;
		setResultPwm(0, 0);
		printf("Turn OFF car.\r\n");
	}
}

unsigned char Send_Count; //������Ҫ���͵����ݸ���

void DataSend(void)
{
	int i;
	DataScope_Get_Channel_Data(Angle_Balance * 10, 1); // X10���ڹ۲�
	DataScope_Get_Channel_Data(Encoder_Left, 4);
	DataScope_Get_Channel_Data(Encoder_Right, 5);

	Send_Count = DataScope_Data_Generate(5);

	for (i = 0; i < Send_Count; i++)
	{
		while ((USART1->SR & 0X40) == 0);
		USART1->DR = DataScope_OutPut_Buffer[i];
	}
}

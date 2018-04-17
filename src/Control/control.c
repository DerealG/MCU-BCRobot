#include "control.h"	

u8 MPU6050_DMP_DATA = 0; // 1为DMP，0为原始数据
u8 filter_method = 2;  //1.卡尔曼滤波 2.一阶互补滤波

int Motor_Close = 0;

int Temperature = 0;
float battery_volt = 0;
int Encoder_Left = 0, Encoder_Right = 0;
float Angle_Balance, Gyro_Balance, Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
float Acceleration_Z;                         //Z轴加速度计
int Balance_Pwm = 0, Velocity_Pwm = 0, Turn_Pwm = 0;
int Moto1, Moto2;                             //电机PWM变量

float middle = -3.5;
float Balance_Kp = 250, Balance_Kd = 1;
float Velocity_Kp = 3, Velocity_Ki = 0.02;
float Turn_Kp = 0, Turn_Kd = 0;

//2018-4-17 19:20 {-3.5,400,1,2,0}
//2018-4-17 20:20 {-3.5,250,1,3,0.02}
/**************************************************************************
函数功能：所有的控制代码
		 5ms定时中断由MPU6050的INT引脚触发
		 严格保证采样和数据处理的时间同步
**************************************************************************/
int EXTI15_10_IRQHandler(void)
{
	if (MPU6050_INT == 0)
	{
		EXTI_ClearITPendingBit(EXTI_Line12);  	 //清除LINE12上的中断标志位   

		Temperature = Read_Temperature();      					 //读取MPU6050内置温度传感器数据
		battery_volt = Get_battery_volt();
		Encoder_Left = Read_Encoder(2);                //===读取编码器的值
		Encoder_Right = Read_Encoder(4);               //===读取编码器的值
		Get_Angle(MPU6050_DMP_DATA);                 //===更新姿态
		SafeCheck();
		Balance_Pwm = balance(Angle_Balance, Gyro_Balance);        //===平衡PID控制
		Velocity_Pwm = velocity(Encoder_Left, Encoder_Right);      //===速度环PID控制
		//Turn_Pwm = turn(Encoder_Left, Encoder_Right, Gyro_Turn);   //===转向环PID控制
		Moto1 = Balance_Pwm - Velocity_Pwm + Turn_Pwm;             //===计算左轮电机最终PWM
		Moto2 = Balance_Pwm - Velocity_Pwm - Turn_Pwm;             //===计算右轮电机最终PWM
		setResultPwm(Moto1, Moto2);

		//Data
		if ((USART_RX_STA & 0x8000) == 0x8000)
		{
			if (USART_RX_BUF[0] == 'W' && USART_RX_BUF[1] == ':') //校验包头
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
	delay_ms(10);	//消抖
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
函数功能：获取角度
入口参数：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
返回  值：无
**************************************************************************/
void Get_Angle(u8 dataSource)
{
	float Accel_Y, Accel_Angle, Accel_Z, Gyro_X, Gyro_Z;
	if (dataSource == 1)                           //===DMP的读取在数据采集中断读取，严格遵循时序要求
	{
		Read_DMP();                      //===读取加速度、角速度、倾角
		Angle_Balance = -Roll;             //===更新平衡倾角
		Gyro_Balance = -gyro[0];            //===更新平衡角速度
		Gyro_Turn = gyro[2];               //===更新转向角速度
		Acceleration_Z = accel[2];         //===更新Z轴加速度计
	}
	else
	{
		Gyro_X = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_XOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_XOUT_L);    //读取Y轴陀螺仪
		Gyro_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
		Accel_Y = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度计
		Accel_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计
		if (Gyro_X > 32768)  Gyro_X -= 65536;                       //数据类型转换  也可通过short强制类型转换
		if (Gyro_Z > 32768)  Gyro_Z -= 65536;                       //数据类型转换
		if (Accel_Y > 32768) Accel_Y -= 65536;                      //数据类型转换
		if (Accel_Z > 32768) Accel_Z -= 65536;                      //数据类型转换
		Gyro_Balance = Gyro_X;                                   //更新平衡角速度
		Accel_Angle = atan2(Accel_Y, Accel_Z) * 180 / PI;        //计算倾角
		Gyro_X = Gyro_X / 16.4;                                  //陀螺仪量程转换
		if (filter_method == 1)		  	Kalman_Filter(Accel_Angle, Gyro_X);//卡尔曼滤波
		else if (filter_method == 2)   First_Filter(Accel_Angle, Gyro_X);//互补滤波
		Angle_Balance = angle;                                   //更新平衡倾角
		Gyro_Turn = Gyro_Z;                                      //更新转向角速度
		Acceleration_Z = Accel_Z;                                //===更新Z轴加速度计
	}
}

/**************************************************************************
函数功能：直立PD控制
入口参数：角度、角速度
返回  值：直立控制PWM
**************************************************************************/
int balance(float Angle, float Gyro)
{
	float Bias;
	int balance;
	Bias = Angle - middle;       //===求出平衡的角度中值 和机械相关
	balance = Balance_Kp*Bias + Gyro*Balance_Kd;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	return balance;
}

/**************************************************************************
函数功能：速度PI控制 修改前进后退速度
入口参数：左轮编码器、右轮编码器
返回  值：速度控制PWM
**************************************************************************/
int velocity(int encoder_left, int encoder_right)
{
	static float Velocity = 0, Encoder_Least = 0, Encoder = 0;
	static float Encoder_Integral = 0;
	Encoder_Least = (Encoder_Left + Encoder_Right) - 0;                 //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
	Encoder *= 0.8;		                                                  //===一阶低通滤波器
	Encoder += Encoder_Least*0.2;	                                      //===一阶低通滤波器
	Encoder_Integral += Encoder;                                        //===积分出位移 积分时间：10ms
	Encoder_Integral = Encoder_Integral;                     //===接收遥控器数据，控制前进后退
	if (Encoder_Integral > 10000)  	Encoder_Integral = 10000;           //===积分限幅
	if (Encoder_Integral < -10000)	Encoder_Integral = -10000;          //===积分限幅
	Velocity = Encoder*Velocity_Kp + Encoder_Integral*Velocity_Ki;      //===速度控制
	return Velocity;
}

/**************************************************************************
函数功能：转向控制
入口参数：左轮编码器、右轮编码器、Z轴陀螺仪
返回  值：转向控制PWM
**************************************************************************/
int turn(int encoder_left, int encoder_right, float gyro)//转向控制
{
	static float Turn_Target = 0, Turn = 0;
	Turn = -Turn_Target*Turn_Kp - gyro*Turn_Kd;  //===结合Z轴陀螺仪进行PD控制
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

unsigned char Send_Count; //串口需要发送的数据个数

void DataSend(void)
{
	int i;
	DataScope_Get_Channel_Data(Angle_Balance * 10, 1); // X10利于观察
	DataScope_Get_Channel_Data(Encoder_Left, 4);
	DataScope_Get_Channel_Data(Encoder_Right, 5);

	Send_Count = DataScope_Data_Generate(5);

	for (i = 0; i < Send_Count; i++)
	{
		while ((USART1->SR & 0X40) == 0);
		USART1->DR = DataScope_OutPut_Buffer[i];
	}
}

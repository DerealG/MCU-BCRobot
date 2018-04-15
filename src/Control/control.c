#include "control.h"	

u8 filter_method = 2;  //选择卡尔曼滤波

int Motor_Close = 0;

int Encoder_Left = 0, Encoder_Right = 0;
float Angle_Balance, Gyro_Balance, Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
float Acceleration_Z;                         //Z轴加速度计
int Balance_Pwm = 0, Velocity_Pwm = 0, Turn_Pwm = 0;
int Moto1, Moto2;                             //电机PWM变量 应是Motor的 向Moto致敬
float Balance_Kp = 300, Balance_Kd = 1;
float Velocity_Kp = 80, Velocity_Ki = 0.4;
/**************************************************************************
函数功能：所有的控制代码
		 5ms定时中断由MPU6050的INT引脚触发
		 严格保证采样和数据处理的时间同步
**************************************************************************/
int EXTI15_10_IRQHandler(void)
{
	if (PAin(12) == 0)
	{
		EXTI_ClearITPendingBit(EXTI_Line12);  	 //清除LINE12上的中断标志位   

		if ((USART_RX_STA & 0x8000) == 0x8000)
		{
			if (USART_RX_BUF[0] == 'W' && USART_RX_BUF[1] == ':') //校验包头
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

		Encoder_Left = -Read_Encoder(2);                //===读取编码器的值
		Encoder_Right = Read_Encoder(4);                //===读取编码器的值
		Get_Angle(filter_method);                       //===更新姿态
		SafeCheck();
		Balance_Pwm = balance(Angle_Balance, Gyro_Balance);        //===平衡PID控制	
		Velocity_Pwm = velocity(Encoder_Left, Encoder_Right);      //===速度环PID控制
		Moto1 = Balance_Pwm - Velocity_Pwm + Turn_Pwm;             //===计算左轮电机最终PWM
		Moto2 = Balance_Pwm - Velocity_Pwm - Turn_Pwm;             //===计算右轮电机最终PWM
		setResultPwm(Moto1, Moto2);
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
void Get_Angle(u8 way)
{
	float Accel_Y, Accel_X, Accel_Z, Gyro_Y, Gyro_Z;
	Gyro_Y = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪
	Gyro_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
	Accel_X = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度计
	Accel_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计
	if (Gyro_Y > 32768)  Gyro_Y -= 65536;                    //数据类型转换  
	if (Gyro_Z > 32768)  Gyro_Z -= 65536;                    //数据类型转换
	if (Accel_X > 32768) Accel_X -= 65536;                   //数据类型转换
	if (Accel_Z > 32768) Accel_Z -= 65536;                   //数据类型转换
	Gyro_Balance = -Gyro_Y;                                  //更新平衡角速度
	Accel_Y = atan2(Accel_X, Accel_Z) * 180 / PI;            //计算倾角	
	Gyro_Y = Gyro_Y / 16.4;                                  //陀螺仪量程转换	
	Kalman_Filter(Accel_Y, -Gyro_Y);						 //卡尔曼滤波	
	Angle_Balance = angle;                                   //更新平衡倾角
	Gyro_Turn = Gyro_Z;                                      //更新转向角速度
	Acceleration_Z = Accel_Z;                                //===更新Z轴加速度计	
}

int balance(float Angle, float Gyro)
{
	float Bias;
	int balance;
	Bias = Angle - MIDDLE;       //===求出平衡的角度中值 和机械相关
	balance = Balance_Kp*Bias + Gyro*Balance_Kd;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	return balance;
}

int velocity(int encoder_left, int encoder_right)
{
	static float Velocity, Encoder_Least, Encoder, Movement;
	static float Encoder_Integral, Target_Velocity;
	Target_Velocity = 45;
	Movement = Target_Velocity / 2;
	Encoder_Least = (Encoder_Left + Encoder_Right) - 0;                 //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
	Encoder *= 0.8;		                                                //===一阶低通滤波器       
	Encoder += Encoder_Least*0.2;	                                    //===一阶低通滤波器    
	Encoder_Integral += Encoder;                                        //===积分出位移 积分时间：10ms
	Encoder_Integral = Encoder_Integral - Movement;                     //===接收遥控器数据，控制前进后退
	if (Encoder_Integral > 10000)  	Encoder_Integral = 10000;           //===积分限幅
	if (Encoder_Integral < -10000)	Encoder_Integral = -10000;          //===积分限幅	
	Velocity = Encoder*Velocity_Kp + Encoder_Integral*Velocity_Ki;      //===速度控制	
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

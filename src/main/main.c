#include <sys.h>

int main() {

	/* 系统 */
	//Stm32_Clock_Init(9);            //系统时钟设置
	delay_init();
	uart3_init(9600);
	uart1_init(256000);
	printf("Init.\r\n");
	MY_NVIC_PriorityGroupConfig(2);
	LED_Init();		//LED指示
	KEY_Init();		//按键

	/* 传感base */
	IIC_Init();                     //模拟IIC初始化
	MPU6050_initialize();           //MPU6050初始化	
	DMP_Init();                     //初始化DMP
	Adc_Init(); 	//电压测量
	Encoder_Init_TIM2(); //编码器测速1
	Encoder_Init_TIM4(); //编码器测速2

	/* 控制 */
	MiniBalance_PWM_Init(7199, 0);   //初始化PWM 10KHZ，用于驱动电机
	Set_Pwm(0, 0);
	Motor_Close = 1;

	//控制中断
	MPU6050_EXTI_Init();

	printf("Start.\r\n");
	while (1)
	{

	}
	//return 0;
}

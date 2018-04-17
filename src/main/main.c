#include <sys.h>

int main() {

	/* ϵͳ */
	//Stm32_Clock_Init(9);            //ϵͳʱ������
	delay_init();
	uart3_init(9600);
	uart1_init(256000);
	printf("Init.\r\n");
	MY_NVIC_PriorityGroupConfig(2);
	LED_Init();		//LEDָʾ
	KEY_Init();		//����

	/* ����base */
	IIC_Init();                     //ģ��IIC��ʼ��
	MPU6050_initialize();           //MPU6050��ʼ��	
	DMP_Init();                     //��ʼ��DMP
	Adc_Init(); 	//��ѹ����
	Encoder_Init_TIM2(); //����������1
	Encoder_Init_TIM4(); //����������2

	/* ���� */
	MiniBalance_PWM_Init(7199, 0);   //��ʼ��PWM 10KHZ�������������
	Set_Pwm(0, 0);
	Motor_Close = 1;

	//�����ж�
	MPU6050_EXTI_Init();

	printf("Start.\r\n");
	while (1)
	{

	}
	//return 0;
}

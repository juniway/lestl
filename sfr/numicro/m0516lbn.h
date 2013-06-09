#ifndef __M0516LBN_H
#define __M0516LBN_H

//*** <<< Use Configuration Wizard in Context Menu >>> ***
//	<o0> �ⲿ 4~24 MHz ����Ƶ��
//		<4000000-240000000>
#define __XTAL		0x00B71B00

//	<h> Config0 ����
//		<o0.1> ��ȫ����
//			<i> Config0.LOCK
//		<o0.7> ����ѡ��
//			<i> Config0.CBS
//			<1=> �� LDROM ����
//			<0=> �� APROM ����
//		<o0.20> Ƿѹ��λʹ��
//			<i> Config0.CBORST
//		<o0.21..22> Ƿѹ��ѹѡ��
//			<i> Config0.CBOV1-0
//			<3=> 2.2V
//			<2=> 2.7V
//			<1=> 3.8V
//			<0=> 4.5V
//		<o0.23> Ƿѹ���ʹ��
//			<i> Config0.CBODEN
//		<o0.24..26> ��λ�� CPU ʱ��Դѡ��
//			<i> Config0.CFOSC
//			<7=> �ⲿ 4~24 MHz ����
//			<0=> �ڲ� 22.1184 MHz ����
//		<o0.28> XT1 ʱ���˲�ʹ��
//			<i> Config0.CKF
//			<#+1>
//	</h>
#define __CONFIG0_VAL		~0x07F00000

//	<h> ʱ������
#define __PWRCON_DEFAULT	(0x0000001C | ((~__CONFIG0_VAL >> 24) & 1))
#define __AHBCLK_DEFAULT	0x0000000D
#define __APBCLK_DEFAULT	0x00000001
#define __CLKSEL0_DEFAULT	(0x00000038 | ((__CONFIG0_VAL >> 24) & 7))
#define __CLKSEL1_DEFAULT	0xFFFFFFFF
#define __CLKDIV_DEFAULT	0x00000000
#define __CLKSEL2_DEFAULT	0x000000FF
#define __PLLCON_DEFAULT	0x0005C22E
#define __FRQDIV_DEFAULT	0x00000000
//		<o0> �ⲿ 4~24 MHz ����ʹ��
//			<i> PWRCON.XTL12M_EN
//			<2=> �� Config0.CFOSC����λ�� CPU ʱ��Դѡ�������Զ�ȷ��
//			<0=> ����
//			<1=> ʹ��
//		<o1.2> �ڲ� 22.1184 MHz ����ʹ��
//			<i> PWRCON.OSC22M_EN
//		<o1.3> �ڲ� 10 KHz ����ʹ��
//			<i> PWRCON.OSC10K_EN
//		<h> PLL ����
//			<i> PLLCON
//			<o8.19> ʱ��Դ
//				<i> PLLCON.PLL_SRC(FIN)
//				<0=> �ⲿ 4~24 MHz ����
//				<1=> �ڲ� 22.1184 MHz ����
//			<o8.0..8> ������Ƶ����
//				<i> PLLCON.FB_DV(NF)
//				<2-513><#-2>
//			<o8.9..13> �����Ƶ����
//				<i> PLLCON.IN_DV(NR)
//				<2-17><#-2>
//			<o8.14..15> �����Ƶ����
//				<i> PLLCON.OUT_DV(NO)
//				<0=> 1
//				<1=> 2
//				<3=> 4
//			<o8.16> ����ģʽ
//				<i> PLLCON.PD
//			<o8.17> ��·
//				<i> PLLCON.BP
//			<o8.18> ���ʹ��
//				<i> PLLCON.OE
//				<0=> ʹ��
//				<1=> ����
//		</h>
//		<h> ϵͳʱ������
//			<i> CLKSEL0, CLKDIV
//			<o4> HCLK ʱ��Դ
//				<i> CLKSEL0.HCLK_S
//				<8=> �� Config0.CFOSC����λ�� CPU ʱ��Դѡ�������Զ�ȷ��
//				<0=> �ⲿ 4~24 MHz ����
//				<2=> PLL ʱ��
//				<3=> �ڲ� 10 KHz ����ʱ��
//				<7=> �ڲ� 22.1184 MHz ����
//			<o6.0..3> HCLK ʱ�ӷ�Ƶ��
//				<i> CLKDIV.HCLK_N
//				<1-16><#-1>
//		</h>
//		<h> AHB �豸ʱ������
//			<i> AHBCLK
//			<o2.2> Flash ISP ������ʱ��ʹ��
//				<i> AHBCLK.ISP_EN
//			<o2.3> �����߽ӿڿ�����ʱ��ʹ��
//				<i> AHBCLK.EBI_EN
//		</h>
//		<h> APB �豸ʱ������
//			<i> APBCLK, CLKSEL1, CLKSEL2, CLKDIV
//			<o5.0..1> Watchdog ʱ��Դѡ��
//				<i> CLKSEL1.WDT_S
//				<2=> HCLK / 2048
//				<3=> �ڲ� 10 KHz ����ʱ��
//			<o3.0> Watchdog ʱ��ʹ��
//				<i> APBCLK.WDT_EN
//			<o5.2..3> ADC ʱ��Դѡ��
//				<i> CLKSEL1.ADC_S
//				<0=> �ⲿ 4~24 MHz ����ʱ��
//				<1=> PLL ʱ��
//				<3=> �ڲ� 22.1184 MHz ����ʱ��
//			<o6.16..23>	ADC ʱ�ӷ�Ƶ��
//				<i> CLKDIV.ADC_N
//				<1-256><#-1>
//			<o3.28> ADC ʱ��ʹ��
//				<i> APBCLK.ADC_EN
//			<o5.8..10> TIMER0 ʱ��Դѡ��
//				<i> CLKSEL1.TMR0_S
//				<0=> �ⲿ 4~24 MHz ����ʱ��
//				<2=> HCLK
//				<7=> �ڲ� 22.1184 MHz ����ʱ��
//			<o3.2> TIMER0 ʱ��ʹ��
//				<i> APBCLK.TMR0_EN
//			<o5.12..14> TIMER1 ʱ��Դѡ��
//				<i> CLKSEL1.TMR1_S
//				<0=> �ⲿ 4~24 MHz ����ʱ��
//				<2=> HCLK
//				<7=> �ڲ� 22.1184 MHz ����ʱ��
//			<o3.3> TIMER1 ʱ��ʹ��
//				<i> APBCLK.TMR1_EN
//			<o5.16..18> TIMER2 ʱ��Դѡ��
//				<i> CLKSEL1.TMR2_S
//				<0=> �ⲿ 4~24 MHz ����ʱ��
//				<2=> HCLK
//				<7=> �ڲ� 22.1184 MHz ����ʱ��
//			<o3.4> TIMER2 ʱ��ʹ��
//				<i> APBCLK.TMR2_EN
//			<o5.20..22> TIMER3 ʱ��Դѡ��
//				<i> CLKSEL1.TMR3_S
//				<0=> �ⲿ 4~24 MHz ����ʱ��
//				<2=> HCLK
//				<7=> �ڲ� 22.1184 MHz ����ʱ��
//			<o3.5> TIMER3 ʱ��ʹ��
//				<i> APBCLK.TMR3_EN
//			<o3.8> I2C ʱ��ʹ��
//				<i> APBCLK.I2C0_EN
//			<o3.12> SPI0 ʱ��ʹ��
//				<i> APBCLK.SPI0_EN
//			<o3.13> SPI1 ʱ��ʹ��
//				<i> APBCLK.SPI1_EN
//			<o5.24..25> UART ʱ��Դѡ��
//				<i> CLKSEL1.UART_S
//				<0=> �ⲿ 4~24 MHz ����ʱ��
//				<1=> PLL ʱ��
//				<3=> �ڲ� 22.1184 MHz ����ʱ��
//			<o6.8..11> UART ʱ�ӷ�Ƶ��
//				<i> CLKDIV.UART_N
//				<1-16><#-1>
//			<o3.16>UART0 ʱ��ʹ��
//				<i> APBCLK.UART0_EN
//			<o3.17>UART1 ʱ��ʹ��
//				<i> APBCLK.UART1_EN
//			<o5.28..29> PWM0 �� PWM1 ʱ��Դѡ��
//				<i> CLKSEL1.PWM01_S
//				<0=> �ⲿ 4~24 MHz ����ʱ��
//				<2=> HCLK
//				<3=> �ڲ� 22.1184 MHz ����ʱ��
//			<o3.20> PWM0 �� PWM1 ʱ��ʹ��
//				<i> APBCLK.PWM01_EN
//			<o5.30..31> PWM2 �� PWM3 ʱ��Դѡ��
//				<i> CLKSEL1.PWM23_S
//				<0=> �ⲿ 4~24 MHz ����ʱ��
//				<2=> HCLK
//				<3=> �ڲ� 22.1184 MHz ����ʱ��
//			<o3.21> PWM2 �� PWM3 ʱ��ʹ��
//				<i> APBCLK.PWM23_EN
//			<o7.2..3> ��Ƶ��ʱ��Դѡ��
//				<i> CLKSEL2.FRQDIV_S
//				<0=> �ⲿ 4~24 MHz ����ʱ��
//				<2=> HCLK
//				<3=> �ڲ� 22.1184 MHz ����ʱ��
//			<o3.6> ��Ƶ��ʱ��ʹ��
//				<i> APBCLK.FDIV_EN
//			<o9.0..3> ��Ƶ��ʱ�ӷ�Ƶ��
//				<i> FRQDIV.FSEL
//				<0=> 2
//				<1=> 4
//				<2=> 8
//				<3=> 16
//				<4=> 32
//				<5=> 64
//				<6=> 128
//				<7=> 256
//				<8=> 512
//				<9=> 1024
//				<10=> 2048
//				<11=> 4096
//				<12=> 8192
//				<13=> 16384
//				<14=> 32768
//				<15=> 65536
//			<o9.4> ��Ƶ����Ƶʹ��
//				<i> FRQDIV.DIVIDER_EN
//			<o7.4..5> PWM4 �� PWM5 ʱ��Դѡ��
//				<i> CLKSEL2.PWM45_S
//				<0=> �ⲿ 4~24 MHz ����ʱ��
//				<2=> HCLK
//				<3=> �ڲ� 22.1184 MHz ����ʱ��
//			<o3.22> PWM4 �� PWM5 ʱ��ʹ��
//				<i> APBCLK.PWM45_EN
//			<o7.6..7> PWM6 �� PWM7 ʱ��Դѡ��
//				<i> CLKSEL2.PWM67_S
//				<0=> �ⲿ 4~24 MHz ����ʱ��
//				<2=> HCLK
//				<3=> �ڲ� 22.1184 MHz ����ʱ��
//			<o3.23> PWM6 �� PWM7 ʱ��ʹ��
//				<i> APBCLK.PWM67_EN
//		</h>
//	</h>
//	<h> ���缰��������
//		<o1.4> ������ʱ������ʹ��
//			<i> PWRCON.PD_WU_DLY
//		<o1.5> �����ж�ʹ��
//			<i> PWRCON.PD_WU_INT_EN
//		<o1.7> ϵͳ����ʹ��
//			<i> PWRCON.PWR_DOWN_EN
//		<o1.8> �ȴ� CPU ִ�� WFI ָ��
//			<i> PWRCON.PD_WAIT_CPU
//	</h>
#define __XTL12M_PAR		2
#define __PWRCON_PAR		0x0000003C
#define __AHBCLK_VAL		0x0000000D
#define __APBCLK_VAL		0x00010021
#define __HCLK_PAR			0x00000008
#define __CLKSEL1_VAL		0xFCFFFFFF
#define __CLKDIV_VAL		0x00000000
#define __CLKSEL2_VAL		0x000000FF
#define __PLLCON_VAL		0x0000C22E
#define __FRQDIV_VAL		0x00000000

#if __XTL12M_PAR == 2
#define __PWRCON_VAL	(__PWRCON_PAR | ((~__CONFIG0_VAL >> 24) & 1))
#else
#define __PWRCON_VAL	(__PWRCON_PAR | __XTL12M_PAR)
#endif

#if __HCLK_PAR == 8
#define __CLKSEL0_VAL		(((__CONFIG0_VAL >> 24) & 7) | 0x38)
#else
#define __CLKSEL0_VAL		(__HCLK_PAR | 0x38)
#endif

//	<h> Ƿѹ�������
#define __BODCR_DEFAULT		(0x00000080 | ((~__CONFIG0_VAL >> 23) & 1) | ((__CONFIG0_VAL >> 20) & 6) | ((~__CONFIG0_VAL >> 17) & 8))
//		<o0.5> ʹ�ܵ͹���ģʽ
//			<i> BODCR.BOD_LPM
//		<o0.7> ��ѹ��λʹ��
//			<i> BODCR.LVR_EN
//	</h>
#define __BODCR_VAL			(0x00000080 | ((~__CONFIG0_VAL >> 23) & 1) | ((__CONFIG0_VAL >> 20) & 6) | ((~__CONFIG0_VAL >> 17) & 8))

//	<o0.0> �����ϵ縴λ
#define __PORCR_VAL			0

//	<o0.0> ��ʼ���������Ĵ���д������
#define __LOCK_SETUP		0

//	<h> P0 �ܽ�����
//		<h> P0.0/AD0/CTS1
//			<i> Pin: 40
//			<o0.0..1> �๦�ܹܽ�ѡ��
//				<i> P0_MFP.P0_MFP[0], P0_MFP.P0_ALT[0]
//				<0=> P0.0
//				<1=> AD0
//				<2=> CTS1
//			<o0.16> ʩ���ش�������ʹ��
//				<i> P0_MFP.P0_TYPE[0]
//			<o1.0..1> �˿�ģʽ
//				<i> P0_PMD.PMD0
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.16> OFF ��������ͨ��ʹ��
//				<i> P0_OFFD.OFFD[0]
//			<o3.0> ���ֵ
//				<i> P0_DOUT.DOUT[0]
//				<0=> ��
//				<1=> ��
//			<o4.0> ȥ��ʹ��
//				<i> P0_DBEN.DBEN[0]
//			<o5.0> �ж�ģʽ
//				<i> P0_IMD.IMD[0]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P0.1/AD1/RTS1
//			<i> Pin: 39
//			<o0.2..3> �๦�ܹܽ�ѡ��
//				<i> P0_MFP.P0_MFP[1], P0_MFP.P0_ALT[1]
//				<0=> P0.1
//				<1=> AD1
//				<2=> RTS1
//			<o0.17> ʩ���ش�������ʹ��
//				<i> P0_MFP.P0_TYPE[1]
//			<o1.2..3> �˿�ģʽ
//				<i> P0_PMD.PMD1
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.17> OFF ��������ͨ��ʹ��
//				<i> P0_OFFD.OFFD[1]
//			<o3.1> ���ֵ
//				<i> P0_DOUT.DOUT[1]
//				<0=> ��
//				<1=> ��
//			<o4.1> ȥ��ʹ��
//				<i> P0_DBEN.DBEN[1]
//			<o5.1> �ж�ģʽ
//				<i> P0_IMD.IMD[1]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P0.2/AD2/CTS0
//			<i> Pin: 38
//			<o0.4..5> �๦�ܹܽ�ѡ��
//				<i> P0_MFP.P0_MFP[2], P0_MFP.P0_ALT[2]
//				<0=> P0.2
//				<1=> AD2
//				<2=> CTS0
//			<o0.18> ʩ���ش�������ʹ��
//				<i> P0_MFP.P0_TYPE[2]
//			<o1.4..5> �˿�ģʽ
//				<i> P0_PMD.PMD2
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.18> OFF ��������ͨ��ʹ��
//				<i> P0_OFFD.OFFD[2]
//			<o3.2> ���ֵ
//				<i> P0_DOUT.DOUT[2]
//				<0=> ��
//				<1=> ��
//			<o4.2> ȥ��ʹ��
//				<i> P0_DBEN.DBEN[2]
//			<o5.2> �ж�ģʽ
//				<i> P0_IMD.IMD[2]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P0.3/AD3/RTS0
//			<i> Pin: 37
//			<o0.6..7> �๦�ܹܽ�ѡ��
//				<i> P0_MFP.P0_MFP[3], P0_MFP.P0_ALT[3]
//				<0=> P0.3
//				<1=> AD3
//				<2=> RTS0
//			<o0.19> ʩ���ش�������ʹ��
//				<i> P0_MFP.P0_TYPE[3]
//			<o1.6..7> �˿�ģʽ
//				<i> P0_PMD.PMD3
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.19> OFF ��������ͨ��ʹ��
//				<i> P0_OFFD.OFFD[3]
//			<o3.3> ���ֵ
//				<i> P0_DOUT.DOUT[3]
//				<0=> ��
//				<1=> ��
//			<o4.3> ȥ��ʹ��
//				<i> P0_DBEN.DBEN[3]
//			<o5.3> �ж�ģʽ
//				<i> P0_IMD.IMD[3]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P0.4/AD4/SPISS1
//			<i> Pin: 35
//			<o0.8..9> �๦�ܹܽ�ѡ��
//				<i> P0_MFP.P0_MFP[4], P0_MFP.P0_ALT[4]
//				<0=> P0.4
//				<1=> AD4
//				<2=> SPISS1
//			<o0.20> ʩ���ش�������ʹ��
//				<i> P0_MFP.P0_TYPE[4]
//			<o1.8..9> �˿�ģʽ
//				<i> P0_PMD.PMD4
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.20> OFF ��������ͨ��ʹ��
//				<i> P0_OFFD.OFFD[4]
//			<o3.4> ���ֵ
//				<i> P0_DOUT.DOUT[4]
//				<0=> ��
//				<1=> ��
//			<o4.4> ȥ��ʹ��
//				<i> P0_DBEN.DBEN[4]
//			<o5.4> �ж�ģʽ
//				<i> P0_IMD.IMD[4]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P0.5/AD5/MOSI1
//			<i> Pin: 34
//			<o0.10..11> �๦�ܹܽ�ѡ��
//				<i> P0_MFP.P0_MFP[5], P0_MFP.P0_ALT[5]
//				<0=> P0.5
//				<1=> AD5
//				<2=> MOSI1
//			<o0.21> ʩ���ش�������ʹ��
//				<i> P0_MFP.P0_TYPE[5]
//			<o1.10..11> �˿�ģʽ
//				<i> P0_PMD.PMD5
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.21> OFF ��������ͨ��ʹ��
//				<i> P0_OFFD.OFFD[5]
//			<o3.5> ���ֵ
//				<i> P0_DOUT.DOUT[5]
//				<0=> ��
//				<1=> ��
//			<o4.5> ȥ��ʹ��
//				<i> P0_DBEN.DBEN[5]
//			<o5.5> �ж�ģʽ
//				<i> P0_IMD.IMD[5]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P0.6/AD6/MISO1
//			<i> Pin: 33
//			<o0.12..13> �๦�ܹܽ�ѡ��
//				<i> P0_MFP.P0_MFP[6], P0_MFP.P0_ALT[6]
//				<0=> P0.6
//				<1=> AD6
//				<2=> MISO1
//			<o0.22> ʩ���ش�������ʹ��
//				<i> P0_MFP.P0_TYPE[6]
//			<o1.12..13> �˿�ģʽ
//				<i> P0_PMD.PMD6
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.22> OFF ��������ͨ��ʹ��
//				<i> P0_OFFD.OFFD[6]
//			<o3.6> ���ֵ
//				<i> P0_DOUT.DOUT[6]
//				<0=> ��
//				<1=> ��
//			<o4.6> ȥ��ʹ��
//				<i> P0_DBEN.DBEN[6]
//			<o5.6> �ж�ģʽ
//				<i> P0_IMD.IMD[6]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P0.7/AD7/SPICLK1
//			<i> Pin: 32
//			<o0.14..15> �๦�ܹܽ�ѡ��
//				<i> P0_MFP.P0_MFP[7], P0_MFP.P0_ALT[7]
//				<0=> P0.7
//				<1=> AD7
//				<2=> SPICLK1
//			<o0.23> ʩ���ش�������ʹ��
//				<i> P0_MFP.P0_TYPE[7]
//			<o1.14..15> �˿�ģʽ
//				<i> P0_PMD.PMD7
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.23> OFF ��������ͨ��ʹ��
//				<i> P0_OFFD.OFFD[7]
//			<o3.7> ���ֵ
//				<i> P0_DOUT.DOUT[7]
//				<0=> ��
//				<1=> ��
//			<o4.7> ȥ��ʹ��
//				<i> P0_DBEN.DBEN[7]
//			<o5.7> �ж�ģʽ
//				<i> P0_IMD.IMD[7]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//	</h>
#define __P0MFP_VAL			0x00FF0000
#define __P0PMD_VAL			0x00000000
#define __P0OFFD_VAL		0x00000000
#define __P0DOUT_VAL		0x000000FF
#define __P0DBEN_VAL		0x000000FF
#define __P0IMD_VAL			0x00000000

//	<h> P1 �ܽ�����
//		<h> P1.0/AIN0/T2
//			<i> Pin: 43
//			<o0.0..1> �๦�ܹܽ�ѡ��
//				<i> P1_MFP.P1_MFP[0], P1_MFP.P1_ALT[0]
//				<0=> P1.0
//				<1=> AIN0
//				<2=> T2
//			<o0.16> ʩ���ش�������ʹ��
//				<i> P1_MFP.P1_TYPE[0]
//			<o1.0..1> �˿�ģʽ
//				<i> P1_PMD.PMD0
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.16> OFF ��������ͨ��ʹ��
//				<i> P1_OFFD.OFFD[0]
//			<o3.0> ���ֵ
//				<i> P1_DOUT.DOUT[0]
//				<0=> ��
//				<1=> ��
//			<o4.0> ȥ��ʹ��
//				<i> P1_DBEN.DBEN[0]
//			<o5.0> �ж�ģʽ
//				<i> P1_IMD.IMD[0]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P1.1/AIN1/T3
//			<i> Pin: 44
//			<o0.2..3> �๦�ܹܽ�ѡ��
//				<i> P1_MFP.P1_MFP[1], P1_MFP.P1_ALT[1]
//				<0=> P1.1
//				<1=> AIN1
//				<2=> T3
//			<o0.17> ʩ���ش�������ʹ��
//				<i> P1_MFP.P1_TYPE[1]
//			<o1.2..3> �˿�ģʽ
//				<i> P1_PMD.PMD1
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.17> OFF ��������ͨ��ʹ��
//				<i> P1_OFFD.OFFD[1]
//			<o3.1> ���ֵ
//				<i> P1_DOUT.DOUT[1]
//				<0=> ��
//				<1=> ��
//			<o4.1> ȥ��ʹ��
//				<i> P1_DBEN.DBEN[1]
//			<o5.1> �ж�ģʽ
//				<i> P1_IMD.IMD[1]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P1.2/AIN2/RXD1
//			<i> Pin: 45
//			<o0.4..5> �๦�ܹܽ�ѡ��
//				<i> P1_MFP.P1_MFP[2], P1_MFP.P1_ALT[2]
//				<0=> P1.2
//				<1=> AIN2
//				<2=> RXD1
//			<o0.18> ʩ���ش�������ʹ��
//				<i> P1_MFP.P1_TYPE[2]
//			<o1.4..5> �˿�ģʽ
//				<i> P1_PMD.PMD2
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.18> OFF ��������ͨ��ʹ��
//				<i> P1_OFFD.OFFD[2]
//			<o3.2> ���ֵ
//				<i> P1_DOUT.DOUT[2]
//				<0=> ��
//				<1=> ��
//			<o4.2> ȥ��ʹ��
//				<i> P1_DBEN.DBEN[2]
//			<o5.2> �ж�ģʽ
//				<i> P1_IMD.IMD[2]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P1.3/AIN3/TXD1
//			<i> Pin: 46
//			<o0.6..7> �๦�ܹܽ�ѡ��
//				<i> P1_MFP.P1_MFP[3], P1_MFP.P1_ALT[3]
//				<0=> P1.3
//				<1=> AIN3
//				<2=> RTS0
//			<o0.19> ʩ���ش�������ʹ��
//				<i> P1_MFP.P1_TYPE[3]
//			<o1.6..7> �˿�ģʽ
//				<i> P1_PMD.PMD3
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.19> OFF ��������ͨ��ʹ��
//				<i> P1_OFFD.OFFD[3]
//			<o3.3> ���ֵ
//				<i> P1_DOUT.DOUT[3]
//				<0=> ��
//				<1=> ��
//			<o4.3> ȥ��ʹ��
//				<i> P1_DBEN.DBEN[3]
//			<o5.3> �ж�ģʽ
//				<i> P1_IMD.IMD[3]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P1.4/AIN4/SPISS0
//			<i> Pin: 47
//			<o0.8..9> �๦�ܹܽ�ѡ��
//				<i> P1_MFP.P1_MFP[4], P1_MFP.P1_ALT[4]
//				<0=> P1.4
//				<1=> AIN4
//				<2=> SPISS0
//			<o0.20> ʩ���ش�������ʹ��
//				<i> P1_MFP.P1_TYPE[4]
//			<o1.8..9> �˿�ģʽ
//				<i> P1_PMD.PMD4
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.20> OFF ��������ͨ��ʹ��
//				<i> P1_OFFD.OFFD[4]
//			<o3.4> ���ֵ
//				<i> P1_DOUT.DOUT[4]
//				<0=> ��
//				<1=> ��
//			<o4.4> ȥ��ʹ��
//				<i> P1_DBEN.DBEN[4]
//			<o5.4> �ж�ģʽ
//				<i> P1_IMD.IMD[4]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P1.5/AIN5/MOSI0
//			<i> Pin: 1
//			<o0.10..11> �๦�ܹܽ�ѡ��
//				<i> P1_MFP.P1_MFP[5], P1_MFP.P1_ALT[5]
//				<0=> P1.5
//				<1=> AIN5
//				<2=> MOSI0
//			<o0.21> ʩ���ش�������ʹ��
//				<i> P1_MFP.P1_TYPE[5]
//			<o1.10..11> �˿�ģʽ
//				<i> P1_PMD.PMD5
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.21> OFF ��������ͨ��ʹ��
//				<i> P1_OFFD.OFFD[5]
//			<o3.5> ���ֵ
//				<i> P1_DOUT.DOUT[5]
//				<0=> ��
//				<1=> ��
//			<o4.5> ȥ��ʹ��
//				<i> P1_DBEN.DBEN[5]
//			<o5.5> �ж�ģʽ
//				<i> P1_IMD.IMD[5]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P1.6/AIN6/MISO0
//			<i> Pin: 2
//			<o0.12..13> �๦�ܹܽ�ѡ��
//				<i> P1_MFP.P1_MFP[6], P1_MFP.P1_ALT[6]
//				<0=> P1.6
//				<1=> AIN6
//				<2=> MISO0
//			<o0.22> ʩ���ش�������ʹ��
//				<i> P1_MFP.P1_TYPE[6]
//			<o1.12..13> �˿�ģʽ
//				<i> P1_PMD.PMD6
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.22> OFF ��������ͨ��ʹ��
//				<i> P1_OFFD.OFFD[6]
//			<o3.6> ���ֵ
//				<i> P1_DOUT.DOUT[6]
//				<0=> ��
//				<1=> ��
//			<o4.6> ȥ��ʹ��
//				<i> P1_DBEN.DBEN[6]
//			<o5.6> �ж�ģʽ
//				<i> P1_IMD.IMD[6]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P1.7/AIN7/SPICLK0
//			<i> Pin: 3
//			<o0.14..15> �๦�ܹܽ�ѡ��
//				<i> P1_MFP.P1_MFP[7], P1_MFP.P1_ALT[7]
//				<0=> P1.7
//				<1=> AIN7
//				<2=> SPICLK0
//			<o0.23> ʩ���ش�������ʹ��
//				<i> P1_MFP.P1_TYPE[7]
//			<o1.14..15> �˿�ģʽ
//				<i> P1_PMD.PMD7
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.23> OFF ��������ͨ��ʹ��
//				<i> P1_OFFD.OFFD[7]
//			<o3.7> ���ֵ
//				<i> P1_DOUT.DOUT[7]
//				<0=> ��
//				<1=> ��
//			<o4.7> ȥ��ʹ��
//				<i> P1_DBEN.DBEN[7]
//			<o5.7> �ж�ģʽ
//				<i> P1_IMD.IMD[7]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//	</h>
#define __P1MFP_VAL			0x00005555
#define __P1PMD_VAL			0x0000FFFF
#define __P1OFFD_VAL		0x00000000
#define __P1DOUT_VAL		0x000000FF
#define __P1DBEN_VAL		0x00000000
#define __P1IMD_VAL			0x00000000

//	<h> P2 �ܽ�����
//		<h> P2.0/AD8/PWM0
//			<i> Pin: 19
//			<o0.0..1> �๦�ܹܽ�ѡ��
//				<i> P2_MFP.P2_MFP[0], P2_MFP.P2_ALT[0]
//				<0=> P2.0
//				<1=> AD8
//				<2=> PWM0
//			<o0.16> ʩ���ش�������ʹ��
//				<i> P2_MFP.P2_TYPE[0]
//			<o1.0..1> �˿�ģʽ
//				<i> P2_PMD.PMD0
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.16> OFF ��������ͨ��ʹ��
//				<i> P2_OFFD.OFFD[0]
//			<o3.0> ���ֵ
//				<i> P2_DOUT.DOUT[0]
//				<0=> ��
//				<1=> ��
//			<o4.0> ȥ��ʹ��
//				<i> P2_DBEN.DBEN[0]
//			<o5.0> �ж�ģʽ
//				<i> P2_IMD.IMD[0]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P2.1/AD9/PWM1
//			<i> Pin: 20
//			<o0.2..3> �๦�ܹܽ�ѡ��
//				<i> P2_MFP.P2_MFP[1], P2_MFP.P2_ALT[1]
//				<0=> P2.1
//				<1=> AD9
//				<2=> PWM1
//			<o0.17> ʩ���ش�������ʹ��
//				<i> P2_MFP.P2_TYPE[1]
//			<o1.2..3> �˿�ģʽ
//				<i> P2_PMD.PMD1
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.17> OFF ��������ͨ��ʹ��
//				<i> P2_OFFD.OFFD[1]
//			<o3.1> ���ֵ
//				<i> P2_DOUT.DOUT[1]
//				<0=> ��
//				<1=> ��
//			<o4.1> ȥ��ʹ��
//				<i> P2_DBEN.DBEN[1]
//			<o5.1> �ж�ģʽ
//				<i> P2_IMD.IMD[1]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P2.2/AD10/PWM2
//			<i> Pin: 21
//			<o0.4..5> �๦�ܹܽ�ѡ��
//				<i> P2_MFP.P2_MFP[2], P2_MFP.P2_ALT[2]
//				<0=> P2.2
//				<1=> AD10
//				<2=> PWM2
//			<o0.18> ʩ���ش�������ʹ��
//				<i> P2_MFP.P2_TYPE[2]
//			<o1.4..5> �˿�ģʽ
//				<i> P2_PMD.PMD2
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.18> OFF ��������ͨ��ʹ��
//				<i> P2_OFFD.OFFD[2]
//			<o3.2> ���ֵ
//				<i> P2_DOUT.DOUT[2]
//				<0=> ��
//				<1=> ��
//			<o4.2> ȥ��ʹ��
//				<i> P2_DBEN.DBEN[2]
//			<o5.2> �ж�ģʽ
//				<i> P2_IMD.IMD[2]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P2.3/AD11/PWM3
//			<i> Pin: 22
//			<o0.6..7> �๦�ܹܽ�ѡ��
//				<i> P2_MFP.P2_MFP[3], P2_MFP.P2_ALT[3]
//				<0=> P2.3
//				<1=> AD11
//				<2=> PWM3
//			<o0.19> ʩ���ش�������ʹ��
//				<i> P2_MFP.P2_TYPE[3]
//			<o1.6..7> �˿�ģʽ
//				<i> P2_PMD.PMD3
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.19> OFF ��������ͨ��ʹ��
//				<i> P2_OFFD.OFFD[3]
//			<o3.3> ���ֵ
//				<i> P2_DOUT.DOUT[3]
//				<0=> ��
//				<1=> ��
//			<o4.3> ȥ��ʹ��
//				<i> P2_DBEN.DBEN[3]
//			<o5.3> �ж�ģʽ
//				<i> P2_IMD.IMD[3]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P2.4/AD12/PWM4
//			<i> Pin: 23
//			<o0.8..9> �๦�ܹܽ�ѡ��
//				<i> P2_MFP.P2_MFP[4], P2_MFP.P2_ALT[4]
//				<0=> P2.4
//				<1=> AD12
//				<2=> PWM4
//			<o0.20> ʩ���ش�������ʹ��
//				<i> P2_MFP.P2_TYPE[4]
//			<o1.8..9> �˿�ģʽ
//				<i> P2_PMD.PMD4
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.20> OFF ��������ͨ��ʹ��
//				<i> P2_OFFD.OFFD[4]
//			<o3.4> ���ֵ
//				<i> P2_DOUT.DOUT[4]
//				<0=> ��
//				<1=> ��
//			<o4.4> ȥ��ʹ��
//				<i> P2_DBEN.DBEN[4]
//			<o5.4> �ж�ģʽ
//				<i> P2_IMD.IMD[4]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P2.5/AD13/PWM5
//			<i> Pin: 25
//			<o0.10..11> �๦�ܹܽ�ѡ��
//				<i> P2_MFP.P2_MFP[5], P2_MFP.P2_ALT[5]
//				<0=> P2.5
//				<1=> AD13
//				<2=> PWM5
//			<o0.21> ʩ���ش�������ʹ��
//				<i> P2_MFP.P2_TYPE[5]
//			<o1.10..11> �˿�ģʽ
//				<i> P2_PMD.PMD5
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.21> OFF ��������ͨ��ʹ��
//				<i> P2_OFFD.OFFD[5]
//			<o3.5> ���ֵ
//				<i> P2_DOUT.DOUT[5]
//				<0=> ��
//				<1=> ��
//			<o4.5> ȥ��ʹ��
//				<i> P2_DBEN.DBEN[5]
//			<o5.5> �ж�ģʽ
//				<i> P2_IMD.IMD[5]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P2.6/AD14/PWM6
//			<i> Pin: 26
//			<o0.12..13> �๦�ܹܽ�ѡ��
//				<i> P2_MFP.P2_MFP[6], P2_MFP.P2_ALT[6]
//				<0=> P2.6
//				<1=> AD14
//				<2=> PWM6
//			<o0.22> ʩ���ش�������ʹ��
//				<i> P2_MFP.P2_TYPE[6]
//			<o1.12..13> �˿�ģʽ
//				<i> P2_PMD.PMD6
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.22> OFF ��������ͨ��ʹ��
//				<i> P2_OFFD.OFFD[6]
//			<o3.6> ���ֵ
//				<i> P2_DOUT.DOUT[6]
//				<0=> ��
//				<1=> ��
//			<o4.6> ȥ��ʹ��
//				<i> P2_DBEN.DBEN[6]
//			<o5.6> �ж�ģʽ
//				<i> P2_IMD.IMD[6]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P2.7/AD15/PWM7
//			<i> Pin: 27
//			<o0.14..15> �๦�ܹܽ�ѡ��
//				<i> P2_MFP.P2_MFP[7], P2_MFP.P2_ALT[7]
//				<0=> P2.7
//				<1=> AD15
//				<2=> PWM7
//			<o0.23> ʩ���ش�������ʹ��
//				<i> P2_MFP.P2_TYPE[7]
//			<o1.14..15> �˿�ģʽ
//				<i> P2_PMD.PMD7
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.23> OFF ��������ͨ��ʹ��
//				<i> P2_OFFD.OFFD[7]
//			<o3.7> ���ֵ
//				<i> P2_DOUT.DOUT[7]
//				<0=> ��
//				<1=> ��
//			<o4.7> ȥ��ʹ��
//				<i> P2_DBEN.DBEN[7]
//			<o5.7> �ж�ģʽ
//				<i> P2_IMD.IMD[7]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//	</h>
#define __P2MFP_VAL			0x00000000
#define __P2PMD_VAL			0x00005555
#define __P2OFFD_VAL		0x00000000
#define __P2DOUT_VAL		0x00000000
#define __P2DBEN_VAL		0x00000000
#define __P2IMD_VAL			0x00000000

//	<h> P3 �ܽ�����
//		<h> P3.0/RXD0
//			<i> Pin: 5
//			<o0.0..1> �๦�ܹܽ�ѡ��
//				<i> P3_MFP.P3_MFP[0]
//				<0=> P3.0
//				<1=> RXD0
//			<o0.16> ʩ���ش�������ʹ��
//				<i> P3_MFP.P3_TYPE[0]
//			<o1.0..1> �˿�ģʽ
//				<i> P3_PMD.PMD0
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.16> OFF ��������ͨ��ʹ��
//				<i> P3_OFFD.OFFD[0]
//			<o3.0> ���ֵ
//				<i> P3_DOUT.DOUT[0]
//				<0=> ��
//				<1=> ��
//			<o4.0> ȥ��ʹ��
//				<i> P3_DBEN.DBEN[0]
//			<o5.0> �ж�ģʽ
//				<i> P3_IMD.IMD[0]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P3.1/TXD0
//			<i> Pin: 7
//			<o0.2..3> �๦�ܹܽ�ѡ��
//				<i> P3_MFP.P3_MFP[1]
//				<0=> P3.1
//				<1=> TXD0
//			<o0.17> ʩ���ش�������ʹ��
//				<i> P3_MFP.P3_TYPE[1]
//			<o1.2..3> �˿�ģʽ
//				<i> P3_PMD.PMD1
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.17> OFF ��������ͨ��ʹ��
//				<i> P3_OFFD.OFFD[1]
//			<o3.1> ���ֵ
//				<i> P3_DOUT.DOUT[1]
//				<0=> ��
//				<1=> ��
//			<o4.1> ȥ��ʹ��
//				<i> P3_DBEN.DBEN[1]
//			<o5.1> �ж�ģʽ
//				<i> P3_IMD.IMD[1]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P3.2(STADC)/INT0
//			<i> Pin: 8
//			<o0.4..5> �๦�ܹܽ�ѡ��
//				<i> P3_MFP.P3_MFP[2]
//				<0=> P3.2(STADC)
//				<1=> INT0
//			<o0.18> ʩ���ش�������ʹ��
//				<i> P3_MFP.P3_TYPE[2]
//			<o1.4..5> �˿�ģʽ
//				<i> P3_PMD.PMD2
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.18> OFF ��������ͨ��ʹ��
//				<i> P3_OFFD.OFFD[2]
//			<o3.2> ���ֵ
//				<i> P3_DOUT.DOUT[2]
//				<0=> ��
//				<1=> ��
//			<o4.2> ȥ��ʹ��
//				<i> P3_DBEN.DBEN[2]
//			<o5.2> �ж�ģʽ
//				<i> P3_IMD.IMD[2]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P3.3/INT1/MCLK
//			<i> Pin: 9
//			<o0.6..7> �๦�ܹܽ�ѡ��
//				<i> P3_MFP.P3_MFP[3], P3_MFP.P3_ALT[3]
//				<0=> P3.3
//				<1=> INT1
//				<2=> MCLK
//			<o0.19> ʩ���ش�������ʹ��
//				<i> P3_MFP.P3_TYPE[3]
//			<o1.6..7> �˿�ģʽ
//				<i> P3_PMD.PMD3
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.19> OFF ��������ͨ��ʹ��
//				<i> P3_OFFD.OFFD[3]
//			<o3.3> ���ֵ
//				<i> P3_DOUT.DOUT[3]
//				<0=> ��
//				<1=> ��
//			<o4.3> ȥ��ʹ��
//				<i> P3_DBEN.DBEN[3]
//			<o5.3> �ж�ģʽ
//				<i> P3_IMD.IMD[3]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P3.4/T0/SDA
//			<i> Pin: 10
//			<o0.8..9> �๦�ܹܽ�ѡ��
//				<i> P3_MFP.P3_MFP[4], P3_MFP.P3_ALT[4]
//				<0=> P3.4
//				<1=> T0
//				<2=> SDA
//			<o0.20> ʩ���ش�������ʹ��
//				<i> P3_MFP.P3_TYPE[4]
//			<o1.8..9> �˿�ģʽ
//				<i> P3_PMD.PMD4
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.20> OFF ��������ͨ��ʹ��
//				<i> P3_OFFD.OFFD[4]
//			<o3.4> ���ֵ
//				<i> P3_DOUT.DOUT[4]
//				<0=> ��
//				<1=> ��
//			<o4.4> ȥ��ʹ��
//				<i> P3_DBEN.DBEN[4]
//			<o5.4> �ж�ģʽ
//				<i> P3_IMD.IMD[4]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P3.5/T1/SCL
//			<i> Pin: 11
//			<o0.10..11> �๦�ܹܽ�ѡ��
//				<i> P3_MFP.P3_MFP[5], P3_MFP.P3_ALT[5]
//				<0=> P3.5
//				<1=> T1
//				<2=> SCL
//			<o0.21> ʩ���ش�������ʹ��
//				<i> P3_MFP.P3_TYPE[5]
//			<o1.10..11> �˿�ģʽ
//				<i> P3_PMD.PMD5
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.21> OFF ��������ͨ��ʹ��
//				<i> P3_OFFD.OFFD[5]
//			<o3.5> ���ֵ
//				<i> P3_DOUT.DOUT[5]
//				<0=> ��
//				<1=> ��
//			<o4.5> ȥ��ʹ��
//				<i> P3_DBEN.DBEN[5]
//			<o5.5> �ж�ģʽ
//				<i> P3_IMD.IMD[5]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P3.6/WR/CKO
//			<i> Pin: 13
//			<o0.12..13> �๦�ܹܽ�ѡ��
//				<i> P3_MFP.P3_MFP[6], P3_MFP.P3_ALT[6]
//				<0=> P3.6
//				<1=> WR
//				<2=> CKO
//			<o0.22> ʩ���ش�������ʹ��
//				<i> P3_MFP.P3_TYPE[6]
//			<o1.12..13> �˿�ģʽ
//				<i> P3_PMD.PMD6
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.22> OFF ��������ͨ��ʹ��
//				<i> P3_OFFD.OFFD[6]
//			<o3.6> ���ֵ
//				<i> P3_DOUT.DOUT[6]
//				<0=> ��
//				<1=> ��
//			<o4.6> ȥ��ʹ��
//				<i> P3_DBEN.DBEN[6]
//			<o5.6> �ж�ģʽ
//				<i> P3_IMD.IMD[6]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P3.7/RD
//			<i> Pin: 14
//			<o0.14..15> �๦�ܹܽ�ѡ��
//				<i> P3_MFP.P3_MFP[7]
//				<0=> P3.7
//				<1=> RD
//			<o0.23> ʩ���ش�������ʹ��
//				<i> P3_MFP.P3_TYPE[7]
//			<o1.14..15> �˿�ģʽ
//				<i> P3_PMD.PMD7
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.23> OFF ��������ͨ��ʹ��
//				<i> P3_OFFD.OFFD[7]
//			<o3.7> ���ֵ
//				<i> P3_DOUT.DOUT[7]
//				<0=> ��
//				<1=> ��
//			<o4.7> ȥ��ʹ��
//				<i> P3_DBEN.DBEN[7]
//			<o5.7> �ж�ģʽ
//				<i> P3_IMD.IMD[7]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//	</h>
#define __P3MFP_VAL			0x00000005
#define __P3PMD_VAL			0x00005554
#define __P3OFFD_VAL		0x00000000
#define __P3DOUT_VAL		0x000000FF
#define __P3DBEN_VAL		0x00000000
#define __P3IMD_VAL			0x00000000

//	<h> P4 �ܽ�����
//		<h> P4.0/PWM0
//			<i> Pin: 24
//			<o0.0..1> �๦�ܹܽ�ѡ��
//				<i> P4_MFP.P4_MFP[0]
//				<0=> P4.0
//				<1=> PWM0
//			<o0.16> ʩ���ش�������ʹ��
//				<i> P4_MFP.P4_TYPE[0]
//			<o1.0..1> �˿�ģʽ
//				<i> P4_PMD.PMD0
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.16> OFF ��������ͨ��ʹ��
//				<i> P4_OFFD.OFFD[0]
//			<o3.0> ���ֵ
//				<i> P4_DOUT.DOUT[0]
//				<0=> ��
//				<1=> ��
//			<o4.0> ȥ��ʹ��
//				<i> P4_DBEN.DBEN[0]
//			<o5.0> �ж�ģʽ
//				<i> P4_IMD.IMD[0]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P4.1/PWM1
//			<i> Pin: 36
//			<o0.2..3> �๦�ܹܽ�ѡ��
//				<i> P4_MFP.P4_MFP[1]
//				<0=> P4.1
//				<1=> PWM1
//			<o0.17> ʩ���ش�������ʹ��
//				<i> P4_MFP.P4_TYPE[1]
//			<o1.2..3> �˿�ģʽ
//				<i> P4_PMD.PMD1
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.17> OFF ��������ͨ��ʹ��
//				<i> P4_OFFD.OFFD[1]
//			<o3.1> ���ֵ
//				<i> P4_DOUT.DOUT[1]
//				<0=> ��
//				<1=> ��
//			<o4.1> ȥ��ʹ��
//				<i> P4_DBEN.DBEN[1]
//			<o5.1> �ж�ģʽ
//				<i> P4_IMD.IMD[1]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P4.2/PWM2
//			<i> Pin: 48
//			<o0.4..5> �๦�ܹܽ�ѡ��
//				<i> P4_MFP.P4_MFP[2]
//				<0=> P4.2(STADC)
//				<1=> PWM2
//			<o0.18> ʩ���ش�������ʹ��
//				<i> P4_MFP.P4_TYPE[2]
//			<o1.4..5> �˿�ģʽ
//				<i> P4_PMD.PMD2
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.18> OFF ��������ͨ��ʹ��
//				<i> P4_OFFD.OFFD[2]
//			<o3.2> ���ֵ
//				<i> P4_DOUT.DOUT[2]
//				<0=> ��
//				<1=> ��
//			<o4.2> ȥ��ʹ��
//				<i> P4_DBEN.DBEN[2]
//			<o5.2> �ж�ģʽ
//				<i> P4_IMD.IMD[2]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P4.3/PWM3
//			<i> Pin: 12
//			<o0.6..7> �๦�ܹܽ�ѡ��
//				<i> P4_MFP.P4_MFP[3]
//				<0=> P4.3
//				<1=> PWM3
//			<o0.19> ʩ���ش�������ʹ��
//				<i> P4_MFP.P4_TYPE[3]
//			<o1.6..7> �˿�ģʽ
//				<i> P4_PMD.PMD3
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.19> OFF ��������ͨ��ʹ��
//				<i> P4_OFFD.OFFD[3]
//			<o3.3> ���ֵ
//				<i> P4_DOUT.DOUT[3]
//				<0=> ��
//				<1=> ��
//			<o4.3> ȥ��ʹ��
//				<i> P4_DBEN.DBEN[3]
//			<o5.3> �ж�ģʽ
//				<i> P4_IMD.IMD[3]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P4.4/CS
//			<i> Pin: 28
//			<o0.8..9> �๦�ܹܽ�ѡ��
//				<i> P4_MFP.P4_MFP[4]
//				<0=> P4.4
//				<1=> CS
//			<o0.20> ʩ���ش�������ʹ��
//				<i> P4_MFP.P4_TYPE[4]
//			<o1.8..9> �˿�ģʽ
//				<i> P4_PMD.PMD4
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.20> OFF ��������ͨ��ʹ��
//				<i> P4_OFFD.OFFD[4]
//			<o3.4> ���ֵ
//				<i> P4_DOUT.DOUT[4]
//				<0=> ��
//				<1=> ��
//			<o4.4> ȥ��ʹ��
//				<i> P4_DBEN.DBEN[4]
//			<o5.4> �ж�ģʽ
//				<i> P4_IMD.IMD[4]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P4.5/ALE
//			<i> Pin: 29
//			<o0.10..11> �๦�ܹܽ�ѡ��
//				<i> P4_MFP.P4_MFP[5]
//				<0=> P4.5
//				<1=> ALE
//			<o0.21> ʩ���ش�������ʹ��
//				<i> P4_MFP.P4_TYPE[5]
//			<o1.10..11> �˿�ģʽ
//				<i> P4_PMD.PMD5
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.21> OFF ��������ͨ��ʹ��
//				<i> P4_OFFD.OFFD[5]
//			<o3.5> ���ֵ
//				<i> P4_DOUT.DOUT[5]
//				<0=> ��
//				<1=> ��
//			<o4.5> ȥ��ʹ��
//				<i> P4_DBEN.DBEN[5]
//			<o5.5> �ж�ģʽ
//				<i> P4_IMD.IMD[5]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P4.6/ICE_CLK
//			<i> Pin: 30
//			<o0.12..13> �๦�ܹܽ�ѡ��
//				<i> P4_MFP.P4_MFP[6]
//				<0=> P4.6
//				<1=> ICE_CLK
//			<o0.22> ʩ���ش�������ʹ��
//				<i> P4_MFP.P4_TYPE[6]
//			<o1.12..13> �˿�ģʽ
//				<i> P4_PMD.PMD6
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.22> OFF ��������ͨ��ʹ��
//				<i> P4_OFFD.OFFD[6]
//			<o3.6> ���ֵ
//				<i> P4_DOUT.DOUT[6]
//				<0=> ��
//				<1=> ��
//			<o4.6> ȥ��ʹ��
//				<i> P4_DBEN.DBEN[6]
//			<o5.6> �ж�ģʽ
//				<i> P4_IMD.IMD[6]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//		<h> P4.7/ICE_DAT
//			<i> Pin: 31
//			<o0.14..15> �๦�ܹܽ�ѡ��
//				<i> P4_MFP.P4_MFP[7]
//				<0=> P4.7
//				<1=> ICE_DAT
//			<o0.23> ʩ���ش�������ʹ��
//				<i> P4_MFP.P4_TYPE[7]
//			<o1.14..15> �˿�ģʽ
//				<i> P4_PMD.PMD7
//				<0=> ����
//				<1=> ���
//				<2=> ��©
//				<3=> ׼˫��
//			<o2.23> OFF ��������ͨ��ʹ��
//				<i> P4_OFFD.OFFD[7]
//			<o3.7> ���ֵ
//				<i> P4_DOUT.DOUT[7]
//				<0=> ��
//				<1=> ��
//			<o4.7> ȥ��ʹ��
//				<i> P4_DBEN.DBEN[7]
//			<o5.7> �ж�ģʽ
//				<i> P4_IMD.IMD[7]
//				<0=> ����
//				<1=> ��ƽ
//		</h>
//	</h>
#define __P4MFP_VAL			0x00005000
#define __P4PMD_VAL			0x0000F555
#define __P4OFFD_VAL		0x00000000
#define __P4DOUT_VAL		0x000000FF
#define __P4DBEN_VAL		0x00000000
#define __P4IMD_VAL			0x00000000

//	<h> �ж�ȥ������
//		<o0.0..3> ȥ����������ѡ��
//			<i> DBNCECON.DBCLKSEL
//			<0=> ÿ 1 ȥ��ʱ�Ӳ����ж�����һ��
//			<1=> ÿ 2 ȥ��ʱ�Ӳ����ж�����һ��
//			<2=> ÿ 4 ȥ��ʱ�Ӳ����ж�����һ��
//			<3=> ÿ 8 ȥ��ʱ�Ӳ����ж�����һ��
//			<4=> ÿ 16 ȥ��ʱ�Ӳ����ж�����һ��
//			<5=> ÿ 32 ȥ��ʱ�Ӳ����ж�����һ��
//			<6=> ÿ 64 ȥ��ʱ�Ӳ����ж�����һ��
//			<7=> ÿ 128 ȥ��ʱ�Ӳ����ж�����һ��
//			<8=> ÿ 256 ȥ��ʱ�Ӳ����ж�����һ��
//			<9=> ÿ 512 ȥ��ʱ�Ӳ����ж�����һ��
//			<10=> ÿ 1024 ȥ��ʱ�Ӳ����ж�����һ��
//			<11=> ÿ 2048 ȥ��ʱ�Ӳ����ж�����һ��
//			<12=> ÿ 4096 ȥ��ʱ�Ӳ����ж�����һ��
//			<13=> ÿ 8192 ȥ��ʱ�Ӳ����ж�����һ��
//			<14=> ÿ 16384 ȥ��ʱ�Ӳ����ж�����һ��
//			<15=> ÿ 32768 ȥ��ʱ�Ӳ����ж�����һ��
//		<o0.4> ȥ��������ʱ��Դѡ��
//			<i> DBNCECON.DBCLKSRC
//			<0=> HCLK
//			<1=> �ڲ� 10KHz ʱ��
//		<o0.5> �ж�ʱ�� On ģʽ
//			<i> DBNCECON.ICLK_ON
//			<0=> ����ж� GPIOA/B/C/D/E[n] ����ֹ��������ж�ʱ��
//			<1=> ����ʹ���ж�ʱ��
//	</h>
#define __DBNCECON_VAL		0x00000036
//*** <<< end of configuration section >>> ***

#if (__PWRCON_VAL & 4) != 0
#define __OSC22M	22118400
#else
#define __OSC22M	0
#endif

#if (__PWRCON_VAL & 8) != 0
#define __OSC10K	10000
#else
#define __OSC10K	0
#endif

#if (__PLLCON_VAL & (1 << 18)) != 0		// OE
#define __PLL_FOUT	0
#else
#if (__PLLCON_VAL & (1 << 19)) == 0		// PLL_SRC
#define __PLL_FIN	__XTAL
#else
#define __PLL_FIN	__OSC22M
#endif
#if (__PLLCON_VAL & (1 << 17)) != 0		// BP
#define __PLL_FOUT	__PLL_FIN
#else
#define __PLL_FR		(__PLL_FIN / (((__PLLCON_VAL >> 9) & 0x1f) + 2))
#if __PLL_FR < 1600000 || __PLL_FR > 16000000
#error "the PLL constraints, 800 KHz < (FIN / (2 * NR)) < 8 MHz, must be met"
#else
#define __PLL_FCO		(__PLL_FR * ((__PLLCON_VAL & 0x1ff) + 2))
#if __PLL_FCO < 100000000 || __PLL_FCO > 200000000
#error "the PLL constraints, 100 MHz < (FIN / NR * NF) < 200 MHz, must be met"
#else
#define __PLL_FOUT	(__PLL_FCO / (((__PLLCON_VAL >> 14) & 3) + 1))
#endif
#endif
#endif
#endif

#if (__CLKSEL0_VAL & 7) == 0			// external 4~24 MHz crystal clock
#define __HCLK_FIN	__XTAL
#elif (__CLKSEL0_VAL & 7) == 2			// PLL
#define __HCLK_FIN	__PLL_FOUT
#elif (__CLKSEL0_VAL & 7) == 3			// internal 10 kHz oscillator
#define __HCLK_FIN	__OSC10K
#else									// internal 22.1184 MHz oscillator
#define __HCLK_FIN	__OSC22M
#endif

#define __HCLK	(__HCLK_FIN / ((__CLKDIV_VAL & 15) + 1))

#define __CLKVAL(VAL, LSB, WIDTH, SEL0, SEL1, SEL2, SEL3, SEL7, FDIV)		\
	(																		\
		((((VAL) >> (LSB)) & ((1 << (WIDTH)) - 1)) == 0 ? (SEL0) : (		\
		(((VAL) >> (LSB)) & ((1 << (WIDTH)) - 1)) == 1 ? (SEL1) : (			\
		(((VAL) >> (LSB)) & ((1 << (WIDTH)) - 1)) == 2 ? (SEL2) : (			\
		(((VAL) >> (LSB)) & ((1 << (WIDTH)) - 1)) == 3 ? (SEL3) : (SEL7)	\
		)))) / (FDIV)														\
	)

#define F_CPU		__HCLK
#define F_WDT		__CLKVAL(__CLKSEL1_VAL,  0, 2,      0,          0, __HCLK / 2048, __OSC10K,        0, 1)
#define F_ADC		__CLKVAL(__CLKSEL1_VAL,	 2, 2, __XTAL, __PLL_FOUT,             0, __OSC22M,        0, ((__CLKDIV_VAL > 16) & 0xff) + 1)
#define F_TIMER0	__CLKVAL(__CLKSEL1_VAL,  8, 3, __XTAL,          0,        __HCLK,        0, __OSC22M, 1)
#define F_TIMER1	__CLKVAL(__CLKSEL1_VAL, 12, 3, __XTAL,          0,        __HCLK,        0, __OSC22M, 1)
#define F_TIMER2	__CLKVAL(__CLKSEL1_VAL, 16, 3, __XTAL,          0,        __HCLK,        0, __OSC22M, 1)
#define F_TIMER3	__CLKVAL(__CLKSEL1_VAL, 20, 3, __XTAL,          0,        __HCLK,        0, __OSC22M, 1)
#define F_UART		__CLKVAL(__CLKSEL1_VAL, 24, 2, __XTAL, __PLL_FOUT,             0, __OSC22M,        0, ((__CLKDIV_VAL > 8) & 0xf) + 1)
#define F_PWM01		__CLKVAL(__CLKSEL1_VAL, 28, 2, __XTAL,          0,        __HCLK, __OSC22M,        0, 1)
#define F_PWM23		__CLKVAL(__CLKSEL1_VAL, 30, 2, __XTAL,          0,        __HCLK, __OSC22M,        0, 1)
#define F_FRQDIV	__CLKVAL(__CLKSEL2_VAL,  2, 2, __XTAL,          0,        __HCLK, __OSC22M,        0, 1 << ((0 - ((__FRQDIV_VAL >> 4) & 1)) & __FRQDIV_VAL & 0xf))
#define F_PWM45		__CLKVAL(__CLKSEL2_VAL,  4, 2, __XTAL,          0,        __HCLK, __OSC22M,        0, 1)
#define F_PWM67		__CLKVAL(__CLKSEL2_VAL,  6, 2, __XTAL,          0,        __HCLK, __OSC22M,        0, 1)

enum {
	FLASHSIZE = 64 * 1024,
	RAMSIZE = 4 * 1024,
	PDID = 0x10005A00
};

#endif	// __M0516BN_H

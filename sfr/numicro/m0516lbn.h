#ifndef __M0516LBN_H
#define __M0516LBN_H

//*** <<< Use Configuration Wizard in Context Menu >>> ***
//	<o0> 外部 4~24 MHz 晶振频率
//		<4000000-240000000>
#define __XTAL		0x00B71B00

//	<h> Config0 配置
//		<o0.1> 安全加密
//			<i> Config0.LOCK
//		<o0.7> 启动选择
//			<i> Config0.CBS
//			<1=> 从 LDROM 启动
//			<0=> 从 APROM 启动
//		<o0.20> 欠压复位使能
//			<i> Config0.CBORST
//		<o0.21..22> 欠压电压选择
//			<i> Config0.CBOV1-0
//			<3=> 2.2V
//			<2=> 2.7V
//			<1=> 3.8V
//			<0=> 4.5V
//		<o0.23> 欠压检测使能
//			<i> Config0.CBODEN
//		<o0.24..26> 复位后 CPU 时钟源选择
//			<i> Config0.CFOSC
//			<7=> 外部 4~24 MHz 晶振
//			<0=> 内部 22.1184 MHz 振荡器
//		<o0.28> XT1 时钟滤波使能
//			<i> Config0.CKF
//			<#+1>
//	</h>
#define __CONFIG0_VAL		~0x07F00000

//	<h> 时钟配置
#define __PWRCON_DEFAULT	(0x0000001C | ((~__CONFIG0_VAL >> 24) & 1))
#define __AHBCLK_DEFAULT	0x0000000D
#define __APBCLK_DEFAULT	0x00000001
#define __CLKSEL0_DEFAULT	(0x00000038 | ((__CONFIG0_VAL >> 24) & 7))
#define __CLKSEL1_DEFAULT	0xFFFFFFFF
#define __CLKDIV_DEFAULT	0x00000000
#define __CLKSEL2_DEFAULT	0x000000FF
#define __PLLCON_DEFAULT	0x0005C22E
#define __FRQDIV_DEFAULT	0x00000000
//		<o0> 外部 4~24 MHz 晶振使能
//			<i> PWRCON.XTL12M_EN
//			<2=> 由 Config0.CFOSC“复位后 CPU 时钟源选择”配置自动确定
//			<0=> 禁能
//			<1=> 使能
//		<o1.2> 内部 22.1184 MHz 振荡器使能
//			<i> PWRCON.OSC22M_EN
//		<o1.3> 内部 10 KHz 振荡器使能
//			<i> PWRCON.OSC10K_EN
//		<h> PLL 配置
//			<i> PLLCON
//			<o8.19> 时钟源
//				<i> PLLCON.PLL_SRC(FIN)
//				<0=> 外部 4~24 MHz 晶振
//				<1=> 内部 22.1184 MHz 振荡器
//			<o8.0..8> 反馈分频控制
//				<i> PLLCON.FB_DV(NF)
//				<2-513><#-2>
//			<o8.9..13> 输入分频控制
//				<i> PLLCON.IN_DV(NR)
//				<2-17><#-2>
//			<o8.14..15> 输出分频控制
//				<i> PLLCON.OUT_DV(NO)
//				<0=> 1
//				<1=> 2
//				<3=> 4
//			<o8.16> 掉电模式
//				<i> PLLCON.PD
//			<o8.17> 旁路
//				<i> PLLCON.BP
//			<o8.18> 输出使能
//				<i> PLLCON.OE
//				<0=> 使能
//				<1=> 禁能
//		</h>
//		<h> 系统时钟配置
//			<i> CLKSEL0, CLKDIV
//			<o4> HCLK 时钟源
//				<i> CLKSEL0.HCLK_S
//				<8=> 由 Config0.CFOSC“复位后 CPU 时钟源选择”配置自动确定
//				<0=> 外部 4~24 MHz 晶振
//				<2=> PLL 时钟
//				<3=> 内部 10 KHz 振荡器时钟
//				<7=> 内部 22.1184 MHz 振荡器
//			<o6.0..3> HCLK 时钟分频数
//				<i> CLKDIV.HCLK_N
//				<1-16><#-1>
//		</h>
//		<h> AHB 设备时钟配置
//			<i> AHBCLK
//			<o2.2> Flash ISP 控制器时钟使能
//				<i> AHBCLK.ISP_EN
//			<o2.3> 外总线接口控制器时钟使能
//				<i> AHBCLK.EBI_EN
//		</h>
//		<h> APB 设备时钟配置
//			<i> APBCLK, CLKSEL1, CLKSEL2, CLKDIV
//			<o5.0..1> Watchdog 时钟源选择
//				<i> CLKSEL1.WDT_S
//				<2=> HCLK / 2048
//				<3=> 内部 10 KHz 振荡器时钟
//			<o3.0> Watchdog 时钟使能
//				<i> APBCLK.WDT_EN
//			<o5.2..3> ADC 时钟源选择
//				<i> CLKSEL1.ADC_S
//				<0=> 外部 4~24 MHz 晶振时钟
//				<1=> PLL 时钟
//				<3=> 内部 22.1184 MHz 振荡器时钟
//			<o6.16..23>	ADC 时钟分频数
//				<i> CLKDIV.ADC_N
//				<1-256><#-1>
//			<o3.28> ADC 时钟使能
//				<i> APBCLK.ADC_EN
//			<o5.8..10> TIMER0 时钟源选择
//				<i> CLKSEL1.TMR0_S
//				<0=> 外部 4~24 MHz 晶振时钟
//				<2=> HCLK
//				<7=> 内部 22.1184 MHz 振荡器时钟
//			<o3.2> TIMER0 时钟使能
//				<i> APBCLK.TMR0_EN
//			<o5.12..14> TIMER1 时钟源选择
//				<i> CLKSEL1.TMR1_S
//				<0=> 外部 4~24 MHz 晶振时钟
//				<2=> HCLK
//				<7=> 内部 22.1184 MHz 振荡器时钟
//			<o3.3> TIMER1 时钟使能
//				<i> APBCLK.TMR1_EN
//			<o5.16..18> TIMER2 时钟源选择
//				<i> CLKSEL1.TMR2_S
//				<0=> 外部 4~24 MHz 晶振时钟
//				<2=> HCLK
//				<7=> 内部 22.1184 MHz 振荡器时钟
//			<o3.4> TIMER2 时钟使能
//				<i> APBCLK.TMR2_EN
//			<o5.20..22> TIMER3 时钟源选择
//				<i> CLKSEL1.TMR3_S
//				<0=> 外部 4~24 MHz 晶振时钟
//				<2=> HCLK
//				<7=> 内部 22.1184 MHz 振荡器时钟
//			<o3.5> TIMER3 时钟使能
//				<i> APBCLK.TMR3_EN
//			<o3.8> I2C 时钟使能
//				<i> APBCLK.I2C0_EN
//			<o3.12> SPI0 时钟使能
//				<i> APBCLK.SPI0_EN
//			<o3.13> SPI1 时钟使能
//				<i> APBCLK.SPI1_EN
//			<o5.24..25> UART 时钟源选择
//				<i> CLKSEL1.UART_S
//				<0=> 外部 4~24 MHz 晶振时钟
//				<1=> PLL 时钟
//				<3=> 内部 22.1184 MHz 振荡器时钟
//			<o6.8..11> UART 时钟分频数
//				<i> CLKDIV.UART_N
//				<1-16><#-1>
//			<o3.16>UART0 时钟使能
//				<i> APBCLK.UART0_EN
//			<o3.17>UART1 时钟使能
//				<i> APBCLK.UART1_EN
//			<o5.28..29> PWM0 和 PWM1 时钟源选择
//				<i> CLKSEL1.PWM01_S
//				<0=> 外部 4~24 MHz 晶振时钟
//				<2=> HCLK
//				<3=> 内部 22.1184 MHz 振荡器时钟
//			<o3.20> PWM0 和 PWM1 时钟使能
//				<i> APBCLK.PWM01_EN
//			<o5.30..31> PWM2 和 PWM3 时钟源选择
//				<i> CLKSEL1.PWM23_S
//				<0=> 外部 4~24 MHz 晶振时钟
//				<2=> HCLK
//				<3=> 内部 22.1184 MHz 振荡器时钟
//			<o3.21> PWM2 和 PWM3 时钟使能
//				<i> APBCLK.PWM23_EN
//			<o7.2..3> 分频器时钟源选择
//				<i> CLKSEL2.FRQDIV_S
//				<0=> 外部 4~24 MHz 晶振时钟
//				<2=> HCLK
//				<3=> 内部 22.1184 MHz 振荡器时钟
//			<o3.6> 分频器时钟使能
//				<i> APBCLK.FDIV_EN
//			<o9.0..3> 分频器时钟分频数
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
//			<o9.4> 分频器分频使能
//				<i> FRQDIV.DIVIDER_EN
//			<o7.4..5> PWM4 和 PWM5 时钟源选择
//				<i> CLKSEL2.PWM45_S
//				<0=> 外部 4~24 MHz 晶振时钟
//				<2=> HCLK
//				<3=> 内部 22.1184 MHz 振荡器时钟
//			<o3.22> PWM4 和 PWM5 时钟使能
//				<i> APBCLK.PWM45_EN
//			<o7.6..7> PWM6 和 PWM7 时钟源选择
//				<i> CLKSEL2.PWM67_S
//				<0=> 外部 4~24 MHz 晶振时钟
//				<2=> HCLK
//				<3=> 内部 22.1184 MHz 振荡器时钟
//			<o3.23> PWM6 和 PWM7 时钟使能
//				<i> APBCLK.PWM67_EN
//		</h>
//	</h>
//	<h> 掉电及唤醒配置
//		<o1.4> 唤醒延时计数器使能
//			<i> PWRCON.PD_WU_DLY
//		<o1.5> 唤醒中断使能
//			<i> PWRCON.PD_WU_INT_EN
//		<o1.7> 系统掉电使能
//			<i> PWRCON.PWR_DOWN_EN
//		<o1.8> 等待 CPU 执行 WFI 指令
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

//	<h> 欠压检测配置
#define __BODCR_DEFAULT		(0x00000080 | ((~__CONFIG0_VAL >> 23) & 1) | ((__CONFIG0_VAL >> 20) & 6) | ((~__CONFIG0_VAL >> 17) & 8))
//		<o0.5> 使能低功耗模式
//			<i> BODCR.BOD_LPM
//		<o0.7> 低压复位使能
//			<i> BODCR.LVR_EN
//	</h>
#define __BODCR_VAL			(0x00000080 | ((~__CONFIG0_VAL >> 23) & 1) | ((__CONFIG0_VAL >> 20) & 6) | ((~__CONFIG0_VAL >> 17) & 8))

//	<o0.0> 禁用上电复位
#define __PORCR_VAL			0

//	<o0.0> 初始化后锁定寄存器写保护锁
#define __LOCK_SETUP		0

//	<h> P0 管脚配置
//		<h> P0.0/AD0/CTS1
//			<i> Pin: 40
//			<o0.0..1> 多功能管脚选择
//				<i> P0_MFP.P0_MFP[0], P0_MFP.P0_ALT[0]
//				<0=> P0.0
//				<1=> AD0
//				<2=> CTS1
//			<o0.16> 施密特触发输入使能
//				<i> P0_MFP.P0_TYPE[0]
//			<o1.0..1> 端口模式
//				<i> P0_PMD.PMD0
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.16> OFF 数字输入通道使能
//				<i> P0_OFFD.OFFD[0]
//			<o3.0> 输出值
//				<i> P0_DOUT.DOUT[0]
//				<0=> 低
//				<1=> 高
//			<o4.0> 去抖使能
//				<i> P0_DBEN.DBEN[0]
//			<o5.0> 中断模式
//				<i> P0_IMD.IMD[0]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P0.1/AD1/RTS1
//			<i> Pin: 39
//			<o0.2..3> 多功能管脚选择
//				<i> P0_MFP.P0_MFP[1], P0_MFP.P0_ALT[1]
//				<0=> P0.1
//				<1=> AD1
//				<2=> RTS1
//			<o0.17> 施密特触发输入使能
//				<i> P0_MFP.P0_TYPE[1]
//			<o1.2..3> 端口模式
//				<i> P0_PMD.PMD1
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.17> OFF 数字输入通道使能
//				<i> P0_OFFD.OFFD[1]
//			<o3.1> 输出值
//				<i> P0_DOUT.DOUT[1]
//				<0=> 低
//				<1=> 高
//			<o4.1> 去抖使能
//				<i> P0_DBEN.DBEN[1]
//			<o5.1> 中断模式
//				<i> P0_IMD.IMD[1]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P0.2/AD2/CTS0
//			<i> Pin: 38
//			<o0.4..5> 多功能管脚选择
//				<i> P0_MFP.P0_MFP[2], P0_MFP.P0_ALT[2]
//				<0=> P0.2
//				<1=> AD2
//				<2=> CTS0
//			<o0.18> 施密特触发输入使能
//				<i> P0_MFP.P0_TYPE[2]
//			<o1.4..5> 端口模式
//				<i> P0_PMD.PMD2
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.18> OFF 数字输入通道使能
//				<i> P0_OFFD.OFFD[2]
//			<o3.2> 输出值
//				<i> P0_DOUT.DOUT[2]
//				<0=> 低
//				<1=> 高
//			<o4.2> 去抖使能
//				<i> P0_DBEN.DBEN[2]
//			<o5.2> 中断模式
//				<i> P0_IMD.IMD[2]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P0.3/AD3/RTS0
//			<i> Pin: 37
//			<o0.6..7> 多功能管脚选择
//				<i> P0_MFP.P0_MFP[3], P0_MFP.P0_ALT[3]
//				<0=> P0.3
//				<1=> AD3
//				<2=> RTS0
//			<o0.19> 施密特触发输入使能
//				<i> P0_MFP.P0_TYPE[3]
//			<o1.6..7> 端口模式
//				<i> P0_PMD.PMD3
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.19> OFF 数字输入通道使能
//				<i> P0_OFFD.OFFD[3]
//			<o3.3> 输出值
//				<i> P0_DOUT.DOUT[3]
//				<0=> 低
//				<1=> 高
//			<o4.3> 去抖使能
//				<i> P0_DBEN.DBEN[3]
//			<o5.3> 中断模式
//				<i> P0_IMD.IMD[3]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P0.4/AD4/SPISS1
//			<i> Pin: 35
//			<o0.8..9> 多功能管脚选择
//				<i> P0_MFP.P0_MFP[4], P0_MFP.P0_ALT[4]
//				<0=> P0.4
//				<1=> AD4
//				<2=> SPISS1
//			<o0.20> 施密特触发输入使能
//				<i> P0_MFP.P0_TYPE[4]
//			<o1.8..9> 端口模式
//				<i> P0_PMD.PMD4
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.20> OFF 数字输入通道使能
//				<i> P0_OFFD.OFFD[4]
//			<o3.4> 输出值
//				<i> P0_DOUT.DOUT[4]
//				<0=> 低
//				<1=> 高
//			<o4.4> 去抖使能
//				<i> P0_DBEN.DBEN[4]
//			<o5.4> 中断模式
//				<i> P0_IMD.IMD[4]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P0.5/AD5/MOSI1
//			<i> Pin: 34
//			<o0.10..11> 多功能管脚选择
//				<i> P0_MFP.P0_MFP[5], P0_MFP.P0_ALT[5]
//				<0=> P0.5
//				<1=> AD5
//				<2=> MOSI1
//			<o0.21> 施密特触发输入使能
//				<i> P0_MFP.P0_TYPE[5]
//			<o1.10..11> 端口模式
//				<i> P0_PMD.PMD5
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.21> OFF 数字输入通道使能
//				<i> P0_OFFD.OFFD[5]
//			<o3.5> 输出值
//				<i> P0_DOUT.DOUT[5]
//				<0=> 低
//				<1=> 高
//			<o4.5> 去抖使能
//				<i> P0_DBEN.DBEN[5]
//			<o5.5> 中断模式
//				<i> P0_IMD.IMD[5]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P0.6/AD6/MISO1
//			<i> Pin: 33
//			<o0.12..13> 多功能管脚选择
//				<i> P0_MFP.P0_MFP[6], P0_MFP.P0_ALT[6]
//				<0=> P0.6
//				<1=> AD6
//				<2=> MISO1
//			<o0.22> 施密特触发输入使能
//				<i> P0_MFP.P0_TYPE[6]
//			<o1.12..13> 端口模式
//				<i> P0_PMD.PMD6
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.22> OFF 数字输入通道使能
//				<i> P0_OFFD.OFFD[6]
//			<o3.6> 输出值
//				<i> P0_DOUT.DOUT[6]
//				<0=> 低
//				<1=> 高
//			<o4.6> 去抖使能
//				<i> P0_DBEN.DBEN[6]
//			<o5.6> 中断模式
//				<i> P0_IMD.IMD[6]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P0.7/AD7/SPICLK1
//			<i> Pin: 32
//			<o0.14..15> 多功能管脚选择
//				<i> P0_MFP.P0_MFP[7], P0_MFP.P0_ALT[7]
//				<0=> P0.7
//				<1=> AD7
//				<2=> SPICLK1
//			<o0.23> 施密特触发输入使能
//				<i> P0_MFP.P0_TYPE[7]
//			<o1.14..15> 端口模式
//				<i> P0_PMD.PMD7
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.23> OFF 数字输入通道使能
//				<i> P0_OFFD.OFFD[7]
//			<o3.7> 输出值
//				<i> P0_DOUT.DOUT[7]
//				<0=> 低
//				<1=> 高
//			<o4.7> 去抖使能
//				<i> P0_DBEN.DBEN[7]
//			<o5.7> 中断模式
//				<i> P0_IMD.IMD[7]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//	</h>
#define __P0MFP_VAL			0x00FF0000
#define __P0PMD_VAL			0x00000000
#define __P0OFFD_VAL		0x00000000
#define __P0DOUT_VAL		0x000000FF
#define __P0DBEN_VAL		0x000000FF
#define __P0IMD_VAL			0x00000000

//	<h> P1 管脚配置
//		<h> P1.0/AIN0/T2
//			<i> Pin: 43
//			<o0.0..1> 多功能管脚选择
//				<i> P1_MFP.P1_MFP[0], P1_MFP.P1_ALT[0]
//				<0=> P1.0
//				<1=> AIN0
//				<2=> T2
//			<o0.16> 施密特触发输入使能
//				<i> P1_MFP.P1_TYPE[0]
//			<o1.0..1> 端口模式
//				<i> P1_PMD.PMD0
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.16> OFF 数字输入通道使能
//				<i> P1_OFFD.OFFD[0]
//			<o3.0> 输出值
//				<i> P1_DOUT.DOUT[0]
//				<0=> 低
//				<1=> 高
//			<o4.0> 去抖使能
//				<i> P1_DBEN.DBEN[0]
//			<o5.0> 中断模式
//				<i> P1_IMD.IMD[0]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P1.1/AIN1/T3
//			<i> Pin: 44
//			<o0.2..3> 多功能管脚选择
//				<i> P1_MFP.P1_MFP[1], P1_MFP.P1_ALT[1]
//				<0=> P1.1
//				<1=> AIN1
//				<2=> T3
//			<o0.17> 施密特触发输入使能
//				<i> P1_MFP.P1_TYPE[1]
//			<o1.2..3> 端口模式
//				<i> P1_PMD.PMD1
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.17> OFF 数字输入通道使能
//				<i> P1_OFFD.OFFD[1]
//			<o3.1> 输出值
//				<i> P1_DOUT.DOUT[1]
//				<0=> 低
//				<1=> 高
//			<o4.1> 去抖使能
//				<i> P1_DBEN.DBEN[1]
//			<o5.1> 中断模式
//				<i> P1_IMD.IMD[1]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P1.2/AIN2/RXD1
//			<i> Pin: 45
//			<o0.4..5> 多功能管脚选择
//				<i> P1_MFP.P1_MFP[2], P1_MFP.P1_ALT[2]
//				<0=> P1.2
//				<1=> AIN2
//				<2=> RXD1
//			<o0.18> 施密特触发输入使能
//				<i> P1_MFP.P1_TYPE[2]
//			<o1.4..5> 端口模式
//				<i> P1_PMD.PMD2
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.18> OFF 数字输入通道使能
//				<i> P1_OFFD.OFFD[2]
//			<o3.2> 输出值
//				<i> P1_DOUT.DOUT[2]
//				<0=> 低
//				<1=> 高
//			<o4.2> 去抖使能
//				<i> P1_DBEN.DBEN[2]
//			<o5.2> 中断模式
//				<i> P1_IMD.IMD[2]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P1.3/AIN3/TXD1
//			<i> Pin: 46
//			<o0.6..7> 多功能管脚选择
//				<i> P1_MFP.P1_MFP[3], P1_MFP.P1_ALT[3]
//				<0=> P1.3
//				<1=> AIN3
//				<2=> RTS0
//			<o0.19> 施密特触发输入使能
//				<i> P1_MFP.P1_TYPE[3]
//			<o1.6..7> 端口模式
//				<i> P1_PMD.PMD3
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.19> OFF 数字输入通道使能
//				<i> P1_OFFD.OFFD[3]
//			<o3.3> 输出值
//				<i> P1_DOUT.DOUT[3]
//				<0=> 低
//				<1=> 高
//			<o4.3> 去抖使能
//				<i> P1_DBEN.DBEN[3]
//			<o5.3> 中断模式
//				<i> P1_IMD.IMD[3]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P1.4/AIN4/SPISS0
//			<i> Pin: 47
//			<o0.8..9> 多功能管脚选择
//				<i> P1_MFP.P1_MFP[4], P1_MFP.P1_ALT[4]
//				<0=> P1.4
//				<1=> AIN4
//				<2=> SPISS0
//			<o0.20> 施密特触发输入使能
//				<i> P1_MFP.P1_TYPE[4]
//			<o1.8..9> 端口模式
//				<i> P1_PMD.PMD4
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.20> OFF 数字输入通道使能
//				<i> P1_OFFD.OFFD[4]
//			<o3.4> 输出值
//				<i> P1_DOUT.DOUT[4]
//				<0=> 低
//				<1=> 高
//			<o4.4> 去抖使能
//				<i> P1_DBEN.DBEN[4]
//			<o5.4> 中断模式
//				<i> P1_IMD.IMD[4]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P1.5/AIN5/MOSI0
//			<i> Pin: 1
//			<o0.10..11> 多功能管脚选择
//				<i> P1_MFP.P1_MFP[5], P1_MFP.P1_ALT[5]
//				<0=> P1.5
//				<1=> AIN5
//				<2=> MOSI0
//			<o0.21> 施密特触发输入使能
//				<i> P1_MFP.P1_TYPE[5]
//			<o1.10..11> 端口模式
//				<i> P1_PMD.PMD5
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.21> OFF 数字输入通道使能
//				<i> P1_OFFD.OFFD[5]
//			<o3.5> 输出值
//				<i> P1_DOUT.DOUT[5]
//				<0=> 低
//				<1=> 高
//			<o4.5> 去抖使能
//				<i> P1_DBEN.DBEN[5]
//			<o5.5> 中断模式
//				<i> P1_IMD.IMD[5]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P1.6/AIN6/MISO0
//			<i> Pin: 2
//			<o0.12..13> 多功能管脚选择
//				<i> P1_MFP.P1_MFP[6], P1_MFP.P1_ALT[6]
//				<0=> P1.6
//				<1=> AIN6
//				<2=> MISO0
//			<o0.22> 施密特触发输入使能
//				<i> P1_MFP.P1_TYPE[6]
//			<o1.12..13> 端口模式
//				<i> P1_PMD.PMD6
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.22> OFF 数字输入通道使能
//				<i> P1_OFFD.OFFD[6]
//			<o3.6> 输出值
//				<i> P1_DOUT.DOUT[6]
//				<0=> 低
//				<1=> 高
//			<o4.6> 去抖使能
//				<i> P1_DBEN.DBEN[6]
//			<o5.6> 中断模式
//				<i> P1_IMD.IMD[6]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P1.7/AIN7/SPICLK0
//			<i> Pin: 3
//			<o0.14..15> 多功能管脚选择
//				<i> P1_MFP.P1_MFP[7], P1_MFP.P1_ALT[7]
//				<0=> P1.7
//				<1=> AIN7
//				<2=> SPICLK0
//			<o0.23> 施密特触发输入使能
//				<i> P1_MFP.P1_TYPE[7]
//			<o1.14..15> 端口模式
//				<i> P1_PMD.PMD7
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.23> OFF 数字输入通道使能
//				<i> P1_OFFD.OFFD[7]
//			<o3.7> 输出值
//				<i> P1_DOUT.DOUT[7]
//				<0=> 低
//				<1=> 高
//			<o4.7> 去抖使能
//				<i> P1_DBEN.DBEN[7]
//			<o5.7> 中断模式
//				<i> P1_IMD.IMD[7]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//	</h>
#define __P1MFP_VAL			0x00005555
#define __P1PMD_VAL			0x0000FFFF
#define __P1OFFD_VAL		0x00000000
#define __P1DOUT_VAL		0x000000FF
#define __P1DBEN_VAL		0x00000000
#define __P1IMD_VAL			0x00000000

//	<h> P2 管脚配置
//		<h> P2.0/AD8/PWM0
//			<i> Pin: 19
//			<o0.0..1> 多功能管脚选择
//				<i> P2_MFP.P2_MFP[0], P2_MFP.P2_ALT[0]
//				<0=> P2.0
//				<1=> AD8
//				<2=> PWM0
//			<o0.16> 施密特触发输入使能
//				<i> P2_MFP.P2_TYPE[0]
//			<o1.0..1> 端口模式
//				<i> P2_PMD.PMD0
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.16> OFF 数字输入通道使能
//				<i> P2_OFFD.OFFD[0]
//			<o3.0> 输出值
//				<i> P2_DOUT.DOUT[0]
//				<0=> 低
//				<1=> 高
//			<o4.0> 去抖使能
//				<i> P2_DBEN.DBEN[0]
//			<o5.0> 中断模式
//				<i> P2_IMD.IMD[0]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P2.1/AD9/PWM1
//			<i> Pin: 20
//			<o0.2..3> 多功能管脚选择
//				<i> P2_MFP.P2_MFP[1], P2_MFP.P2_ALT[1]
//				<0=> P2.1
//				<1=> AD9
//				<2=> PWM1
//			<o0.17> 施密特触发输入使能
//				<i> P2_MFP.P2_TYPE[1]
//			<o1.2..3> 端口模式
//				<i> P2_PMD.PMD1
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.17> OFF 数字输入通道使能
//				<i> P2_OFFD.OFFD[1]
//			<o3.1> 输出值
//				<i> P2_DOUT.DOUT[1]
//				<0=> 低
//				<1=> 高
//			<o4.1> 去抖使能
//				<i> P2_DBEN.DBEN[1]
//			<o5.1> 中断模式
//				<i> P2_IMD.IMD[1]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P2.2/AD10/PWM2
//			<i> Pin: 21
//			<o0.4..5> 多功能管脚选择
//				<i> P2_MFP.P2_MFP[2], P2_MFP.P2_ALT[2]
//				<0=> P2.2
//				<1=> AD10
//				<2=> PWM2
//			<o0.18> 施密特触发输入使能
//				<i> P2_MFP.P2_TYPE[2]
//			<o1.4..5> 端口模式
//				<i> P2_PMD.PMD2
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.18> OFF 数字输入通道使能
//				<i> P2_OFFD.OFFD[2]
//			<o3.2> 输出值
//				<i> P2_DOUT.DOUT[2]
//				<0=> 低
//				<1=> 高
//			<o4.2> 去抖使能
//				<i> P2_DBEN.DBEN[2]
//			<o5.2> 中断模式
//				<i> P2_IMD.IMD[2]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P2.3/AD11/PWM3
//			<i> Pin: 22
//			<o0.6..7> 多功能管脚选择
//				<i> P2_MFP.P2_MFP[3], P2_MFP.P2_ALT[3]
//				<0=> P2.3
//				<1=> AD11
//				<2=> PWM3
//			<o0.19> 施密特触发输入使能
//				<i> P2_MFP.P2_TYPE[3]
//			<o1.6..7> 端口模式
//				<i> P2_PMD.PMD3
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.19> OFF 数字输入通道使能
//				<i> P2_OFFD.OFFD[3]
//			<o3.3> 输出值
//				<i> P2_DOUT.DOUT[3]
//				<0=> 低
//				<1=> 高
//			<o4.3> 去抖使能
//				<i> P2_DBEN.DBEN[3]
//			<o5.3> 中断模式
//				<i> P2_IMD.IMD[3]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P2.4/AD12/PWM4
//			<i> Pin: 23
//			<o0.8..9> 多功能管脚选择
//				<i> P2_MFP.P2_MFP[4], P2_MFP.P2_ALT[4]
//				<0=> P2.4
//				<1=> AD12
//				<2=> PWM4
//			<o0.20> 施密特触发输入使能
//				<i> P2_MFP.P2_TYPE[4]
//			<o1.8..9> 端口模式
//				<i> P2_PMD.PMD4
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.20> OFF 数字输入通道使能
//				<i> P2_OFFD.OFFD[4]
//			<o3.4> 输出值
//				<i> P2_DOUT.DOUT[4]
//				<0=> 低
//				<1=> 高
//			<o4.4> 去抖使能
//				<i> P2_DBEN.DBEN[4]
//			<o5.4> 中断模式
//				<i> P2_IMD.IMD[4]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P2.5/AD13/PWM5
//			<i> Pin: 25
//			<o0.10..11> 多功能管脚选择
//				<i> P2_MFP.P2_MFP[5], P2_MFP.P2_ALT[5]
//				<0=> P2.5
//				<1=> AD13
//				<2=> PWM5
//			<o0.21> 施密特触发输入使能
//				<i> P2_MFP.P2_TYPE[5]
//			<o1.10..11> 端口模式
//				<i> P2_PMD.PMD5
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.21> OFF 数字输入通道使能
//				<i> P2_OFFD.OFFD[5]
//			<o3.5> 输出值
//				<i> P2_DOUT.DOUT[5]
//				<0=> 低
//				<1=> 高
//			<o4.5> 去抖使能
//				<i> P2_DBEN.DBEN[5]
//			<o5.5> 中断模式
//				<i> P2_IMD.IMD[5]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P2.6/AD14/PWM6
//			<i> Pin: 26
//			<o0.12..13> 多功能管脚选择
//				<i> P2_MFP.P2_MFP[6], P2_MFP.P2_ALT[6]
//				<0=> P2.6
//				<1=> AD14
//				<2=> PWM6
//			<o0.22> 施密特触发输入使能
//				<i> P2_MFP.P2_TYPE[6]
//			<o1.12..13> 端口模式
//				<i> P2_PMD.PMD6
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.22> OFF 数字输入通道使能
//				<i> P2_OFFD.OFFD[6]
//			<o3.6> 输出值
//				<i> P2_DOUT.DOUT[6]
//				<0=> 低
//				<1=> 高
//			<o4.6> 去抖使能
//				<i> P2_DBEN.DBEN[6]
//			<o5.6> 中断模式
//				<i> P2_IMD.IMD[6]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P2.7/AD15/PWM7
//			<i> Pin: 27
//			<o0.14..15> 多功能管脚选择
//				<i> P2_MFP.P2_MFP[7], P2_MFP.P2_ALT[7]
//				<0=> P2.7
//				<1=> AD15
//				<2=> PWM7
//			<o0.23> 施密特触发输入使能
//				<i> P2_MFP.P2_TYPE[7]
//			<o1.14..15> 端口模式
//				<i> P2_PMD.PMD7
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.23> OFF 数字输入通道使能
//				<i> P2_OFFD.OFFD[7]
//			<o3.7> 输出值
//				<i> P2_DOUT.DOUT[7]
//				<0=> 低
//				<1=> 高
//			<o4.7> 去抖使能
//				<i> P2_DBEN.DBEN[7]
//			<o5.7> 中断模式
//				<i> P2_IMD.IMD[7]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//	</h>
#define __P2MFP_VAL			0x00000000
#define __P2PMD_VAL			0x00005555
#define __P2OFFD_VAL		0x00000000
#define __P2DOUT_VAL		0x00000000
#define __P2DBEN_VAL		0x00000000
#define __P2IMD_VAL			0x00000000

//	<h> P3 管脚配置
//		<h> P3.0/RXD0
//			<i> Pin: 5
//			<o0.0..1> 多功能管脚选择
//				<i> P3_MFP.P3_MFP[0]
//				<0=> P3.0
//				<1=> RXD0
//			<o0.16> 施密特触发输入使能
//				<i> P3_MFP.P3_TYPE[0]
//			<o1.0..1> 端口模式
//				<i> P3_PMD.PMD0
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.16> OFF 数字输入通道使能
//				<i> P3_OFFD.OFFD[0]
//			<o3.0> 输出值
//				<i> P3_DOUT.DOUT[0]
//				<0=> 低
//				<1=> 高
//			<o4.0> 去抖使能
//				<i> P3_DBEN.DBEN[0]
//			<o5.0> 中断模式
//				<i> P3_IMD.IMD[0]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P3.1/TXD0
//			<i> Pin: 7
//			<o0.2..3> 多功能管脚选择
//				<i> P3_MFP.P3_MFP[1]
//				<0=> P3.1
//				<1=> TXD0
//			<o0.17> 施密特触发输入使能
//				<i> P3_MFP.P3_TYPE[1]
//			<o1.2..3> 端口模式
//				<i> P3_PMD.PMD1
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.17> OFF 数字输入通道使能
//				<i> P3_OFFD.OFFD[1]
//			<o3.1> 输出值
//				<i> P3_DOUT.DOUT[1]
//				<0=> 低
//				<1=> 高
//			<o4.1> 去抖使能
//				<i> P3_DBEN.DBEN[1]
//			<o5.1> 中断模式
//				<i> P3_IMD.IMD[1]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P3.2(STADC)/INT0
//			<i> Pin: 8
//			<o0.4..5> 多功能管脚选择
//				<i> P3_MFP.P3_MFP[2]
//				<0=> P3.2(STADC)
//				<1=> INT0
//			<o0.18> 施密特触发输入使能
//				<i> P3_MFP.P3_TYPE[2]
//			<o1.4..5> 端口模式
//				<i> P3_PMD.PMD2
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.18> OFF 数字输入通道使能
//				<i> P3_OFFD.OFFD[2]
//			<o3.2> 输出值
//				<i> P3_DOUT.DOUT[2]
//				<0=> 低
//				<1=> 高
//			<o4.2> 去抖使能
//				<i> P3_DBEN.DBEN[2]
//			<o5.2> 中断模式
//				<i> P3_IMD.IMD[2]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P3.3/INT1/MCLK
//			<i> Pin: 9
//			<o0.6..7> 多功能管脚选择
//				<i> P3_MFP.P3_MFP[3], P3_MFP.P3_ALT[3]
//				<0=> P3.3
//				<1=> INT1
//				<2=> MCLK
//			<o0.19> 施密特触发输入使能
//				<i> P3_MFP.P3_TYPE[3]
//			<o1.6..7> 端口模式
//				<i> P3_PMD.PMD3
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.19> OFF 数字输入通道使能
//				<i> P3_OFFD.OFFD[3]
//			<o3.3> 输出值
//				<i> P3_DOUT.DOUT[3]
//				<0=> 低
//				<1=> 高
//			<o4.3> 去抖使能
//				<i> P3_DBEN.DBEN[3]
//			<o5.3> 中断模式
//				<i> P3_IMD.IMD[3]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P3.4/T0/SDA
//			<i> Pin: 10
//			<o0.8..9> 多功能管脚选择
//				<i> P3_MFP.P3_MFP[4], P3_MFP.P3_ALT[4]
//				<0=> P3.4
//				<1=> T0
//				<2=> SDA
//			<o0.20> 施密特触发输入使能
//				<i> P3_MFP.P3_TYPE[4]
//			<o1.8..9> 端口模式
//				<i> P3_PMD.PMD4
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.20> OFF 数字输入通道使能
//				<i> P3_OFFD.OFFD[4]
//			<o3.4> 输出值
//				<i> P3_DOUT.DOUT[4]
//				<0=> 低
//				<1=> 高
//			<o4.4> 去抖使能
//				<i> P3_DBEN.DBEN[4]
//			<o5.4> 中断模式
//				<i> P3_IMD.IMD[4]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P3.5/T1/SCL
//			<i> Pin: 11
//			<o0.10..11> 多功能管脚选择
//				<i> P3_MFP.P3_MFP[5], P3_MFP.P3_ALT[5]
//				<0=> P3.5
//				<1=> T1
//				<2=> SCL
//			<o0.21> 施密特触发输入使能
//				<i> P3_MFP.P3_TYPE[5]
//			<o1.10..11> 端口模式
//				<i> P3_PMD.PMD5
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.21> OFF 数字输入通道使能
//				<i> P3_OFFD.OFFD[5]
//			<o3.5> 输出值
//				<i> P3_DOUT.DOUT[5]
//				<0=> 低
//				<1=> 高
//			<o4.5> 去抖使能
//				<i> P3_DBEN.DBEN[5]
//			<o5.5> 中断模式
//				<i> P3_IMD.IMD[5]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P3.6/WR/CKO
//			<i> Pin: 13
//			<o0.12..13> 多功能管脚选择
//				<i> P3_MFP.P3_MFP[6], P3_MFP.P3_ALT[6]
//				<0=> P3.6
//				<1=> WR
//				<2=> CKO
//			<o0.22> 施密特触发输入使能
//				<i> P3_MFP.P3_TYPE[6]
//			<o1.12..13> 端口模式
//				<i> P3_PMD.PMD6
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.22> OFF 数字输入通道使能
//				<i> P3_OFFD.OFFD[6]
//			<o3.6> 输出值
//				<i> P3_DOUT.DOUT[6]
//				<0=> 低
//				<1=> 高
//			<o4.6> 去抖使能
//				<i> P3_DBEN.DBEN[6]
//			<o5.6> 中断模式
//				<i> P3_IMD.IMD[6]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P3.7/RD
//			<i> Pin: 14
//			<o0.14..15> 多功能管脚选择
//				<i> P3_MFP.P3_MFP[7]
//				<0=> P3.7
//				<1=> RD
//			<o0.23> 施密特触发输入使能
//				<i> P3_MFP.P3_TYPE[7]
//			<o1.14..15> 端口模式
//				<i> P3_PMD.PMD7
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.23> OFF 数字输入通道使能
//				<i> P3_OFFD.OFFD[7]
//			<o3.7> 输出值
//				<i> P3_DOUT.DOUT[7]
//				<0=> 低
//				<1=> 高
//			<o4.7> 去抖使能
//				<i> P3_DBEN.DBEN[7]
//			<o5.7> 中断模式
//				<i> P3_IMD.IMD[7]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//	</h>
#define __P3MFP_VAL			0x00000005
#define __P3PMD_VAL			0x00005554
#define __P3OFFD_VAL		0x00000000
#define __P3DOUT_VAL		0x000000FF
#define __P3DBEN_VAL		0x00000000
#define __P3IMD_VAL			0x00000000

//	<h> P4 管脚配置
//		<h> P4.0/PWM0
//			<i> Pin: 24
//			<o0.0..1> 多功能管脚选择
//				<i> P4_MFP.P4_MFP[0]
//				<0=> P4.0
//				<1=> PWM0
//			<o0.16> 施密特触发输入使能
//				<i> P4_MFP.P4_TYPE[0]
//			<o1.0..1> 端口模式
//				<i> P4_PMD.PMD0
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.16> OFF 数字输入通道使能
//				<i> P4_OFFD.OFFD[0]
//			<o3.0> 输出值
//				<i> P4_DOUT.DOUT[0]
//				<0=> 低
//				<1=> 高
//			<o4.0> 去抖使能
//				<i> P4_DBEN.DBEN[0]
//			<o5.0> 中断模式
//				<i> P4_IMD.IMD[0]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P4.1/PWM1
//			<i> Pin: 36
//			<o0.2..3> 多功能管脚选择
//				<i> P4_MFP.P4_MFP[1]
//				<0=> P4.1
//				<1=> PWM1
//			<o0.17> 施密特触发输入使能
//				<i> P4_MFP.P4_TYPE[1]
//			<o1.2..3> 端口模式
//				<i> P4_PMD.PMD1
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.17> OFF 数字输入通道使能
//				<i> P4_OFFD.OFFD[1]
//			<o3.1> 输出值
//				<i> P4_DOUT.DOUT[1]
//				<0=> 低
//				<1=> 高
//			<o4.1> 去抖使能
//				<i> P4_DBEN.DBEN[1]
//			<o5.1> 中断模式
//				<i> P4_IMD.IMD[1]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P4.2/PWM2
//			<i> Pin: 48
//			<o0.4..5> 多功能管脚选择
//				<i> P4_MFP.P4_MFP[2]
//				<0=> P4.2(STADC)
//				<1=> PWM2
//			<o0.18> 施密特触发输入使能
//				<i> P4_MFP.P4_TYPE[2]
//			<o1.4..5> 端口模式
//				<i> P4_PMD.PMD2
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.18> OFF 数字输入通道使能
//				<i> P4_OFFD.OFFD[2]
//			<o3.2> 输出值
//				<i> P4_DOUT.DOUT[2]
//				<0=> 低
//				<1=> 高
//			<o4.2> 去抖使能
//				<i> P4_DBEN.DBEN[2]
//			<o5.2> 中断模式
//				<i> P4_IMD.IMD[2]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P4.3/PWM3
//			<i> Pin: 12
//			<o0.6..7> 多功能管脚选择
//				<i> P4_MFP.P4_MFP[3]
//				<0=> P4.3
//				<1=> PWM3
//			<o0.19> 施密特触发输入使能
//				<i> P4_MFP.P4_TYPE[3]
//			<o1.6..7> 端口模式
//				<i> P4_PMD.PMD3
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.19> OFF 数字输入通道使能
//				<i> P4_OFFD.OFFD[3]
//			<o3.3> 输出值
//				<i> P4_DOUT.DOUT[3]
//				<0=> 低
//				<1=> 高
//			<o4.3> 去抖使能
//				<i> P4_DBEN.DBEN[3]
//			<o5.3> 中断模式
//				<i> P4_IMD.IMD[3]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P4.4/CS
//			<i> Pin: 28
//			<o0.8..9> 多功能管脚选择
//				<i> P4_MFP.P4_MFP[4]
//				<0=> P4.4
//				<1=> CS
//			<o0.20> 施密特触发输入使能
//				<i> P4_MFP.P4_TYPE[4]
//			<o1.8..9> 端口模式
//				<i> P4_PMD.PMD4
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.20> OFF 数字输入通道使能
//				<i> P4_OFFD.OFFD[4]
//			<o3.4> 输出值
//				<i> P4_DOUT.DOUT[4]
//				<0=> 低
//				<1=> 高
//			<o4.4> 去抖使能
//				<i> P4_DBEN.DBEN[4]
//			<o5.4> 中断模式
//				<i> P4_IMD.IMD[4]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P4.5/ALE
//			<i> Pin: 29
//			<o0.10..11> 多功能管脚选择
//				<i> P4_MFP.P4_MFP[5]
//				<0=> P4.5
//				<1=> ALE
//			<o0.21> 施密特触发输入使能
//				<i> P4_MFP.P4_TYPE[5]
//			<o1.10..11> 端口模式
//				<i> P4_PMD.PMD5
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.21> OFF 数字输入通道使能
//				<i> P4_OFFD.OFFD[5]
//			<o3.5> 输出值
//				<i> P4_DOUT.DOUT[5]
//				<0=> 低
//				<1=> 高
//			<o4.5> 去抖使能
//				<i> P4_DBEN.DBEN[5]
//			<o5.5> 中断模式
//				<i> P4_IMD.IMD[5]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P4.6/ICE_CLK
//			<i> Pin: 30
//			<o0.12..13> 多功能管脚选择
//				<i> P4_MFP.P4_MFP[6]
//				<0=> P4.6
//				<1=> ICE_CLK
//			<o0.22> 施密特触发输入使能
//				<i> P4_MFP.P4_TYPE[6]
//			<o1.12..13> 端口模式
//				<i> P4_PMD.PMD6
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.22> OFF 数字输入通道使能
//				<i> P4_OFFD.OFFD[6]
//			<o3.6> 输出值
//				<i> P4_DOUT.DOUT[6]
//				<0=> 低
//				<1=> 高
//			<o4.6> 去抖使能
//				<i> P4_DBEN.DBEN[6]
//			<o5.6> 中断模式
//				<i> P4_IMD.IMD[6]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//		<h> P4.7/ICE_DAT
//			<i> Pin: 31
//			<o0.14..15> 多功能管脚选择
//				<i> P4_MFP.P4_MFP[7]
//				<0=> P4.7
//				<1=> ICE_DAT
//			<o0.23> 施密特触发输入使能
//				<i> P4_MFP.P4_TYPE[7]
//			<o1.14..15> 端口模式
//				<i> P4_PMD.PMD7
//				<0=> 输入
//				<1=> 输出
//				<2=> 开漏
//				<3=> 准双向
//			<o2.23> OFF 数字输入通道使能
//				<i> P4_OFFD.OFFD[7]
//			<o3.7> 输出值
//				<i> P4_DOUT.DOUT[7]
//				<0=> 低
//				<1=> 高
//			<o4.7> 去抖使能
//				<i> P4_DBEN.DBEN[7]
//			<o5.7> 中断模式
//				<i> P4_IMD.IMD[7]
//				<0=> 边沿
//				<1=> 电平
//		</h>
//	</h>
#define __P4MFP_VAL			0x00005000
#define __P4PMD_VAL			0x0000F555
#define __P4OFFD_VAL		0x00000000
#define __P4DOUT_VAL		0x000000FF
#define __P4DBEN_VAL		0x00000000
#define __P4IMD_VAL			0x00000000

//	<h> 中断去抖配置
//		<o0.0..3> 去抖采样周期选择
//			<i> DBNCECON.DBCLKSEL
//			<0=> 每 1 去抖时钟采样中断输入一次
//			<1=> 每 2 去抖时钟采样中断输入一次
//			<2=> 每 4 去抖时钟采样中断输入一次
//			<3=> 每 8 去抖时钟采样中断输入一次
//			<4=> 每 16 去抖时钟采样中断输入一次
//			<5=> 每 32 去抖时钟采样中断输入一次
//			<6=> 每 64 去抖时钟采样中断输入一次
//			<7=> 每 128 去抖时钟采样中断输入一次
//			<8=> 每 256 去抖时钟采样中断输入一次
//			<9=> 每 512 去抖时钟采样中断输入一次
//			<10=> 每 1024 去抖时钟采样中断输入一次
//			<11=> 每 2048 去抖时钟采样中断输入一次
//			<12=> 每 4096 去抖时钟采样中断输入一次
//			<13=> 每 8192 去抖时钟采样中断输入一次
//			<14=> 每 16384 去抖时钟采样中断输入一次
//			<15=> 每 32768 去抖时钟采样中断输入一次
//		<o0.4> 去抖计数器时钟源选择
//			<i> DBNCECON.DBCLKSRC
//			<0=> HCLK
//			<1=> 内部 10KHz 时钟
//		<o0.5> 中断时钟 On 模式
//			<i> DBNCECON.ICLK_ON
//			<0=> 如果中断 GPIOA/B/C/D/E[n] 被禁止，则禁能中断时钟
//			<1=> 总是使能中断时钟
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

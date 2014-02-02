// DescMPU6050Registers.h

#ifndef __Desc_MPU6050_Registers__38287DAD_9439_4BCF_A376_053436826667
#define __Desc_MPU6050_Registers__38287DAD_9439_4BCF_A376_053436826667

#include <stdio.h>
#include <string.h>

#ifndef __DReturn
#define __DReturn	"\r\n"
#endif

class DescMpu6050Register
{
	// p_ucBitBegin: 0 起始
	static unsigned char __GetValue( unsigned char p_ucVal, unsigned char p_ucBitBegin, unsigned char p_ucBitLen )
	{
		static const unsigned char __ucMask[] = {0, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF};

		if( p_ucBitLen > 7 || 0 == p_ucBitLen || p_ucBitLen > p_ucBitBegin+1 )
		{
			throw "error arg";
			return 0;
		}

		return (p_ucVal>>(p_ucBitBegin+1-p_ucBitLen))&__ucMask[p_ucBitLen];
	}

public:
	static bool Desc( unsigned char *p_pRegisters, unsigned char p_usSize, void (*p_pfunc)(const char *, void *), void *p_pUserData )
	{
		static const char *__sExtSyncSel[] = {
			"Input disabled",
			"TEMP_OUT_L[0]",
			"GYRO_XOUT_L[0]",
			"GYRO_YOUT_L[0]",
			"GYRO_ZOUT_L[0]",
			"ACCEL_XOUT_L[0]",
			"ACCEL_YOUT_L[0]",
			"ACCEL_ZOUT_L[0]"
		};
		static const char *__DLPFDesc[] = {
			"260Hz/0ms/256Hz/0.98ms/8kHz",
			"184Hz/2.0ms/188Hz/1.9ms/1kHz",
			"94Hz/3.0ms/98Hz/2.8ms/1kHz",
			"44Hz/4.9ms/42Hz/4.8ms/1kHz",
			"21Hz/8.5ms/20Hz/8.3ms/1kHz",
			"10Hz/13.8ms/10Hz/13.4ms/1kHz",
			"5Hz/19.0ms/5Hz/18.6ms/1kHz",
			"Reserved/Reserved/Reserved"
		};
		static const char *__GyroFullScaleRangeDesc[] = {
			"± 250 °/s",
			"± 500 °/s",
			"± 1000 °/s",
			"± 2000 °/s"
		};
		static const char *__AccelFullScaleRangeDesc[] = {
			"± 2g",
			"± 4g",
			"± 8g",
			"± 16g"
		};
		static const char *__I2CMasterClock[] = {
			"348 kHz 23",
			"333 kHz 24",
			"320 kHz 25",
			"308 kHz 26",
			"296 kHz 27",
			"286 kHz 28",
			"276 kHz 29",
			"267 kHz 30",
			"258 kHz 31",
			"500 kHz 16",
			"471 kHz 17",
			"444 kHz 18",
			"421 kHz 19",
			"400 kHz 20",
			"381 kHz 21",
			"364 kHz 22"
		};
		static const char *__ClockSelectDesc[] = {
			"Internal 8MHz",
			"X axis gyro",
			"Y axis gyro",
			"Z axis gyro",
			"external 32.768kHz",
			"external 19.2MHz",
			"Reserved",
			"Stops the clock and keeps the timing generator in reset",
		};

		char szBuff[128];

		if( 0 == p_pRegisters || 0 == p_usSize || 0 == p_pfunc )
		{
			return false;
		}

		for(unsigned char i=0;i<p_usSize;)
		{
			switch(i)
			{
			case 0:
				{
					if( p_usSize - i >= 3 )
					{
						sprintf( szBuff, "  %*d  0x%02X     PWR_MODE=%d   XG_OFFS_TC=%d   OTP_BNK_VLD=%d"__DReturn,
							3, i, i, __GetValue(p_pRegisters[i],7,1), __GetValue(p_pRegisters[i],6,6), __GetValue(p_pRegisters[i],0,1) );
						p_pfunc( szBuff, p_pUserData );
						i++;

						sprintf( szBuff, "  %*d  0x%02X     PWR_MODE=%d   YG_OFFS_TC=%d   OTP_BNK_VLD=%d"__DReturn,
							3, i, i, __GetValue(p_pRegisters[i],7,1), __GetValue(p_pRegisters[i],6,6), __GetValue(p_pRegisters[i],0,1) );
						p_pfunc( szBuff, p_pUserData );
						i++;

						sprintf( szBuff, "  %*d  0x%02X     PWR_MODE=%d   ZG_OFFS_TC=%d   OTP_BNK_VLD=%d"__DReturn,
							3, i, i, __GetValue(p_pRegisters[i],7,1), __GetValue(p_pRegisters[i],6,6), __GetValue(p_pRegisters[i],0,1) );
						p_pfunc( szBuff, p_pUserData );
						i++;
					}
					else
					{
						i+=3;
					}
				}
				break;

			case 3:
				{
					if( p_usSize - i >= 3 )
					{
						sprintf( szBuff, "  %*d  0x%02X     X_FINE_GAIN=%d"__DReturn,
							3, i, i, p_pRegisters[i] );
						p_pfunc( szBuff, p_pUserData );
						i++;

						sprintf( szBuff, "  %*d  0x%02X     Y_FINE_GAIN=%d"__DReturn,
							3, i, i, p_pRegisters[i] );
						p_pfunc( szBuff, p_pUserData );
						i++;

						sprintf( szBuff, "  %*d  0x%02X     Z_FINE_GAIN=%d"__DReturn,
							3, i, i, p_pRegisters[i] );
						p_pfunc( szBuff, p_pUserData );
						i++;
					}
					else
					{
						i+=3;
					}
				}
				break;

			case 6:
				{
					if( p_usSize - i >= 6 )
					{
						short sVal = ((p_pRegisters[i]<<8)|p_pRegisters[i+1]);
						sprintf( szBuff, "  %*d  0x%02X     x accel bias=%d"__DReturn,
							3, i, i, sVal );
						p_pfunc( szBuff, p_pUserData );
						i+=2;

						sVal = ((p_pRegisters[i]<<8)|p_pRegisters[i+1]);
						sprintf( szBuff, "  %*d  0x%02X     y accel bias=%d"__DReturn,
							3, i, i, sVal );
						p_pfunc( szBuff, p_pUserData );
						i+=2;

						sVal = ((p_pRegisters[i]<<8)|p_pRegisters[i+1]);
						sprintf( szBuff, "  %*d  0x%02X     z accel bias=%d"__DReturn,
							3, i, i, sVal );
						p_pfunc( szBuff, p_pUserData );
						i+=2;
					}
					else
					{
						i+=6;
					}
				}
				break;

			/* 13 14 15 16 自检寄存器*/
			/* 17 18 未知 */

			case 19:
				{
					if( p_usSize - i >= 6 )
					{
						short sVal = ((p_pRegisters[i]<<8)|p_pRegisters[i+1]);
						sprintf( szBuff, "  %*d  0x%02X     x gyro bias user=%d"__DReturn,
							3, i, i, sVal );
						p_pfunc( szBuff, p_pUserData );
						i+=2;

						sVal = ((p_pRegisters[i]<<8)|p_pRegisters[i+1]);
						sprintf( szBuff, "  %*d  0x%02X     y gyro bias user=%d"__DReturn,
							3, i, i, sVal );
						p_pfunc( szBuff, p_pUserData );
						i+=2;

						sVal = ((p_pRegisters[i]<<8)|p_pRegisters[i+1]);
						sprintf( szBuff, "  %*d  0x%02X     z gyro bias user=%d"__DReturn,
							3, i, i, sVal );
						p_pfunc( szBuff, p_pUserData );
						i+=2;
					}
					else
					{
						i+=6;
					}
				}
				break;

			case 25:
				{
					sprintf( szBuff, "  %*d  0x%02X     SMPLRT_DIV=%d"__DReturn,
						3, i, i, p_pRegisters[i] );
					p_pfunc( szBuff, p_pUserData );
					i++;
				}
				break;

			case 26:
				{
					unsigned char ucExSync = __GetValue(p_pRegisters[i],5,3);
					unsigned char ucDlpf = __GetValue(p_pRegisters[i],2,3);
					sprintf( szBuff, "  %*d  0x%02X     EXT_SYNC_SET=%s(%d)   DLPF_CFG=%s(%d)"__DReturn,
						3, i, i, __sExtSyncSel[ucExSync],ucExSync, __DLPFDesc[ucDlpf],ucDlpf );
					p_pfunc( szBuff, p_pUserData );
					i++;
				}
				break;

			case 27:
				{ // 7 6 5 gyro self test
					unsigned char ucVal = __GetValue(p_pRegisters[i],4,2);
					sprintf( szBuff, "  %*d  0x%02X     GyroFullScaleRange=%s(%d)"__DReturn,
						3, i, i, __GyroFullScaleRangeDesc[ucVal],ucVal );
					p_pfunc( szBuff, p_pUserData );
					i++;
				}
				break;

			case 28:
				{ // 7 6 5 accel self test
					unsigned char ucVal = __GetValue(p_pRegisters[i],4,2);
					sprintf( szBuff, "  %*d  0x%02X     AccelFullScaleRange=%s(%d)"__DReturn,
						3, i, i, __AccelFullScaleRangeDesc[ucVal],ucVal );
					p_pfunc( szBuff, p_pUserData );
					i++;
				}
				break;
			/* 29 30 没用*/

			case 31:
				{
					sprintf( szBuff, "  %*d  0x%02X     motion threshold=%d"__DReturn,
						3, i, i, p_pRegisters[i] );
					p_pfunc( szBuff, p_pUserData );
					i++;
				}
				break;

			case 32:
				{
					sprintf( szBuff, "  %*d  0x%02X     motion dur=%d"__DReturn,
					3, i, i, p_pRegisters[i] );
					p_pfunc( szBuff, p_pUserData );
					i++;
				}
				break;

			/* 33 34 不知道*/

			case 35:
				{
					sprintf( szBuff, "  %*d  0x%02X     FIFO enable(%d)=%s%s%s%s%s%s%s%s"__DReturn,
						3, i, i, p_pRegisters[i],
						((p_pRegisters[i]>>7)&1)?"TEMP ":"",
						((p_pRegisters[i]>>6)&1)?"XG ":"",
						((p_pRegisters[i]>>5)&1)?"YG ":"",
						((p_pRegisters[i]>>4)&1)?"ZG ":"",
						((p_pRegisters[i]>>3)&1)?"XYZ_ACCEL ":"",
						((p_pRegisters[i]>>2)&1)?"SLV2 ":"",
						((p_pRegisters[i]>>1)&1)?"SLV1 ":"",
						((p_pRegisters[i]>>0)&1)?"SLV0 ":""
						);
					p_pfunc( szBuff, p_pUserData );
					i++;
				}
				break;

			case 36:
				{
					sprintf( szBuff, "  %*d  0x%02X     MulMasterEN=%d  WAIT_FOR_ES=%d  SLV3_FIFO_EN=%d  I2C_MST_P_NSR=%d  I2C_MST_CLK=%s"__DReturn,
						3, i, i, __GetValue(p_pRegisters[i],7,1),
						__GetValue(p_pRegisters[i],6,1),
						__GetValue(p_pRegisters[i],5,1),
						__GetValue(p_pRegisters[i],4,1),
						__I2CMasterClock[__GetValue(p_pRegisters[i],3,4)]
						);
					p_pfunc( szBuff, p_pUserData );
					i++;
				}
				break;

			/* 37-53 Slave 控制相关, 54 I2C_MST_STATUS, 55 INT_PIN_CFG*/

			case 56:
				{
					sprintf( szBuff, "  %*d  0x%02X     INT_ENABLE,  MO_EN=%d   FIFO_OFLOW_EN=%d   I2C_MST_INT_EN=%d   DMP_INT_EN=%d   DATA_RDY_EN=%d"__DReturn,
						3, i, i,
						__GetValue(p_pRegisters[i],6,1),
						__GetValue(p_pRegisters[i],4,1),
						__GetValue(p_pRegisters[i],3,1),
						__GetValue(p_pRegisters[i],1,1),
						__GetValue(p_pRegisters[i],0,1)
						);
					p_pfunc( szBuff, p_pUserData );
					i++;
				}
				break;

			/* 57 DMP INT status */
			/* 58 INT status */

			/* 59-96 a/g/t 和 外部采样数据的寄存器 */

			/* 97 Motion detect status */
			/* 98 未知 */
			/* 99-102 4个slave设备的 data out */
			/* 103 延迟控制*/
			/* 104 用于复位陀螺仪、加速计、温度的数据通道*/
			/* 105 Motion detect control */

			case 106:
				{
					sprintf( szBuff, "  %*d  0x%02X     FIFO_EN=%d   I2C_MST_EN=%d"__DReturn,
						3, i, i,
						__GetValue(p_pRegisters[i],6,1),
						__GetValue(p_pRegisters[i],5,1)
						);
					p_pfunc( szBuff, p_pUserData );
					i++;
				}
				break;

			case 107:
				{
					sprintf( szBuff, "  %*d  0x%02X     SLEEP=%d   CYCLE=%d   TEMP_DIS=%d   CLKSEL=%s"__DReturn,
						3, i, i,
						__GetValue(p_pRegisters[i],6,1),
						__GetValue(p_pRegisters[i],5,1),
						__GetValue(p_pRegisters[i],3,1),
						__ClockSelectDesc[__GetValue(p_pRegisters[i],2,3)]
						);
					p_pfunc( szBuff, p_pUserData );
					i++;
				}
				break;

			case 108:
				{
					sprintf( szBuff, "  %*d  0x%02X     LP_WAKE_CTRL=%d   STBY_XA=%d   STBY_YA=%d   STBY_ZA=%d   STBY_XG=%d   STBY_YG=%d   STBY_ZG=%d"__DReturn,
						3, i, i,
						__GetValue(p_pRegisters[i],7,2),
						__GetValue(p_pRegisters[i],5,1),
						__GetValue(p_pRegisters[i],4,1),
						__GetValue(p_pRegisters[i],3,1),
						__GetValue(p_pRegisters[i],2,1),
						__GetValue(p_pRegisters[i],1,1),
						__GetValue(p_pRegisters[i],0,1)
					);
					p_pfunc( szBuff, p_pUserData );
					i++;
				}
				break;

			/* 109 BANK_SEL, 110 MEM_START_ADDR, 111 MEM_R_W */
			/* 112 DMP program start high, 113 DMP program start low */
			/* 114 FIFO_COUNT_H, 115 FIFO_COUNT_L, 116 FIFO_R_W */
			/* 117 WHO_AM_I */

			default:
				i++;
				break;
			}
		}

		return true;
	}
};


#endif	// __Desc_MPU6050_Registers__38287DAD_9439_4BCF_A376_053436826667


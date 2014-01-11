// mpumanager.cpp

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "mpumanager.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#include <time.h>

MPUManager::MPUManager()
{
	m_usSampleRate = 100;
	m_iUpdateType = __UDT_Unknown;
	m_bRunning = false;
	m_bStop = false;
	m_pCallback = 0;
	m_iThreadID = 0;
	m_usPackageLength = 0;
	memset( &m_Data, 0, sizeof(__CaptureData_t) );
	m_fCurAccelSensitivity = 0.0f;
	m_fCurGyroSensitivity = 0.0f;
	m_usDefaultFeatures = DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|DMP_FEATURE_GYRO_CAL|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|DMP_FEATURE_PEDOMETER;
}

MPUManager::~MPUManager()
{

}

MPUManager *MPUManager::GetInstance()
{
	static MPUManager _Ins;

	return &_Ins;
}

bool MPUManager::__LoadDataFromDevice( __CaptureData_t *p_pData )
{
	static short sDataGyro[3] = {0};
	static short sDataAccel[3] = {0};
	static unsigned short ucLen = 0;
	static unsigned char ucMore = 0;
	static short sMask;

	if( 0 == __mpu_get_fifo_bytes_count(&ucLen) && ucLen >= m_usPackageLength )
	{
		if( 0 == dmp_read_fifo( sDataGyro, sDataAccel, p_pData->m_lDataQuat, &p_pData->m_ulTimestamp, &sMask, &ucMore ) )
		{
			if( m_usDefaultFeatures&DMP_FEATURE_SEND_CAL_GYRO )
			{
				p_pData->m_sDataGyro[0] = (float)sDataGyro[0]/m_fCurGyroSensitivity;
				p_pData->m_sDataGyro[1] = (float)sDataGyro[1]/m_fCurGyroSensitivity;
				p_pData->m_sDataGyro[2] = (float)sDataGyro[2]/m_fCurGyroSensitivity;
			}

			if( m_usDefaultFeatures&DMP_FEATURE_SEND_RAW_ACCEL )
			{
				p_pData->m_sDataAccel[0] = (float)sDataAccel[0]/m_fCurAccelSensitivity;
				p_pData->m_sDataAccel[1] = (float)sDataAccel[1]/m_fCurAccelSensitivity;
				p_pData->m_sDataAccel[2] = (float)sDataAccel[2]/m_fCurAccelSensitivity;
			}

			if( 0 != __mpu_get_temperature( &p_pData->m_fTemperature ) )
			{
				p_pData->m_fTemperature = 0.0f;
			}
			return true;
		}
	}

	return false;
}

void *MPUManager::__ThreadWrap( void *p_pArg )
{
	if( __UDT_Push == ((MPUManager *)p_pArg)->m_iUpdateType )
	{
		((MPUManager *)p_pArg)->__WorkThreadPush();
	}
	else
	{
		((MPUManager *)p_pArg)->__WorkThread();
	}

	return 0;
}

void MPUManager::__WorkThread()
{
	for(;!m_bStop;)
	{
		usleep(5*1000);

		__DataSwap<__CaptureData_t>::__Buffer_t *pBuffer = m_DataSwap.GetBuffer();
		if( __LoadDataFromDevice( &pBuffer->m_Data ) )
		{
			pBuffer->m_iHaveData = 1;
			m_DataSwap.PushData();
		}
	}
}

void MPUManager::__WorkThreadPush()
{
	__CaptureData_t _Data;

	memset( &_Data, 0, sizeof(__CaptureData_t) );

	for(;!m_bStop;)
	{
		usleep(5*1000);

		if( __LoadDataFromDevice( &_Data ) )
		{
			m_pCallback( &_Data );
		}
	}
}

bool MPUManager::RemoveDataAccel()
{
	if( m_bRunning )
	{
		return false;
	}
	else
	{
		m_usDefaultFeatures &= ~DMP_FEATURE_SEND_RAW_ACCEL;
		return true;
	}
}

bool MPUManager::RemoveDataGyro()
{
	if( m_bRunning )
	{
		return false;
	}
	else
	{
		m_usDefaultFeatures &= ~DMP_FEATURE_SEND_CAL_GYRO;
		return true;
	}
}

bool MPUManager::RemoveDataQuaternion()
{
	if( m_bRunning )
	{
		return false;
	}
	else
	{
		m_usDefaultFeatures &= ~DMP_FEATURE_6X_LP_QUAT;
		return true;
	}
}

bool MPUManager::SetSampleRate( unsigned short p_iRate )
{
	if( m_bRunning || p_iRate <= 0 )
	{
		return false;
	}
	else
	{
		m_usSampleRate = (p_iRate>200)?200:p_iRate;

		return true;
	}
}

bool MPUManager::SetUpdateDataType( MPUManager::__UpdateDataType p_iType, void (*p_pfunc)(const __CaptureData_t *) )
{
	if( m_bRunning )
	{
		return false;
	}
	else
	{
		switch(p_iType)
		{
		case __UDT_Push:
		case __UDT_Pull:
		case __UDT_PullRealTime:
			m_iUpdateType = p_iType;
			break;

		default:
			return false;
		}

		if( __UDT_Push == p_iType && 0 == p_pfunc )
		{
			return false;
		}

		m_pCallback = p_pfunc;

		return true;
	}
}

bool MPUManager::__StartDevice()
{
	if( 0 != mpu_init() )
	{
		printf("mpu_init failed\n");
		return false;
	}

	/* Wake up all sensors. */
	if( 0 != mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL) )
	{
		printf("mpu_set_sensors failed\n");
		return false;
	}

	/* Push both gyro and accel data into the FIFO. */
	if( 0 != mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL) )
	{
		printf("mpu_configure_fifo failed\n");
		return false;
	}

	if( 0 != mpu_set_sample_rate(m_usSampleRate) )
	{
		printf("mpu_set_sample_rate failed\n");
		return false;
	}

	if( 0!= dmp_load_motion_driver_firmware() )
	{
		printf("dmp_load_motion_driver_firmware failed\n");
		return false;
	}

	if( 0 != dmp_enable_feature(m_usDefaultFeatures) )
	{
		printf("dmp_enable_feature failed\n");
		return false;
	}

	if( 0 != dmp_set_fifo_rate(m_usSampleRate) )
	{
		printf("dmp_set_fifo_rate failed\n");
		return false;
	}
	if( 0 != mpu_get_gyro_sens( &m_fCurGyroSensitivity ) )
	{
		printf("mpu_get_gyro_sens failed\n");
		return false;
	}
	else
	{
		printf("Current Gyro Sensitivity: %.2f\n", m_fCurGyroSensitivity);
	}

	unsigned short usSens;
	if( 0 != mpu_get_accel_sens(&usSens) )
	{
		printf("mpu_get_accel_sens failed\n");
		return false;
	}
	else
	{
		m_fCurAccelSensitivity = usSens;
		printf("Current Accel Sensitivity: %.2f\n", m_fCurAccelSensitivity);
	}
	
	m_usPackageLength = __dmp_get_packet_length();
	unsigned short usfr = 0;
	dmp_get_fifo_rate( &usfr );
	printf("DMP start success\npackage length = %d\nFIFO rate = %d\n", m_usPackageLength, usfr);

	if( 0 != mpu_set_dmp_state(1) )
	{
		printf("mpu_set_dmp_state 1 failed\n");
		return false;
	}

	return true;
}

bool MPUManager::Start()
{
	if( m_bRunning )
	{
		return false;
	}

	if( __UDT_Unknown == m_iUpdateType )
	{
		return false;
	}

	if( __UDT_Push == m_iUpdateType && 0 == m_pCallback )
	{
		return false;
	}

	if( !__StartDevice() )
	{
		return false;
	}

	if( __UDT_PullRealTime != m_iUpdateType )
	{
		m_bStop = false;
		if( 0 != pthread_create( &m_iThreadID, 0, __ThreadWrap, this ) )
		{
			Stop();
			return false;
		}
	}

	m_bRunning = true;

	return true;
}

void MPUManager::Stop()
{
	if( 0 != m_iThreadID )
	{
		m_bStop = true;
		pthread_join( m_iThreadID, 0 );
	}

	if( 0 != mpu_set_dmp_state(0) )
	{
		printf("mpu_set_dmp_state 0 failed\n");
	}

	if( 0 != __mpu_set_sleep_mode(1) )
	{
		printf("__mpu_set_sleep_mode 1 failed\n");
	}

	m_bRunning = false;
	m_bStop = false;
	m_iThreadID = 0;

	printf("DMP stopped\n");
}

const __CaptureData_t *MPUManager::GetData()
{
	__CaptureData_t *pRet = NULL;

	if( m_bRunning )
	{
		switch(m_iUpdateType)
		{
		case __UDT_PullRealTime:
			{
				if( __LoadDataFromDevice( &m_Data ) )
				{
					pRet = &m_Data;
				}
			}
			break;

		case __UDT_Pull:
			pRet = m_DataSwap.GetData();
			break;

		default:
			break;
		}
	}

	return pRet;
}

// mpumanager.h

#ifndef __mpu_manager_DD5AB2ED_AD47_4BD8_9BC7_3D1EAD1D88DF_H
#define __mpu_manager_DD5AB2ED_AD47_4BD8_9BC7_3D1EAD1D88DF_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

struct __CaptureData_t 
{
	float m_sDataGyro[3];		// x y z
	float m_sDataAccel[3];		// x y z
	long m_lDataQuat[4];		// w x y z
	float m_fTemperature;
	unsigned long m_ulTimestamp;
};

class MPUManager
{
	/*
	    <----data------

               |  1.get buffer
               |--------------->
     <---------|
    3. get data|  2. push data
               |<---------------
               |
	*/
	// avoid mutex lock
	template <class _Tp>
	class __DataSwap
	{
		friend class MPUManager;

		typedef _Tp _DataType;

		struct __Buffer_t
		{
			int m_iHaveData;
			_DataType m_Data;
		};

		__Buffer_t m_SwapArray[3];
		unsigned long m_ulData;
		unsigned long m_ulSwap;
		unsigned long m_ulBuffer;
		// Raspberry Pi, single core CPU
		/*volatile */int m_iLock;

		enum
		{
			__Status_Free = 0,
			__Status_Lock = 1,
		};

	public:
		__DataSwap()
		{
			m_iLock = __Status_Free;
			m_ulData = (unsigned long)&m_SwapArray[0];
			m_ulSwap = (unsigned long)&m_SwapArray[1];
			m_ulBuffer = (unsigned long)&m_SwapArray[2];
			memset( &m_SwapArray[0], 0, sizeof(m_SwapArray) );
		}

		__Buffer_t *GetBuffer()
		{
			return (__Buffer_t *)m_ulBuffer;
		}

		void PushData()
		{
			for(;;)
			{
				if( __sync_bool_compare_and_swap( &m_iLock, __Status_Free, __Status_Lock ) )
				{
					m_ulBuffer = __sync_val_compare_and_swap( &m_ulSwap, m_ulSwap, m_ulBuffer );
					__sync_bool_compare_and_swap( &m_iLock, __Status_Lock, __Status_Free );
					break;
				}
			}
		}

		_DataType *GetData()
		{
			for(;;)
			{
				if( __sync_bool_compare_and_swap( &m_iLock, __Status_Free, __Status_Lock ) )
				{
					m_ulData = __sync_val_compare_and_swap( &m_ulSwap, m_ulSwap, m_ulData );
					__sync_bool_compare_and_swap( &m_iLock, __Status_Lock, __Status_Free );
					break;
				}
			}

			__Buffer_t *pData = (__Buffer_t *)m_ulData;
			if( 0 != pData->m_iHaveData )
			{
				pData->m_iHaveData = 0;
				return &pData->m_Data;
			}
			else
			{
				return 0;
			}
		}
	};

public:
	typedef enum
	{
		__UDT_Unknown	= 0,
		__UDT_Push,			// push to callback
		__UDT_Pull,			// user pull current data when need
		__UDT_PullRealTime,	// timely pull data, otherwise result in FIFO overflow
	} __UpdateDataType;

private:
	MPUManager();
	~MPUManager();

	bool __StartDevice();
	bool __LoadDataFromDevice( __CaptureData_t *p_pData );
	static void *__ThreadWrap( void *p_pArg );
	void __WorkThread();
	void __WorkThreadPush();

	// current accel sensitivity
	float m_fCurAccelSensitivity;
	// current gyroscope sensitivity
	float m_fCurGyroSensitivity;
	// use for __UDT_PullRealTime mode
	__CaptureData_t m_Data;
	// data swapper
	__DataSwap<__CaptureData_t> m_DataSwap;
	// update data mode type
	__UpdateDataType m_iUpdateType;
	bool m_bRunning;
	bool m_bStop;
	pthread_t m_iThreadID;
	unsigned short m_usPackageLength;
	void (*m_pCallback)(const __CaptureData_t *);
	unsigned short m_usSampleRate;
	unsigned short m_usDefaultFeatures;

public:
	static MPUManager *GetInstance();

public:
	// default data include:
	//     Raw accelerometer data
	//     Calibrated gyroscope data
	//     Quaternion
	bool RemoveDataAccel();
	bool RemoveDataGyro();
	bool RemoveDataQuaternion();

	bool SetSampleRate( unsigned short p_iRate );
	bool SetUpdateDataType( __UpdateDataType p_iType, void (*p_pfunc)(const __CaptureData_t *) );

	// start capture data
	bool Start();

	// stop capture, device goto sleep mode
	void Stop();

	// pull data
	const __CaptureData_t *GetData();
};

#endif	// __mpu_manager_DD5AB2ED_AD47_4BD8_9BC7_3D1EAD1D88DF_H

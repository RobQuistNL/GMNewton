/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "dContainersStdAfx.h"
#include "dThread.h"

#ifdef _MSC_VER
	#include <process.h>
	#include <Windows.h>
	#include <crtdbg.h>
#endif

#ifdef _POSIX_VER
	#include <pthread.h>
	#include <semaphore.h>
	#include <unistd.h>
#endif



static inline void dInterlockedIncrement (int* const Addend )
{
	#if (defined (_POSIX_VER))
		__sync_fetch_and_add ((int32_t*)Addend, 1 );
	#endif

	#if (defined (_MAC_VER))
		OSAtomicAdd32 (1, (int32_t*)Addend);
	#endif

	#if (defined (_MSC_VER))
		InterlockedIncrement((long*) Addend);
	#endif
}


static inline void dInterlockedDecrement(int* const Addend)
{
	#if (defined (_POSIX_VER))
		__sync_fetch_and_sub ((int32_t*)Addend, 1 );
	#endif

	#if (defined (_MAC_VER))
		OSAtomicAdd32 (-1, (int32_t*)Addend);
	#endif

	#if (defined (_MSC_VER))
		InterlockedDecrement((long*) Addend);
	#endif
}


static inline int dInterlockedExchange (int* spin, int testValue)
{
	#if (defined (_POSIX_VER))
		return __sync_val_compare_and_swap((int32_t*)spin, !testValue, testValue);
	#endif

	#if (defined (_MAC_VER))
		return !OSAtomicCompareAndSwap32(!testValue, testValue, (int32_t*) spin);
	#endif

	#if (defined (_MSC_VER))
		return InterlockedExchange ((long*) spin, testValue);
	#endif
}








dThread::dThread(void)
	:m_taskSuspendedCount (1), m_taskExecuting (1), m_terminated(0)
{
//m_taskSuspendedCount = 1;

#if defined (_MSC_VER)
	m_threadhandle = _beginthreadex( NULL, 0, TaskCallback, this, 0, NULL);
#endif

#if (defined (_POSIX_VER) || defined (_MAC_VER))
	pthread_create( &m_threadhandle, NULL, TaskCallback, this);
#endif
}

dThread::~dThread(void)
{
	if (!m_terminated) {
		TerminateTask ();
	}
}


#if defined (_MSC_VER)
unsigned _stdcall dThread::TaskCallback(void *param)
#endif
#if (defined (_POSIX_VER) || defined (_MAC_VER))
void* dThread::TaskCallback(void *param)
#endif
{
	dThread* const me = (dThread*) param;
	me->ExcuteTask();
	return 0;
}

void dThread::TerminateTask ()
{
	dInterlockedExchange((int*) &m_terminated, 1);
	while (m_taskExecuting) {
		YieldTime();
	}
}




void dThread::ContinueExecution ()
{
	dInterlockedDecrement((int*) &m_taskSuspendedCount);
	_ASSERTE (!(m_taskSuspendedCount & 0x80000000));
}


void dThread::StopsExecution ()
{
	dInterlockedIncrement((int*) &m_taskSuspendedCount);
	_ASSERTE (!(m_taskSuspendedCount & 0x80000000));
	while (m_taskExecuting) {
		YieldTime();
	}
}



void dThread::ExcuteTask()
{
	while (!m_terminated) {
		if (m_taskSuspendedCount) {
			dInterlockedExchange((int*) &m_taskExecuting, 0);
			//WaitForSingleObject(m_controlKey, INFINITE);
			while (m_taskSuspendedCount) {
				if (m_terminated) {
					break;
				}
				YieldTime();
			}
		}
		dInterlockedExchange((int*) &m_taskExecuting, 1);
		RunMyTask ();
		YieldTime();
	}
	dInterlockedExchange((int*) &m_taskExecuting, 0);
}

void dThread::YieldTime()
{
	#if defined (_MSC_VER)
		Sleep(0);
	#endif

	#if (defined (_POSIX_VER) || defined (_MAC_VER))
		sched_yield();
	#endif
}

void dThread::Lock(unsigned& lockVar)
{
	_ASSERTE (sizeof (unsigned) == sizeof (long));
	while (dInterlockedExchange((int*) &lockVar, 1)) {
		YieldTime();
	}
}


void dThread::Unlock(unsigned& lockVar)
{
	_ASSERTE (sizeof (unsigned) == sizeof (long));
	lockVar = 0;
}



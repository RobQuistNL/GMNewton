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


#ifndef __DTHREAD_H__
#define __DTHREAD_H__

#if ( defined (_LINUX_VER) || defined (_MINGW_32_VER) || defined (_MINGW_64_VER) )
    #define _POSIX_VER
    #include <pthread.h>
#endif

#if ( defined (_MINGW_32_VER) || defined (_MINGW_64_VER) )
    #include <stdint.h>
    #include <crtdbg.h>
#endif

class dThread
{
	public:
	dThread(void);
	virtual ~dThread(void);

	virtual unsigned RunMyTask () { return 1;}

	void ContinueExecution (); 
	void StopsExecution (); 
	void TerminateTask ();

	void YieldTime();
	void Lock(unsigned& lockVar);
	void Unlock(unsigned& lockVar);

	private:
	void ExcuteTask();

#if defined (_MSC_VER)
	static unsigned _stdcall TaskCallback(void *param);
#endif

#if (defined (_POSIX_VER) || defined (_MAC_VER))
	static void* TaskCallback(void *Param);
#endif


	unsigned m_taskSuspendedCount;
	unsigned m_taskExecuting;
	unsigned m_terminated;

#if defined (_MSC_VER)
	uintptr_t m_threadhandle;
#endif

#if (defined (_POSIX_VER) || defined (_MAC_VER))
	pthread_t m_threadhandle;
#endif
};

#endif
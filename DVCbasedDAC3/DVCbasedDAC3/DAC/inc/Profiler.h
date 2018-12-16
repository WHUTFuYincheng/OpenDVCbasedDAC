// Profiler.h: interface for the CProfiler class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_PROFILER_H__02007357_28E5_4A9E_A2B8_3EB71BA708D2__INCLUDED_)
#define AFX_PROFILER_H__02007357_28E5_4A9E_A2B8_3EB71BA708D2__INCLUDED_

#include "common.h"	// Added by ClassView


#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class CProfiler  
{
public:
    CProfiler();

    VOID                Reset();
    VOID                Start();
    VOID                Stop();
    VOID                Advance();
    REAL                GetAbsTime();
    REAL                GetAppTime();
    REAL                GetElapsedTime();
    DWORD               GetAppMS();
    //VOID SetUsingQPF(BOOL bUsingQPF) { m_bUsingQPF = bUsingQPF; }

protected:    
    BOOL                m_bUsingQPF;
    BOOL                m_bTimerStopped;

    LONGLONG            m_llQPFTicksPerSec;
    LONGLONG            m_llStopTime;
    LONGLONG            m_llLastElapsedTime;
    LONGLONG            m_llBaseTime;
    LARGE_INTEGER       m_qwTime;

    REAL                m_fLastElapsedTime;
    REAL                m_fBaseTime;
    REAL                m_fStopTime;
    REAL                m_fTime;

    VOID                Prepare();
    VOID                QPFPrepare();
};

#endif // !defined(AFX_PROFILER_H__02007357_28E5_4A9E_A2B8_3EB71BA708D2__INCLUDED_)

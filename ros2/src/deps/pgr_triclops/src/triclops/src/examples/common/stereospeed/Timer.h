//=============================================================================
// Copyright � 2011 Point Grey Research, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with PGR.
//
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================

#if defined(WIN32) || defined(WIN64)
#include <windows.h>
#else
#include <sys/time.h>
#endif

class Timer
{
public:
    Timer();
    ~Timer() {};

    void Start();
    void Stop();
    void Pause();
    void Unpause();

    double GetElapsedTimeInMicroseconds();
    double GetElapsedTimeInMilliseconds();
    double GetElapsedTimeInSeconds();


private:

    void GetTimeOnTheFly();

#if defined(WIN32) || defined(WIN64)
    double m_pcFreq;

    __int64 m_startTime;
    __int64 m_endTime;
    
#else
    timeval m_startTime;
    timeval m_endTime;
#endif

    bool   m_isStarted, m_isPaused;
    double m_elapsedTime;
};

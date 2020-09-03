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


#include "Timer.h"

#include <stdio.h>



Timer::Timer() : m_isStarted(false), m_isPaused(true), m_elapsedTime(0.0)
{    
#if defined(WIN32) || defined(WIN64)
    LARGE_INTEGER li;
    QueryPerformanceFrequency(&li);
    m_pcFreq = double(li.QuadPart)/1000000.0;
#else
#endif
}

void Timer::Start()
{
#if defined(WIN32) || defined(WIN64)
    LARGE_INTEGER li;
    QueryPerformanceCounter(&li);
    m_startTime = li.QuadPart;
#else
    gettimeofday(&m_startTime, NULL);
#endif

    m_isStarted = true;
    m_isPaused = false;
    m_elapsedTime = 0.0;
}

void Timer::Pause()
{
   if ( ! m_isPaused )
   {
#if defined(WIN32) || defined(WIN64)
    LARGE_INTEGER li;
    QueryPerformanceCounter(&li);
    m_endTime = li.QuadPart;
    m_elapsedTime += (m_endTime - m_startTime)/m_pcFreq;
#else
    gettimeofday(&m_endTime, NULL);
    m_elapsedTime += (m_endTime.tv_sec - m_startTime.tv_sec) * 1000000.0;
    m_elapsedTime += m_endTime.tv_usec - m_startTime.tv_usec;
#endif

    m_isPaused = true;
   }
}

void Timer::Unpause()
{
   if ( m_isStarted )
   {
#if defined(WIN32) || defined(WIN64)
    LARGE_INTEGER li;
    QueryPerformanceCounter(&li);
    m_startTime = li.QuadPart;
#else
    gettimeofday(&m_startTime, NULL);
#endif

    m_isPaused = false;
   }
}

void Timer::Stop()
{
    m_isStarted = false;
    Timer::Pause();
}


void Timer::GetTimeOnTheFly()
{
    Timer::Pause();
    Timer::Unpause();
}

double Timer::GetElapsedTimeInMicroseconds()
{
    if ( m_isStarted && ! m_isPaused )
    {
        GetTimeOnTheFly();
    }
    return m_elapsedTime;
}

double Timer::GetElapsedTimeInMilliseconds()
{
    return GetElapsedTimeInMicroseconds() / 1000.0;
}

double Timer::GetElapsedTimeInSeconds()
{
    return GetElapsedTimeInMilliseconds() / 1000.0;
}

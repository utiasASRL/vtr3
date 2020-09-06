#include "timing.h"
#include <cutil.h>
#include <iostream>
#include <iomanip>
#include <cuda_runtime_api.h>

Timer::Timer(){
  m_maxTagLen = 9;
}
Timer::~Timer()
{
  std::map<std::string, std::pair<unsigned, unsigned> >::iterator i = m_timers.begin();
  
  //for( ; i != m_timers.end(); i++) {
  //  cutDeleteTimer(i->second.second);
  //}	
}
  
void Timer::start(std::string const & tag, int N){
  cudaDeviceSynchronize();
  std::pair<unsigned,unsigned> & p = getTimer(tag);
  p.first+= N;
  cutStartTimer(p.second);
}

void Timer::stop(std::string const & tag)
{
  cudaDeviceSynchronize();
  std::pair<unsigned,unsigned> & p = getTimer(tag);
  cutStopTimer(p.second);
}

void Timer::printTime()
{
  std::cout << "Operation";
  for(unsigned i=2; i < m_maxTagLen; i++)
    std::cout << "  ";
  std::cout << "Time\n";
  for(unsigned i=2; i < m_maxTagLen + 46; i++)
    std::cout << "-";
  std::cout << std::endl;

  std::map<std::string, std::pair<unsigned, unsigned> >::iterator i = m_timers.begin();
  for( ; i != m_timers.end(); i++) {
	  std::cout << std::setfill(' ') << std::setw((std::streamsize)m_maxTagLen) << i->first << ": ";
    float ms = cutGetTimerValue(i->second.second);
    std::cout << std::setw(10) << ms << "ms (" << std::setw(6) << i->second.first;
    std::cout << " " << std::setw(10) << ms / i->second.first << "ms)\n";
  }

}

float Timer::getTime(std::string const & tag)
{
  std::pair<unsigned,unsigned> & p = getTimer(tag);
  return cutGetTimerValue(p.second);
}

unsigned Timer::getCount(std::string const & tag)
{
  std::pair<unsigned,unsigned> & p = getTimer(tag);
  return p.first;
}


std::pair<unsigned,unsigned> & Timer::getTimer(std::string const & tag)
{

  std::map<std::string, std::pair<unsigned, unsigned> >::iterator i = m_timers.find(tag);
  if(i == m_timers.end()) {
    m_maxTagLen = std::max(tag.size(), m_maxTagLen);
    unsigned t;
    cutCreateTimer(&t);
    return m_timers[tag] = std::make_pair(0,t);
  } else {
    return i->second;
  }

}


 Timer GlobalTimer = Timer();

#ifndef TIMING_FTK_H
#define TIMING_FTK_H

#include <string>
#include <map>

#ifndef GPUSURF_API
#    ifdef WIN32
#      ifdef gpusurf_EXPORTS
#        define GPUSURF_API __declspec(dllexport)
#      else
#        define GPUSURF_API __declspec(dllimport)
#      endif
#    else
#      define GPUSURF_API
#    endif
#endif


class Timer
{
 public:
  GPUSURF_API Timer();
  GPUSURF_API ~Timer();
  
  GPUSURF_API void start(std::string const & tag, int N = 1);
  GPUSURF_API void stop(std::string const & tag);

  GPUSURF_API void printTime();
  GPUSURF_API float getTime(std::string const & tag);
  GPUSURF_API unsigned getCount(std::string const & tag);

 private:
  std::pair<unsigned,unsigned> & getTimer(std::string const &);

  std::map<std::string, std::pair<unsigned, unsigned> > m_timers;
  size_t m_maxTagLen;
 

};

extern GPUSURF_API  Timer GlobalTimer;

#endif








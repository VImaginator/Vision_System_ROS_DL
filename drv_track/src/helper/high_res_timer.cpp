/*
 * high_res_timer.cpp
 *
 *      Author: Alex Teichman
 *
 */


#include "high_res_timer.h"

HighResTimer::HighResTimer(const std::string& description,
                           const clockid_t& clock)
  : description_(description),
    total_us_(0),
    clock_(clock)
{
}

void HighResTimer::start()
{
  clock_gettime(clock_, &start_);
}

void HighResTimer::stop()
{
  clock_gettime(clock_, &end_);
  total_us_ += 1e6 * (end_.tv_sec - start_.tv_sec) + 1e-3 * (end_.tv_nsec - start_.tv_nsec);
}

void HighResTimer::reset(const std::string& description)
{
  description_ = description;
  total_us_ = 0;
}

void HighResTimer::reset()
{
  total_us_ = 0;
}

double HighResTimer::getMicroseconds() const
{
  return total_us_;
}

double HighResTimer::getMilliseconds() const
{
  return getMicroseconds() / 1000.;
}

double HighResTimer::getSeconds() const
{
  return getMilliseconds() / 1000.;
}

double HighResTimer::getMinutes() const
{
  return getSeconds() / 60.;
}

double HighResTimer::getHours() const
{
  return getMinutes() / 60.;
}

std::string HighResTimer::reportMicroseconds() const
{
  std::ostringstream oss; oss << description_ << ": " << getMicroseconds() << " microseconds.";
  return oss.str();
}

std::string HighResTimer::reportMilliseconds() const
{
  std::ostringstream oss; oss << description_ << ": " << getMilliseconds() <
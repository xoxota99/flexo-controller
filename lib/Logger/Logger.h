
#ifndef __LOGGER_H__
#define __LOGGER_H__
/*
   Logger
   Copyright (C) 2018  Philippe Desrosiers

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// #include <math.h>
#include "Arduino.h"
#include <cstdarg>

#define FOREACH_LEVEL(LEVEL) \
  LEVEL(TRACE)               \
  LEVEL(DEBUG)               \
  LEVEL(INFO)                \
  LEVEL(WARN)                \
  LEVEL(ERROR)               \
  LEVEL(FATAL)

#define GENERATE_ENUM(ENUM) ENUM,
#define GENERATE_STRING(STRING) #STRING,

class Logger
{
public:
  enum LogLevel
  {
    FOREACH_LEVEL(GENERATE_ENUM)
  };

  static LogLevel level;
  static void log(const LogLevel level, const char *format, ...);
  // static void setLogLevel(Logger::LogLevel level);
  // static Logger::LogLevel getLogLevel();

  static void trace(const char *format, ...);
  static void debug(const char *format, ...);
  static void info(const char *format, ...);
  static void warn(const char *format, ...);
  static void error(const char *format, ...);
  static void fatal(const char *format, ...);

private:
  static void _log_va_list(const LogLevel level, const char *format, va_list args);

  // static Logger* m_pInstance;
  Logger(){};               // Private so that it can  not be called
  Logger(Logger const &){}; // copy constructor is private
  Logger &operator=(Logger const &)
  {
    return *this;
  }; // assignment operator is private
};

#endif //__LOGGER_H__

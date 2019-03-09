#ifndef __STU_LOGGER_H__
#define __STU_LOGGER_H__

#include "Arduino.h"
#include <cstdarg>

#define LOGGER_HANDLER_SIG(l) static void l(const char *format, ...);

const char *lvlStrings[6] = {
    "TRACE",
    "DEBUG",
    "INFO",
    "WARN",
    "ERROR",
    "FATAL"};

class Logger
{
  public:
    enum LogLevel
    {
        TRACE = 0,
        DEBUG,
        INFO,
        WARN,
        ERROR,
        FATAL
    };

    static LogLevel level;
    static void log(const LogLevel level, const char *format, ...);

    LOGGER_HANDLER_SIG(trace)
    LOGGER_HANDLER_SIG(debug)
    LOGGER_HANDLER_SIG(info)
    LOGGER_HANDLER_SIG(warn)
    LOGGER_HANDLER_SIG(error)
    LOGGER_HANDLER_SIG(fatal)

  private:
    static void _log_va_list(const LogLevel level, const char *format, va_list args);

    // static Logger* m_pInstance;
    Logger(){};                                          // Private so that it can  not be called
    Logger(Logger const &){};                            // copy constructor is private
    Logger &operator=(Logger const &) { return *this; }; // assignment operator is private
};

#endif //__STU_LOGGER_H__

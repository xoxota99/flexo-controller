/*
   stewy
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

#include "Logger.h"

#define LOGGER_HANDLER(l, L)               \
    void Logger::l##(const char *fmt, ...) \
    {                                      \
        if (L## >= Logger::level)          \
        {                                  \
            va_list args;                  \
            va_start(args, fmt);           \
            _log_va_list(L##, fmt, args);  \
            va_end(args);                  \
        }                                  \
    }

Logger::LogLevel Logger::level = TRACE;

void Logger::_log_va_list(const LogLevel level, const char *fmt, va_list args)
{
    char buffer[256];
    // va_start (args, fmt);
    vsprintf(buffer, fmt, args);
    Serial.printf("[%s] - %s\n", lvlStrings[level], buffer);
    // va_end (args);
}

void Logger::log(const LogLevel level, const char *fmt, ...)
{
    if (level >= Logger::level)
    {
        va_list args;
        va_start(args, fmt);
        _log_va_list(level, fmt, args);
        va_end(args);
    }
}

LOGGER_HANDLER(trace, TRACE)
LOGGER_HANDLER(debug, DEBUG)
LOGGER_HANDLER(info, INFO)
LOGGER_HANDLER(warn, WARN)
LOGGER_HANDLER(error, ERROR)
LOGGER_HANDLER(fatal, FATAL)

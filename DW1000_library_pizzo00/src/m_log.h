#pragma once

#include <stdint.h>
#include <string>

//Log tags
#define LOG_DW1000 "DW1000"
#define LOG_DW1000_MSG "DW1000_MSG"

namespace m_log
{
    void setup();

    void log_err(std::string const& tag, const char* msg, ...);
    void log_war(std::string const& tag, const char* msg, ...);
    void log_inf(std::string const& tag, const char* msg, ...);
    void log_dbg(std::string const& tag, const char* msg, ...);
    void log_vrb(std::string const& tag, const char* msg, ...);
};
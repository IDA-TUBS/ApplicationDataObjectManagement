#ifndef ADOM_LOG_h
#define ADOM_LOG_h


#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/setup/settings.hpp>
#include <boost/log/attributes/timer.hpp>
#include <boost/log/attributes/named_scope.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/log/attributes/clock.hpp>
#include <boost/log/support/date_time.hpp>


#include <adom/parameters.h>

#include <cstdlib>
#include <iostream>

/**
 * @brief Preprocessor wrapper for boost logging library
 * 
 */
#ifdef LOG_ON


#ifndef LOG_LEVEL
#define LOG_LEVEL 2 // Default to INFO
#endif

#define LOG_LEVEL_TRACE 0
#define LOG_LEVEL_DEBUG 1
#define LOG_LEVEL_INFO 2
#define LOG_LEVEL_WARNING 3
#define LOG_LEVEL_ERROR 4
#define LOG_LEVEL_FATAL 5


#if LOG_LEVEL <= LOG_LEVEL_TRACE
#define logTrace(msg) BOOST_LOG_TRIVIAL(trace) << msg;
#else
#define logTrace(msg)
#endif

#if LOG_LEVEL <= LOG_LEVEL_DEBUG
#define logDebug(msg) BOOST_LOG_TRIVIAL(debug) << msg;
#else
#define logDebug(msg)
#endif

#if LOG_LEVEL <= LOG_LEVEL_INFO
#define logInfo(msg) BOOST_LOG_TRIVIAL(info) << msg;
#else
#define logInfo(msg)
#endif

#if LOG_LEVEL <= LOG_LEVEL_WARNING
#define logWarning(msg) BOOST_LOG_TRIVIAL(warning) << msg;
#else
#define logWarning(msg)
#endif

#if LOG_LEVEL <= LOG_LEVEL_ERROR
#define logError(msg) BOOST_LOG_TRIVIAL(error) << msg;
#else
#define logError(msg)
#endif

#if LOG_LEVEL <= LOG_LEVEL_FATAL
#define logFatal(msg) BOOST_LOG_TRIVIAL(fatal) << msg;
#else
#define logFatal(msg)
#endif

#define AppLog(msg) logDebug(msg)

#else
#define logTrace(msg)
#define logDebug(msg)
#define logInfo(msg)
#define logWarning(msg)
#define logError(msg)
#define logFatal(msg)
#define AppLog(msg)
#endif

namespace adom {

void init_file_log(std::string log_prefix, std::string log_suffix);

void init_console_log();

};

#endif
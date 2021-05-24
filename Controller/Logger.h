#pragma once

#include "Arduino.h"
#include <stdarg.h>

//#define LOGGER
#define TRACE_LINE_ONLY				1
#define TRACE_FUNC_ONLY				2
#define TRACE_PRETTY_FUNC_ONLY		3
#define TRACE_LINE_AND_FUNC			4
#define TRACE_LINE_AND_PRETTY_FUNC	5


#ifndef BUFFER_SIZE
#define BUFFER_SIZE	256
#endif

#ifdef LOGGER

#ifdef LOG_ENABLE_TRACE

#if LOG_ENABLE_TRACE == TRACE_LINE_ONLY
static const __FlashStringHelper * logFormat = F("(%lu) ");
#define __TRACE			.printf(logFormat, __LINE__)

#elif LOG_ENABLE_TRACE == TRACE_FUNC_ONLY
static const __FlashStringHelper * logFormat = F("(%s) ");
#define __TRACE			.printf(logFormat, __FUNCTION__)

#elif LOG_ENABLE_TRACE == TRACE_PRETTY_FUNC_ONLY
static const __FlashStringHelper * logFormat = F("(%s) ");
#define __TRACE			.printf(logFormat, __PRETTY_FUNCTION__)

#elif LOG_ENABLE_TRACE == TRACE_LINE_AND_FUNC
static const __FlashStringHelper * logFormat = F("(%lu, %s) ");
#define __TRACE			.printf(logFormat, __LINE__, __FUNCTION__)

#elif LOG_ENABLE_TRACE == TRACE_LINE_AND_PRETTY_FUNC
static const __FlashStringHelper * logFormat = F("(%lu, %s) ");
#define __TRACE			.printf(logFormat, __LINE__, __PRETTY_FUNCTION__)

#endif

#else
#define __TRACE
#endif

#define END						end()

#define VERBOSEF(fmt,...)		logger.verbose()__TRACE.printf(F(fmt),__VA_ARGS__).END
#define DEBUGF(fmt,...)			logger.debug()__TRACE.printf(F(fmt),__VA_ARGS__).END
#define INFOF(fmt,...)			logger.info()__TRACE.printf(F(fmt),__VA_ARGS__).END
#define WARNF(fmt,...)			logger.warn()__TRACE.printf(F(fmt),__VA_ARGS__).END
#define ERRORF(fmt,...)			logger.error()__TRACE.printf(F(fmt),__VA_ARGS__).END

#define VERBOSE(fmt)			logger.verbose()__TRACE.printf(F(fmt)).END
#define DEBUG(fmt)				logger.debug()__TRACE.printf(F(fmt)).END
#define INFO(fmt)				logger.info()__TRACE.printf(F(fmt)).END
#define WARN(fmt)				logger.warn()__TRACE.printf(F(fmt)).END
#define ERROR(fmt)				logger.error()__TRACE.printf(F(fmt)).END

#define DUMP(var) {logger.debug().print(#var); logger.print("="); logger.print(var); logger.end();}
#define TRACE {DEBUGF(("%s[%lu]"),__PRETTY_FUNCTION__, __LINE__);}

#else

#define __TRACE
#define VERBOSEF(fmt,...)
#define DEBUGF(fmt,...)
#define INFOF(fmt,...)
#define WARNF(fmt,...)
#define ERRORF(fmt,...)

#define VERBOSE(fmt)
#define DEBUG(fmt)
#define INFO(fmt)
#define WARN(fmt)
#define ERROR(fmt)

#define DUMP(var)
#define TRACE
#endif

enum LogLevels : uint8_t
{
	Verbose = 0,
	Debug,
	Info,
	Warn,
	Error
};

class Logger : public Print
{
private:
	uint8_t currentIndex = 0;
	LogLevels currentLevel = LogLevels::Verbose;
	LogLevels setLevel = LogLevels::Verbose;
	bool isEnded = false;
	char textBuffer[BUFFER_SIZE];

	virtual size_t write(uint8_t ch) override;

	void flush();

	void pushHex(uint8_t number);
	void pushHex16(uint16_t number);

	void convertFlashStringToCharArray(const __FlashStringHelper * src, char * dest, size_t size);

	Logger & begin(const LogLevels level);
public:

	void(*flushPipe)(const char * buffer, const uint8_t length);
	void setLogLevel(const LogLevels level);
	Logger & printArrayHex(const uint8_t * arr, uint8_t length);
	Logger & printArrayHex16(const uint16_t * arr, uint8_t length);
	Logger & printBool(const bool value);
	Logger & printf(const char * fmt, ...);
	Logger & printf(const __FlashStringHelper * fmt, ...);

	Logger & verbose();
	Logger & debug();
	Logger & info();
	Logger & warn();
	Logger & error();
	void end();
};

extern Logger logger;
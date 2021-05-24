#include "Logger.h"

size_t Logger::write(uint8_t ch) {
	// reserve 1 byte for \0
	if (currentIndex >= sizeof(textBuffer) - 1) {
		textBuffer[sizeof(textBuffer) - 1] = '\0';
		return 0;
	}
	textBuffer[currentIndex++] = ch;
	return 1;
}

void Logger::flush() {
  if (flushPipe)
	flushPipe(textBuffer, currentIndex - 1);
  else
	Serial.write((uint8_t*)textBuffer, currentIndex-1);
}

void Logger::pushHex(uint8_t number) {
	/*const char hexChar[] = "0123456789ABCDEF";
	print(hexChar[(number >> 4) & 0x0F]);
	print(hexChar[number & 0x0F]);*/
	this->printf("%02X", number);
}

void Logger::pushHex16(uint16_t number) {
	/*const char hexChar[] = "0123456789ABCDEF";
	print(hexChar[(number >> 12) & 0x0F]);
	print(hexChar[(number >> 8) & 0x0F]);
	print(hexChar[(number >> 4) & 0x0F]);
	print(hexChar[number & 0x0F]);*/
	this->printf("%04X", number);
}

void Logger::convertFlashStringToCharArray(const __FlashStringHelper * src, char * dest, size_t size) {
	const char PROGMEM *p = (const char PROGMEM *)src;
	size_t n = 0;
	while (1) {
		unsigned char c = pgm_read_byte(p++);
		if (n > size)
			break;
		dest[n++] = c;
		if (c == '\0')
			break;
	}
}

void Logger::setLogLevel(const LogLevels level) {
	setLevel = level;
}

Logger & Logger::begin(const LogLevels level) {
	if (!isEnded)
		end();

	currentLevel = level;
	if (currentLevel < setLevel)
		return *this;

	isEnded = false;
	currentIndex = 0;

	uint32_t currentMs = millis();
	uint32_t currentSec = currentMs / 1000;

	uint32_t dd = currentSec / 86400;
	uint32_t hh = (currentSec - dd * 86400) / 3600;
	uint32_t mm = (currentSec - dd * 86400 - hh * 3600) / 60;
	uint32_t ss = currentSec % 60;
	uint32_t ms = (currentMs - currentSec * 1000) / 10;

	char type = '?';

	switch (level) {
	case Verbose:
		type = 'V';
		break;
	case Debug:
		type = 'D';
		break;
	case Info:
		type = 'I';
		break;
	case Warn:
		type = 'W';
		break;
	case Error:
		type = 'E';
		break;
	}

	//printf(F("[%d-%02d:%02d:%02d.%02d] -%c: "), dd, hh, mm, ss, ms, type);
	printf(F("[%02lu:%02lu:%02lu.%02lu] -%c: "), hh, mm, ss, ms, type);

	return *this;
}

Logger & Logger::printArrayHex(const uint8_t * arr, uint8_t length) {
	if (currentLevel < setLevel)
		return *this;
	if (length == 0)
		return *this;

	print(F("0x "));

	uint8_t i = 0;
	for (; i < length; ++i) {
		pushHex(arr[i]);
		print(' ');
	}

	return *this;
}

Logger & Logger::printArrayHex16(const uint16_t * arr, uint8_t length) {
	if (currentLevel < setLevel)
		return *this;

	if (length == 0)
		return *this;

	print(F("0x "));

	uint8_t i = 0;
	for (; i < length; ++i) {
		pushHex16(arr[i]);
		print(' ');
	}

	return *this;
}

Logger & Logger::printBool(const bool value) {
	if (currentLevel < setLevel)
		return *this;

	print(value ? "true" : "false");
	return *this;
}

Logger & Logger::printf(const char * fmt, ...) {
	if (currentLevel < setLevel)
		return *this;

	char buffer[BUFFER_SIZE];

	va_list arglist;
	va_start(arglist, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, arglist);
	va_end(arglist);

	for (uint8_t i = 0; i < sizeof(buffer); ++i) {
		if (buffer[i] == '\0')
			break;
		write(buffer[i]);
	}

	return *this;
}

Logger & Logger::printf(const __FlashStringHelper * fmt, ...) {
	if (currentLevel < setLevel)
		return *this;

	char format[BUFFER_SIZE];
	char buffer[BUFFER_SIZE];
	convertFlashStringToCharArray(fmt, format, sizeof(format));

	va_list arglist;
	va_start(arglist, fmt);
	vsnprintf(buffer, sizeof(buffer), format, arglist);
	va_end(arglist);

	for (uint8_t i = 0; i < sizeof(buffer); ++i) {
		if (buffer[i] == '\0')
			break;
		write(buffer[i]);
	}

	return *this;
}

Logger & Logger::verbose() {
	return begin(Verbose);
}

Logger & Logger::debug() {
	return begin(Debug);
}

Logger & Logger::info() {
	return begin(Info);
}

Logger & Logger::warn() {
	return begin(Warn);
}

Logger & Logger::error() {
	return begin(Error);
}

void Logger::end() {
	write('\r');
	write('\n');
	write('\0');
	isEnded = true;
	if (currentLevel >= setLevel)
		flush();
}

Logger logger;
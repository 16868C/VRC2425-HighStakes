#include "16868C/util/logger.hpp"
#include <cstdarg>
#include <cstdio>

using namespace lib16868C;

#define DEBUG
#define ERROR

void printDebug(std::string fmt...) {
	#ifdef DEBUG
	va_list args;
	printf(fmt.c_str(), args);
	#endif
}
void printError(std::string fmt...) {
	#ifdef ERROR
	va_list args;
	fprintf(stderr, fmt.c_str(), args);
	#endif
}
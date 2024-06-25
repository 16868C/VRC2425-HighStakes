#pragma once
#include <fstream>
#include <initializer_list>
#include <string>
#include <vector>
#include "pros/rtos.hpp"

#define DEBUG
#define ERROR

namespace lib16868C {
template<typename T> T Argument(T value) {
	return value;
}
template<typename T> T const* Argument(std::basic_string<T> const& value) {
	return value.c_str();
}

template<typename ... Args> void print(const char* fmt, Args ... args) {
	#ifdef DEBUG
	printf(fmt, Argument(args) ...);
	fflush(stdout);
	#endif
}
template<typename ... Args> void printError(const char* fmt, Args ... args) {
	#ifdef DEBUG
	fprintf(stderr, fmt, Argument(args) ...);
	#endif
}

class Logger {
	public:
		Logger(const std::string file, std::initializer_list<std::string> cols, uint writeInterval = 100);
		~Logger();

		void setData(std::string col, std::string value);
		std::string getData(std::string col);
		std::vector<std::pair<std::string, std::string>>* getAllData();

		std::string getFileName();
		std::ofstream& getFileOutput();

		void setWriteInterval(uint writeInterval);
		uint getWriteInterval();

	private:
		std::string fileName;
		std::ofstream fileOut;

		std::vector<std::pair<std::string, std::string>> data;

		uint writeInterval { 100 };

		pros::Mutex dataMutex;

		pros::Task writeTask = NULL;
		static void writeThread(void* params);

		static std::vector<std::pair<std::string, std::string>>::iterator find(std::vector<std::pair<std::string, std::string>> v, std::string key);
		static bool contains(std::vector<std::pair<std::string, std::string>> v, std::string key);
};
} // namespace lib16868C
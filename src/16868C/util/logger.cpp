#include "logger.hpp"
#include <algorithm>

using namespace lib16868C;

Logger::Logger(const std::string file, std::initializer_list<std::string> cols, uint writeInterval) {
	fileName = file;
	if (std::find(file.begin(), file.end(), ".csv") == file.end())
		fileName += ".csv";

	fileOut = std::ofstream(file);
	fileOut << "time,";
	for (std::string c : cols) {
		// Headers
		fileOut << c << ",";

		// Adding the columns to the map
		if (!contains(data, c)) {
			data.emplace_back(c, "");
		}
	}
	fileOut.flush();

	this->writeInterval = writeInterval;

	writeTask = pros::Task(writeThread, this, TASK_PRIORITY_MIN, TASK_STACK_DEPTH_DEFAULT, ("Logger::" + fileName).c_str());
}
Logger::~Logger() {
	fileOut.flush();
	fileOut.close();

	writeTask.remove();
	writeTask = NULL;
}

void Logger::setData(std::string col, std::string value) {
	if (!dataMutex.take(100)) {
		printError("[Logger::%s] Mutex timeout - unable to update data", fileName);
		return;
	}
	if (!contains(data, col)) {
		printError("[Logger::%s] Column %s not found - was not initialized in constructor", fileName, col);
		return;
	}

	find(data, col)->second = value;
	
	dataMutex.give();
}
std::string Logger::getData(std::string col) {
	if (!dataMutex.take(100)) {
		printError("[Logger::%s] Mutex timeout - unable to update data", fileName);
		return NULL;
	}
	if (!contains(data, col)) {
		printError("[Logger::%s] Column %s could not be found, was not initialized in constructor", fileName, col);
		return NULL;
	}

	std::string ret = find(data, col)->second;
	dataMutex.give();
	return ret;
}
std::vector<std::pair<std::string, std::string>>* Logger::getAllData() {
	if (!dataMutex.take(100)) {
		printError("[Logger::%s] Mutex timeout - unable to update data", fileName);
		return nullptr;
	}

	std::vector<std::pair<std::string, std::string>>* ret = &data;
	dataMutex.give();
	return ret;
}

std::string Logger::getFileName() {
	return fileName;
}
std::ofstream& Logger::getFileOutput() {
	return fileOut;
}

void Logger::setWriteInterval(uint writeInterval) {
	this->writeInterval = writeInterval;
}
uint Logger::getWriteInterval() {
	return writeInterval;
}

void Logger::writeThread(void* params) {
	Logger* logger = (Logger*) params;

	uint32_t time = pros::millis();
	while (true) {
		if (!logger->dataMutex.take(100)) {
			printError("[Logger::%s] Mutex timeout - unable to update data", logger->fileName);
			return;
		}

		// Output data to file
		logger->fileOut << pros::millis() - time << ",";
		for (auto it = logger->data.begin(); it != logger->data.end(); it++) {
			logger->fileOut << it->second << ",";
		}
		logger->fileOut.flush();

		logger->dataMutex.give();

		pros::Task::delay_until(&time, logger->writeInterval);
	}
}

std::vector<std::pair<std::string, std::string>>::iterator find(std::vector<std::pair<std::string, std::string>> v, std::string key) {
	return std::find_if(v.begin(), v.end(),
					[&key](const std::pair<std::string, std::string>& e) { return e.first == key; });
}
bool contains(const std::vector<std::pair<std::string, std::string>> v, std::string key) {
	return find(v, key) != v.end();
}
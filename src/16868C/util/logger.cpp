#include "logger.hpp"
#include <algorithm>
#include <initializer_list>

using namespace lib16868C;

Logger::Logger(const std::string file) {
	fileName = file;
	if (file.find_last_of( ".") == file.size()) // No extension specified
		fileExt = ".txt";
	fileExt = file.substr(file.find_last_of("."));

	fileOut = std::ofstream("/usd/" + fileName + fileExt);
}
Logger::~Logger() {
	fileOut.flush();
	fileOut.close();
}

std::string Logger::getFileName() {
	return fileName;
}
std::string Logger::getFileExt() {
	return fileExt;
}
std::ofstream& Logger::getFileOutput() {
	return fileOut;
}

CSVLogger::CSVLogger(std::string file, std::initializer_list<std::string> cols, uint writeInterval)
	: Logger(file.find_last_of(".") == file.size() ? file + ".csv" : file) {
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
CSVLogger::~CSVLogger() {
	writeTask.remove();
	writeTask = NULL;
}

void CSVLogger::setData(std::string col, std::string value) {
	if (!dataMutex.take(100)) {
		printError("[CSVLogger::%s] Mutex timeout - unable to update data", fileName);
		return;
	}
	if (!contains(data, col)) {
		printError("[CSVLogger::%s] Column %s not found - was not initialized in constructor", fileName, col);
		return;
	}

	find(data, col)->second = value;
	
	dataMutex.give();
}
std::string CSVLogger::getData(std::string col) {
	if (!dataMutex.take(100)) {
		printError("[CSVLogger::%s] Mutex timeout - unable to update data", fileName);
		return NULL;
	}
	if (!contains(data, col)) {
		printError("[CSVLogger::%s] Column %s could not be found, was not initialized in constructor", fileName, col);
		return NULL;
	}

	std::string ret = find(data, col)->second;
	dataMutex.give();
	return ret;
}
std::vector<std::pair<std::string, std::string>>* CSVLogger::getAllData() {
	if (fileExt != ".csv") {
		printError("[CSVLogger::%s] Incompatible file type - getAllData() cannot be called on a non-CSV file.", fileName);
		return NULL;
	}
	if (!dataMutex.take(100)) {
		printError("[CSVLogger::%s] Mutex timeout - unable to update data", fileName);
		return nullptr;
	}

	std::vector<std::pair<std::string, std::string>>* ret = &data;
	dataMutex.give();
	return ret;
}

void CSVLogger::setWriteInterval(uint writeInterval) {
	this->writeInterval = writeInterval;
}
uint CSVLogger::getWriteInterval() {
	return writeInterval;
}

void CSVLogger::writeThread(void* params) {
	CSVLogger* logger = (CSVLogger*) params;

	uint32_t time = pros::millis();
	while (true) {
		if (!logger->dataMutex.take(100)) {
			printError("[CSVLogger::%s] Mutex timeout - unable to update data", logger->fileName);
			return;
		}

		// Output data to file
		logger->fileOut << pros::millis() - time << ",";
		for (auto it = logger->data.begin(); it != logger->data.end(); it++) {
			logger->fileOut << it->second << ",";
		}
		logger->fileOut << "\n";
		logger->fileOut.flush();

		logger->dataMutex.give();

		pros::Task::delay_until(&time, logger->writeInterval);
	}
}

std::vector<std::pair<std::string, std::string>>::iterator CSVLogger::find(std::vector<std::pair<std::string, std::string>> v, std::string key) {
	return std::find_if(v.begin(), v.end(),
					[&key](const std::pair<std::string, std::string>& e) { return e.first == key; });
}
bool CSVLogger::contains(const std::vector<std::pair<std::string, std::string>> v, std::string key) {
	return find(v, key) != v.end();
}
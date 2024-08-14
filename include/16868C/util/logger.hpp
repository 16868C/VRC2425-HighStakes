#pragma once
#include <fstream>
#include <initializer_list>
#include <string>
#include <vector>
#include "pros/rtos.hpp"

#define DEBUG
#define ERROR

namespace lib16868C {
/**
 * @brief Returns the value passed to the function.
 * 
 * @tparam T The type of the value.
 * @param value The value to be returned.
 * @return The same value that was passed to the function.
 */
template<typename T> T Argument(T value) {
	return value;
}
/**
 * @brief Returns a C-style string representation of the given basic_string.
 * 
 * @tparam T The character type of the basic_string (e.g., char, wchar_t).
 * @param value The std::basic_string to be converted to a C-style string.
 * @return A pointer to a constant character array representing the contents of the basic_string.
 */
template<typename T> T const* Argument(std::basic_string<T> const& value) {
	return value.c_str();
}

/**
 * @brief A standard logger that prints information to a .txt file on the microSD card.
 */
class Logger {
	public:
		/**
		 * @brief Construct a new Logger object with the specified file name
		 * 
		 * @param file The file name and extension
		 */
		Logger(const std::string file);
		/**
		 * @brief Destroy the Logger object
		 */
		~Logger();

		/**
		 * @brief Print a string to the file
		 * 
		 * @tparam Args Multiple arguments allowing for string formatting
		 * @param fmt This is the string that contains the text to be written to stdout.
 				It can optionally contain embedded format tags that are replaced by the 
				values specified in subsequent additional arguments and formatted as requested. 
				Format tags prototype is %[flags][width][.precision][length]specifier
		 * @param args The values to replace the format tags
		 */
		template<typename ... Args> inline void print(const char* fmt, Args ... args) {
			char* fmted;
			char* s;
			// snprintf(fmted, sizeof(fmt), fmt, Argument(args) ...);
			// sprintf(s, "[%s] %s", std::to_string(pros::millis()), fmted);
			// fileOut << s;
			// fileOut.flush();
		}
		
		/**
		 * @brief Gets the file name.
		 * 
		 * @return The file name.
		 */
		std::string getFileName();
		/**
		 * @brief Get the file extension.
		 * 
		 * @return The file extension.
		 */
		std::string getFileExt();

		/**
		 * @brief Gets the file output stream.
		 * 
		 * @return A reference to the output stream.
		 */
		std::ofstream& getFileOutput();

	protected:
		std::string fileName;
		std::string fileExt;
		std::ofstream fileOut;
};

/**
 * @brief Logs data to a csv file on the microSD card on regular intervals.
 */
class CSVLogger : public Logger{
	public:
		/**
		 * @brief Constructs a CSVLogger object with the specified file name, columns, and write interval.
		 * 
		 * @param file The base name of the file to which data will be logged. If the name does not 
		 *				specify an extension, an extension of ".txt" is added.
		 * @param cols An initializer list of strings representing the column names for the CSV file.
		 * @param writeInterval The delay (in milliseconds) between each time data is written to the file.
		 */
		CSVLogger(const std::string file, std::initializer_list<std::string> cols, uint writeInterval = 100);
		/**
		 * @brief Destroy the CSVLogger object
		 */
		~CSVLogger();

		/**
		* @brief Updates the value of a specified column in the logger's data.
		* 
		* @param col The name of the column to be updated.
		* @param value The new value to set for the specified column.
		* 
		* @note The column must be initialized in the constructor. If the column is not found,
		* an error is logged.
		*/
		void setData(std::string col, std::string value);
		/**
		 * @brief Get the data of a specified column in the logger's data.
		 * 
		 * @param col The name of the column.
		 * @return The value of the data.
		 *
		 * @note The column must be initialized in the constructor. If the column is not found,
		 * an error is logged.
		 */
		std::string getData(std::string col);
		/**
		 * @brief Get all the data in a vector of pairs of strings, a key and a value, as a pointer.
		 * 
		 * @return A vector containing the columns' name and its value.
		 */
		std::vector<std::pair<std::string, std::string>>* getAllData();

		/**
		 * @brief Sets the write interval.
		 * 
		 * @param writeInterval The delay (in millisecond) between each write to the file.
		 */
		void setWriteInterval(uint writeInterval);
		/**
		 * @brief Get the write interval.
		 * 
		 * @return The delay (in millisecond) between each write to the file.
		 */
		uint getWriteInterval();

	private:
		std::vector<std::pair<std::string, std::string>> data;

		uint writeInterval { 100 };

		pros::Mutex dataMutex;

		pros::Task writeTask = NULL;
		/**
		 * @brief The thread for periodically writing to the file with the information in the data vector.
		 * 
		 * @param params A pointer to the logger instance the thread is part of.
		 */
		static void writeThread(void* params);

		/**
		 * @brief Finds an element in a vector of pairs based on the first value of the pair.
		 * 
		 * @param v The vector of pairs (each pair consisting of two strings) to be searched.
		 * @param key The string value to search for in the first element of the pairs.
		 * @return An iterator pointing to the first pair in the vector where the first value matches 
		 * the specified key. If no such pair is found, the iterator will be equal to v.end().
		 */
		static std::vector<std::pair<std::string, std::string>>::iterator find(std::vector<std::pair<std::string, std::string>> v, std::string key);
		/**
		 * @brief Checks if a vector of pairs contains a pair with a specified key.
		 * 
		 * @param v The vector of pairs (each pair consisting of two strings) to be searched.
		 * @param key The string value to search for in the first element of the pairs.
		 * @return true if the vector contains a pair with the specified key as the first element.
		 * @return false otherwise
		 */
		static bool contains(std::vector<std::pair<std::string, std::string>> v, std::string key);
};

static Logger debugLog = Logger("debug.txt");
static Logger errorLog = Logger("error.txt");

/**
 * @brief Print a string to the terminal (stdout stream)
 * 
 * @tparam Args Multiple arguments allow for string formatting
 * @param fmt This is the string that contains the text to be written to stdout.
 				It can optionally contain embedded format tags that are replaced by the 
				values specified in subsequent additional arguments and formatted as requested. 
				Format tags prototype is %[flags][width][.precision][length]specifier
 * @param args The values to replace the format tags
 */
template<typename ... Args> inline void print(const char* fmt, Args ... args) {
	#ifdef DEBUG
	printf(fmt, Argument(args) ...);
	debugLog.print(fmt, Argument(args) ...);
	fflush(stdout);
	#endif
}
/**
 * @brief Print a string to the terminal in red (stderr stream)
 * 
 * @tparam Args Multiple arguments allow for string formatting
 * @param fmt This is the string that contains the text to be written to stdout.
 				It can optionally contain embedded format tags that are replaced by the 
				values specified in subsequent additional arguments and formatted as requested. 
				Format tags prototype is %[flags][width][.precision][length]specifier
 * @param args The values to replace the format tags
 */
template<typename ... Args> inline void printError(const char* fmt, Args ... args) {
	#ifdef DEBUG
	fprintf(stderr, fmt, Argument(args) ...);
	errorLog.print(fmt, Argument(args) ...);
	#endif
}
} // namespace lib16868C
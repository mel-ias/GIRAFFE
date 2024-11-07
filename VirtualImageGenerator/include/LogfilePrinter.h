#pragma once

#ifndef LOGFILEPRINTER_H
#define LOGFILEPRINTER_H

#include "stdafx.h"
#include <sstream>
#include <ostream>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <fstream>

/**
 * @class LogFile
 * @brief This class handles logging operations, including appending log entries
 *        to an internal log buffer and printing or saving the log to disk.
 *
 * It provides methods for appending log messages in different formats (string,
 * stringstream, or C-style string) with a specified precision, as well as saving
 * the log content to a file.
 */
class LogFile {

public:
	/** Default constructor */
	LogFile() {	}

	/** Default destructor */
	~LogFile() { }

	/**
	* @brief Saves the entire log content to a specified file on disk.
	*
	* This function writes the contents of the internal `logfile_all` stringstream
	* to a file located in the provided directory. The file is overwritten.
	*
	* @param _workingDir The directory where the log file will be saved.
	*/
	void print_content_disk(std::string path_working_directory) {		
		std::ofstream datFile (path_working_directory); // Open the file in write mode, overwriting any existing content
		datFile << _logfile.rdbuf(); // Write the entire content of the log to the file
		datFile.close(); // Close the file after writing
	}

	/**
     * @brief Appends a stringstream log entry to the internal log buffer and prints it.
     * 
     * This function appends the content of a `std::stringstream` to the log buffer (`logfile_all`),
     * prints the content to the console, and ensures that numerical values are printed with a 
     * specified precision.
     * 
     * @param logfile The stringstream containing the log message.
     * @param precision The precision to be used for numeric values (default is 3).
     */
	void append(const std::stringstream& logfile, int precision = 3) {
		_logfile << std::setprecision(precision) << logfile.str() << std::endl; // Set the precision for numerical output
		std::cout << std::fixed << logfile.str() << std::endl;  // Print the log entry to the console
	}
	
	/**
	* @brief Appends a string log entry to the internal log buffer and prints it.
	*
	* This function appends a `std::string` log message to the log buffer (`logfile_all`),
	* prints the message to the console, and ensures numerical values are printed with
	* a specified precision.
	*
	* @param logfile The log message to append.
	* @param precision The precision to be used for numeric values (default is 3).
	*/
	void append(const std::string& logfile, int precision = 3) {
		_logfile << std::setprecision(precision) << logfile << std::endl;
		std::cout << std::fixed << logfile << std::endl; 
	}

	/**
	 * @brief Appends a C-style string log entry to the internal log buffer and prints it.
	 *
	 * This function appends a `const char*` log message to the log buffer (`logfile_all`),
	 * prints the message to the console, and ensures numerical values are printed with
	 * a specified precision.
	 *
	 * @param logfile The C-style string log message to append.
	 * @param precision The precision to be used for numeric values (default is 3).
	 */
	void append(const char* logfile, int precision = 3) {
		_logfile << std::setprecision(precision) << std::string(logfile) << std::endl;	
		std::cout << std::fixed << std::string(logfile) << std::endl; 
	}

private:
	std::stringstream _logfile; // The internal stringstream that holds the log content
};

#endif
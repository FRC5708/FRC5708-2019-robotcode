#pragma once
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <sstream>

class Logger{
public:
	std::string name;
	std::fstream output_file;
	Logger(std::string name="NONE",std::string output_filepath="");
	~Logger();
	template <class T>
	void log(T message);
	template <class T>
	Logger& operator<<(T message);

private:

	bool endedLine();
	void logTime();
};

template <class T>
Logger& Logger::operator<<(T message) {
	if (endedLine()) logTime();
	output_file << message;
	return *this;
}
template <class T>
void Logger::log(T message){
	logTime();
	output_file << message << std::endl;
}
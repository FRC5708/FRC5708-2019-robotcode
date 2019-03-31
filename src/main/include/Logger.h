#pragma once
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <sstream>

class Logger{
public:
	std::string name;
	std::ofstream output_file;
	Logger(std::string name="NONE",std::string output_filepath="");
	~Logger();
	void log(const char* message);
};

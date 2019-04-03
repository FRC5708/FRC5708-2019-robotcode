#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <sstream>
#include "Logger.h"

using namespace std;
Logger::Logger(string name,string output_filepath){
	this->name=name;
	time_t rawtime; time (&rawtime); struct tm *tm_struct = localtime(&rawtime);
	if(output_filepath==""){
		stringstream output;
		output <<  "/home/lvuser/Logging/Log_" << name <<  ':' << (tm_struct->tm_hour) << ':' << (tm_struct->tm_min) << ':' << (tm_struct->tm_sec) << ".txt";
		output_filepath=output.str();
	}
	this->output_file.open(output_filepath,fstream::out | fstream::in | fstream::trunc);
}
void Logger::logTime() {
	time_t rawtime; time (&rawtime); struct tm *tm_struct = localtime(&rawtime);
	output_file << tm_struct->tm_hour << ':' << tm_struct->tm_min << ':' << tm_struct->tm_sec << ' ';
}
bool Logger::endedLine() {

	output_file.seekg(output_file.tellp() - streamoff(1));
	int lastChar = output_file.get();

	if (lastChar == '\n') return true;
	// needed so stream doesn't get stuck in bad state
	else if (lastChar == -1) output_file.clear(); // clears state, not contents

	return false;
}

Logger::~Logger(){
	output_file.close();
}


#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "logical_expressions.h"
#include "rddl.h"
#include "utils/system_utils.h"
#include "utils/timer.h"
#include "RDDLParser.h"


bool checkExtension(std::string s) {
    return ((s.length() > 5) &&  (s.substr(s.length() - 5).compare(".rddl") == 0));
}

int main (int argc, char** argv) {
    Timer t;
    std::cout << "Parsing..." << std::endl;
    if (argc < 3) {
        SystemUtils::abort("Usage: ./rddl_parse <rddlDesc> <targetDir> [options]\n"
                           "where rddlDesc consists of 1-3 individual files");
    }

    // Find input files and combine them in one file
    //std::stringstream combined;
    unsigned int index = 1;
    std::vector<std::string> files(3, "");

    while (index < argc && checkExtension(argv[index])) {
        //std::ifstream ifs(argv[index], std::ifstream::in);
        //combined << ifs.rdbuf();
        //ifs.close();
        files[index-1] = argv[index]; // Store them in a vector
        index++;
    }
    if (index == 1 || index > 4 || index >= argc) {
        SystemUtils::abort("Usage: ./rddl_parse <rddlDesc> <targetDir> [options]\n"
                           "where rddlDesc consists of 1-3 individual files");
    }

    std::string targetDir = std::string(argv[index++]);

    double seed = time(nullptr);
    int numStates = 250;
    int numSimulations = 25;
    bool useIPC2018Rules = false;

    // Read optionals
    for (; index < argc; ++index) {
        std::string nextOption = std::string(argv[index]);
        if (nextOption == "-s") {
            seed = atoi(std::string(argv[++index]).c_str());
            std::cout << "Setting seed to " << seed << std::endl;
        } else if (nextOption == "-trainingSimulations") {
            numSimulations = atoi(std::string(argv[++index]).c_str());
            std::cout << "Setting number of simulations for training set creation to " << numSimulations << std::endl;
        } else if (nextOption == "-trainingSetSize") {
            numStates = atoi(std::string(argv[++index]).c_str());
            std::cout << "Setting target training set size to " << numStates << std::endl;
        } else if (nextOption == "-ipc2018") {
            useIPC2018Rules = atoi(std::string(argv[++index]).c_str());
            std::cout << "Using IPC 2018 rules: " << useIPC2018Rules << std::endl;
        } else {
            assert(false);
        }
    }

    // Creating RDDLTask object
    //rddlTask = new RDDLTask();

    //yy_scan_string(combined.str().c_str());
    //yyparse();

    RDDLTask *task = RDDLParser::parseRDDLTask(files[0], files[1], files[2]);

    std::cout << "...finished (" << t << ")." << std::endl;

    task->execute(targetDir, seed, numStates, numSimulations, useIPC2018Rules);
    std::cout << "total time: " << t << std::endl;

    return EXIT_SUCCESS;
}
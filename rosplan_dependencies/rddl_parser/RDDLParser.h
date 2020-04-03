//
// Created by gerard on 18/09/18.
//

#ifndef RDDL_PARSER_RDDLPARSER_H
#define RDDL_PARSER_RDDLPARSER_H

#include <string>
#include <fstream>
#include <sstream>

#include "evaluatables.h"
#include "logical_expressions.h"
#include "rddl.h"

class RDDLParser {
private:
    static void appendFileStream(const std::string& file_path, std::stringstream& buffer);
public:
    /**
     * Parse a domain file and instance (input is the rddlDesc from the usage)
     * @param domainPath Path to the domain RDDL definition
     * @param instancePath Path to the RDDL instance file
     * @param opt Third file for when "rddlDesc consists of 3 individual files"
     * @return
     */
    static RDDLTask* parseRDDLTask(const std::string &domainPath, const std::string &instancePath, const std::string &opt = "");
};


#endif //RDDL_PARSER_RDDLPARSER_H

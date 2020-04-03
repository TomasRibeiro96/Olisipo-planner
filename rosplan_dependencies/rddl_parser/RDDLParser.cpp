//
// Created by gerard on 18/09/18.
//

#include "RDDLParser.h"

extern int rddl_yyparse();
typedef struct yy_buffer_state* YY_BUFFER_STATE;
extern YY_BUFFER_STATE rddl_yy_scan_string(const char * str);
extern RDDLTask* rddlTask;

RDDLTask *RDDLParser::parseRDDLTask(const std::string &domain, const std::string &instance, const std::string &opt) {
    if (rddlTask)  delete rddlTask;

    rddlTask = new RDDLTask();

    std::stringstream combined;
    appendFileStream(domain, combined);
    appendFileStream(instance, combined);
    appendFileStream(opt, combined);

    rddl_yy_scan_string(combined.str().c_str());
    rddl_yyparse();
    return rddlTask;
}

void RDDLParser::appendFileStream(const std::string &file_path, std::stringstream &buffer) {
    if (file_path.size() > 0) {
        std::ifstream ifs(file_path, std::ifstream::in);
        buffer << ifs.rdbuf();
        ifs.close();
    }
}

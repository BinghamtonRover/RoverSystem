//
// Created by berniewong on 2/16/19.
//

#ifndef ROVERSYSTEM_LOGGING_FRAMEWORK_H
#define ROVERSYSTEM_LOGGING_FRAMEWORK_H
#include <log4cplus/logger.h>
#include <log4cplus/loggingmacros.h>
#include <log4cplus/configurator.h>
#include <log4cplus/loglevel.h>
#include <iomanip>

using namespace log4cplus;


class logging_framework {
public:
    void printMessage(char* text);
    void adjustLogLevel(LogLevel a);
    void callPrint(char* text);
    logging_framework();
    void setDebugStatus(bool b);
    bool getDebugStatus();
    void setDebugMessageStatus(bool b);
    bool getDebugMessageStatus();

private:
    Logger logger;
    BasicConfigurator config;
    bool debugStatus;
    bool debugMessagePrintedOnce;
};


#endif //ROVERSYSTEM_LOGGING_FRAMEWORK_H

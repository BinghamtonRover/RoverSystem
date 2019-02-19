//
// Created by berniewong on 2/16/19.
//

#include "logging_framework.h"
using namespace log4cplus;

logging_framework::logging_framework() {
    initialize();
    config.configure();
    adjustLogLevel(TRACE_LOG_LEVEL);
    logger = Logger::getInstance(LOG4CPLUS_TEXT("main"));
    debugStatus = false;
    debugMessagePrintedOnce = true;
}

void logging_framework::printMessage(char* text) {
    LOG4CPLUS_TRACE(logger, text);
    LOG4CPLUS_DEBUG(logger, text);
    LOG4CPLUS_ERROR(logger, text);
    LOG4CPLUS_INFO(logger, text);
    LOG4CPLUS_WARN(logger, text);
}

void logging_framework::adjustLogLevel(LogLevel a) {
    logger.setLogLevel(a);
}

void logging_framework::callPrint(char* text) {
    printMessage(text);
}

void logging_framework::setDebugStatus(bool b) {
    debugStatus = b;
}

bool logging_framework::getDebugStatus() {
    return debugStatus;
}

void logging_framework::setDebugMessageStatus(bool b) {
    debugMessagePrintedOnce = b;
}

bool logging_framework::getDebugMessageStatus() {
    return debugMessagePrintedOnce;
}
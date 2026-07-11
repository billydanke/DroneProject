#include "DiagnosticLogger.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "Config.h"

namespace {
    struct DiagnosticEvent {
        char Text[Config::DIAGNOSTIC_MESSAGE_MAX_LENGTH + 1];
    };

    DiagnosticEvent eventQueue[Config::DIAGNOSTIC_QUEUE_CAPACITY];
    uint8_t queueHead = 0;
    uint8_t queueCount = 0;
    bool serialEnabled = Config::DIAGNOSTIC_SERIAL_ENABLED;

    void Enqueue(const char* message) {
        if (message == nullptr) return;

        // Preserve the newest diagnostics. This is preferable to letting a burst
        // of low-value messages hide the state which caused the user to inspect it.
        if (queueCount == Config::DIAGNOSTIC_QUEUE_CAPACITY) {
            queueHead = (queueHead + 1) % Config::DIAGNOSTIC_QUEUE_CAPACITY;
            queueCount--;
        }

        uint8_t tail = (queueHead + queueCount) % Config::DIAGNOSTIC_QUEUE_CAPACITY;
        strncpy(eventQueue[tail].Text, message, Config::DIAGNOSTIC_MESSAGE_MAX_LENGTH);
        eventQueue[tail].Text[Config::DIAGNOSTIC_MESSAGE_MAX_LENGTH] = '\0';
        queueCount++;
    }
}

void DiagnosticLogger::Log(const char* message) {
    Enqueue(message);

    if (serialEnabled) {
        Serial.println(message);
    }
}

void DiagnosticLogger::Logf(const char* format, ...) {
    char message[Config::DIAGNOSTIC_MESSAGE_MAX_LENGTH + 1];
    va_list arguments;
    va_start(arguments, format);
    vsnprintf(message, sizeof(message), format, arguments);
    va_end(arguments);
    Log(message);
}

void DiagnosticLogger::LogProgress(const char* operation, uint16_t current, uint16_t total) {
    Logf("calibration:%s:%u/%u", operation, current, total);
}

void DiagnosticLogger::SetSerialEnabled(bool enabled) {
    serialEnabled = enabled;
}

bool DiagnosticLogger::IsSerialEnabled() {
    return serialEnabled;
}

bool DiagnosticLogger::Peek(char* message, size_t messageSize) {
    if (queueCount == 0 || message == nullptr || messageSize == 0) return false;

    strncpy(message, eventQueue[queueHead].Text, messageSize - 1);
    message[messageSize - 1] = '\0';
    return true;
}

void DiagnosticLogger::Pop() {
    if (queueCount == 0) return;

    queueHead = (queueHead + 1) % Config::DIAGNOSTIC_QUEUE_CAPACITY;
    queueCount--;
}

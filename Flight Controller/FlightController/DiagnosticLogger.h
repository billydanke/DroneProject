#pragma once

#include <Arduino.h>

// Stores short diagnostic events until the communications layer can transmit them.
// Logging is deliberately limited to a bounded memory copy so it is safe to call
// from the 400 Hz control loop.
class DiagnosticLogger {
    public:
    static void Log(const char* message);
    static void Logf(const char* format, ...);
    static void LogProgress(const char* operation, uint16_t current, uint16_t total);
    static void SetSerialEnabled(bool enabled);
    static bool IsSerialEnabled();

    static bool Peek(char* message, size_t messageSize);
    static void Pop();
};

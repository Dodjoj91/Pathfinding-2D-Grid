#pragma once
#include <iostream>
#include "BaseEnums.h"

#ifdef _DEBUG

#pragma region Log Macros

#pragma region Log Valid Templates

template <typename T>
struct IsValidString {
    static constexpr bool value = false;
};

template <>
struct IsValidString<std::string> {
    static constexpr bool value = true;
};


template <typename T>
struct IsValidLogType {
    static constexpr bool value = false;
};

template <>
struct IsValidLogType<ELogType> {
    static constexpr bool value = true;
};

#pragma endregion

#define LOG(logType, message) \
        static_assert(IsValidLogType<decltype(logType)>::value, "'logType' must be a valid ELogType"); \
        static_assert(IsValidString<decltype(message)>::value, "'message' must be a valid std::string"); \
        std::string logTypeString; \
        switch (logType) \
        { \
            case ELogType::Info: \
                logTypeString = "INFO"; \
                break; \
            case ELogType::Warning: \
                logTypeString = "WARNING"; \
                break; \
            case ELogType::Error: \
                logTypeString = "ERROR"; \
                break; \
            default: \
                logTypeString = "UNKNOWN"; \
                break; \
        } \
        std::cout << "[" << logTypeString << "] " << message << std::endl; \

#pragma endregion


#else
#define LOG(logType, message)
#endif

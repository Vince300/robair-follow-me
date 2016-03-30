#ifndef _UTILS_HPP_
#define _UTILS_HPP_

#include <sstream>
#include <stdexcept>

#include <OpenNI.h>

#define NICHECK(expr) {\
    auto result = (expr);\
    if (result != openni::Status::STATUS_OK) {\
        std::stringstream ss;\
        ss << "In " << __FILE__ << ":" << __LINE__ << " at " << ##expr << ": ";\
        ss << (int)result; throw std::runtime_error(ss.str().c_str());\
    }\
}

#endif

#ifndef LKM_DRIVER_COMMON_HH
#define LKM_DRIVER_COMMON_HH

#include <vector>
#include <cstdint>

#define LKM_DEBUG
#ifndef LKM_DEBUG
#define LKM_DEBUG_FUNC()
#define LKM_DEBUG_PRINTF(fmt, ...)
#define LKM_DEBUG_PRINLN(msg)
#else
#define LKM_DEBUG_FUNC Serial.printf("l%d %s\n", __LINE__, __func__);
#define LKM_DEBUG_PRINTF(fmt, ...) Serial.printf(fmt, __VA_ARGS__);
#define LKM_DEBUG_PRINLN(msg) Serial.println(msg);
#endif

namespace lkm_m5 {

typedef std::vector<uint8_t> Frame;

}

#endif // !LKM_DRIVER_COMMON_HH

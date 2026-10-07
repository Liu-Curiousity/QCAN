#pragma once

#ifdef __cplusplus
inline constexpr const char *HARDWARE_VERSIONS[] = {
    "unknown",
    "QCAN V1.0.0",
};
inline const char *HARDWARE_VERSION = HARDWARE_VERSIONS[0]; // 硬件版本

extern "C" {
#endif

#define SOFTWARE_VERSION        0x0001  // 软件版本, bcd编码, 例如0x1234表示V12.3.4

#define CAN_QUEUE_SIZE	        (64 * NUM_CAN_CHANNEL)
#define TIM2_CLOCK_SPEED		48000000
#define CAN_CLOCK_SPEED			48000000
#define NUM_CAN_CHANNEL			1

#ifdef __cplusplus
}
#endif


/**
 * VC0706 camera driver for STM32F4 (HAL)
 *
 * Ported from the Adafruit VC0706 Arduino library to generic STM32 HAL.
 * Original: Adafruit_VC0706.h / .cpp (BSD license).
 *
 * Wiring (typical):
 *   Camera TX  -> MCU RX (e.g., USART3_RX)
 *   Camera RX  -> MCU TX (e.g., USART3_TX)
 *   GND        -> GND
 *   VCC        -> 5V (check your module), logic is usually 3.3V-tolerant.
 *
 * Usage:
 *   - Initialize your UART Handle (huartX) in CubeMX at e.g. 38400-8N1.
 *   - Call vc0706_begin(&cam, &huartX, 38400);
 *   - vc0706_take_picture(...); vc0706_frame_length(...);
 *   - Read data in chunks with vc0706_read_picture(...).
 *
 * This file is C99 and can be used from C or C++.
 */
#ifndef VC0706_HAL_H
#define VC0706_HAL_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* === Command constants (mirrored from Adafruit library) === */
#define VC0706_RESET                 0x26
#define VC0706_GEN_VERSION           0x11
#define VC0706_SET_PORT              0x24
#define VC0706_READ_FBUF             0x32
#define VC0706_GET_FBUF_LEN          0x34
#define VC0706_FBUF_CTRL             0x36
#define VC0706_DOWNSIZE_CTRL         0x54
#define VC0706_DOWNSIZE_STATUS       0x55
#define VC0706_READ_DATA             0x30
#define VC0706_WRITE_DATA            0x31
#define VC0706_COMM_MOTION_CTRL      0x37
#define VC0706_COMM_MOTION_STATUS    0x38
#define VC0706_COMM_MOTION_DETECTED  0x39
#define VC0706_MOTION_CTRL           0x42
#define VC0706_MOTION_STATUS         0x43
#define VC0706_TVOUT_CTRL            0x44
#define VC0706_OSD_ADD_CHAR          0x45
#define VC0706_SET_ZOOM              0x52
#define VC0706_GET_ZOOM              0x53

/* Frame buffer control */
#define VC0706_STOPCURRENTFRAME      0x00
#define VC0706_STOPNEXTFRAME         0x01
#define VC0706_STEPFRAME             0x02
#define VC0706_RESUMEFRAME           0x03

/* Image sizes (standard + some extended that some modules support) */
#define VC0706_640x480               0x00
#define VC0706_320x240               0x11
#define VC0706_160x120               0x22
#define VC0706_1024x768              0x33
#define VC0706_1280x720              0x44
#define VC0706_1280x960              0x55
#define VC0706_1920x1080             0x66

/* Motion */
#define VC0706_MOTIONCONTROL         0x00
#define VC0706_UARTMOTION            0x01
#define VC0706_ACTIVATEMOTION        0x01

/* Internal buffer & delays (tunable) */
#ifndef VC0706_CAMERABUFFSIZ
#define VC0706_CAMERABUFFSIZ         100
#endif
#ifndef VC0706_CAMERADELAY_MS
#define VC0706_CAMERADELAY_MS        10
#endif

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t  serial_num;
    uint8_t  camerabuff[VC0706_CAMERABUFFSIZ + 1];
    uint8_t  buffer_len;
    uint32_t frameptr;
} vc0706_t;

/* Optional sink/callback type for streaming capture */
typedef size_t (*vc0706_sink_t)(const uint8_t *data, size_t len, void *user);

/* === Top-level API === */
bool     vc0706_begin(vc0706_t *cam, UART_HandleTypeDef *huart, uint32_t baud);
bool     vc0706_reset(vc0706_t *cam);

bool     vc0706_take_picture(vc0706_t *cam);
bool     vc0706_resume_video(vc0706_t *cam);
bool     vc0706_camera_framebuff_ctrl(vc0706_t *cam, uint8_t command);

uint32_t vc0706_frame_length(vc0706_t *cam);

uint8_t  vc0706_available(vc0706_t *cam);
uint8_t* vc0706_read_picture(vc0706_t *cam, uint8_t n, uint16_t timeout_ms);

uint8_t  vc0706_get_image_size(vc0706_t *cam);
bool     vc0706_set_image_size(vc0706_t *cam, uint8_t x);

uint8_t  vc0706_get_downsize(vc0706_t *cam);
bool     vc0706_set_downsize(vc0706_t *cam, uint8_t newsize);

uint8_t  vc0706_get_compression(vc0706_t *cam);
bool     vc0706_set_compression(vc0706_t *cam, uint8_t c);

/* Motion detection helpers */
bool     vc0706_set_motion_detect(vc0706_t *cam, bool flag);
bool     vc0706_motion_detected(vc0706_t *cam);
bool     vc0706_set_motion_status(vc0706_t *cam, uint8_t x, uint8_t d1, uint8_t d2);
uint8_t  vc0706_get_motion_status(vc0706_t *cam, uint8_t x);

/* TV out */
bool     vc0706_tvon(vc0706_t *cam);
bool     vc0706_tvoff(vc0706_t *cam);

/* Zoom/PTZ */
bool     vc0706_set_ptz(vc0706_t *cam, uint16_t wz, uint16_t hz, uint16_t pan, uint16_t tilt);
bool     vc0706_get_ptz(vc0706_t *cam, uint16_t *w, uint16_t *h, uint16_t *wz,
                        uint16_t *hz, uint16_t *pan, uint16_t *tilt);

/* Version string into internal buffer; returns pointer to NUL-terminated string */
char*    vc0706_get_version(vc0706_t *cam);

/* Baud helpers (send command to camera; you must re-init MCU UART yourself after) */
bool     vc0706_cmd_set_baud_9600(vc0706_t *cam);
bool     vc0706_cmd_set_baud_19200(vc0706_t *cam);
bool     vc0706_cmd_set_baud_38400(vc0706_t *cam);
bool     vc0706_cmd_set_baud_57600(vc0706_t *cam);
bool     vc0706_cmd_set_baud_115200(vc0706_t *cam);

/* Convenience: capture current frame and stream it out via a callback in chunks */
bool     vc0706_capture_to_sink(vc0706_t *cam, uint16_t chunk_bytes,
                                vc0706_sink_t sink, void *user,
                                uint32_t *total_out);

/* === Low-level helpers (public in case you need them) === */
bool     vc0706_run_command(vc0706_t *cam, uint8_t cmd, const uint8_t *args,
                            uint8_t argn, uint8_t resplen, bool flush_before);
void     vc0706_send_command(vc0706_t *cam, uint8_t cmd, const uint8_t *args, uint8_t argn);
uint8_t  vc0706_read_response(vc0706_t *cam, uint8_t numbytes, uint16_t timeout_ms);
bool     vc0706_verify_response(vc0706_t *cam, uint8_t command);

#ifdef __cplusplus
}
#endif

#endif /* VC0706_HAL_H */

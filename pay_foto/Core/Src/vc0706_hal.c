
#include "vc0706_hal.h"
#include <string.h>

/* === Internal === */
static void vc0706_common_init(vc0706_t *cam) {
    cam->serial_num = 0;
    cam->frameptr   = 0;
    cam->buffer_len = 0;
}

/* Begin: bind UART and optionally set baud (re-inits HAL UART). */
bool vc0706_begin(vc0706_t *cam, UART_HandleTypeDef *huart, uint32_t baud) {
    if (!cam || !huart) return false;
    vc0706_common_init(cam);
    cam->huart = huart;

    /* Reconfigure UART baud if it differs */
    if (cam->huart->Init.BaudRate != baud) {
        HAL_UART_DeInit(cam->huart);
        cam->huart->Init.BaudRate = baud;
        if (HAL_UART_Init(cam->huart) != HAL_OK) {
            return false;
        }
    }
    return vc0706_reset(cam);
}

bool vc0706_reset(vc0706_t *cam) {
    uint8_t args[] = {0x00};
    return vc0706_run_command(cam, VC0706_RESET, args, 1, 5, true);
}

bool vc0706_motion_detected(vc0706_t *cam) {
    if (vc0706_read_response(cam, 4, 200) != 4) return false;
    if (!vc0706_verify_response(cam, VC0706_COMM_MOTION_DETECTED)) return false;
    return true;
}

bool vc0706_set_motion_status(vc0706_t *cam, uint8_t x, uint8_t d1, uint8_t d2) {
    uint8_t args[] = {0x03, x, d1, d2};
    return vc0706_run_command(cam, VC0706_MOTION_CTRL, args, sizeof(args), 5, true);
}

uint8_t vc0706_get_motion_status(vc0706_t *cam, uint8_t x) {
    uint8_t args[] = {0x01, x};
    return vc0706_run_command(cam, VC0706_MOTION_STATUS, args, sizeof(args), 5, true);
}

bool vc0706_set_motion_detect(vc0706_t *cam, bool flag) {
    if (!vc0706_set_motion_status(cam, VC0706_MOTIONCONTROL, VC0706_UARTMOTION, VC0706_ACTIVATEMOTION))
        return false;
    uint8_t args[] = {0x01, (uint8_t)(flag ? 1 : 0)};
    return vc0706_run_command(cam, VC0706_COMM_MOTION_CTRL, args, sizeof(args), 5, true);
}

uint8_t vc0706_get_image_size(vc0706_t *cam) {
    uint8_t args[] = {0x04, 0x04, 0x01, 0x00, 0x19};
    if (!vc0706_run_command(cam, VC0706_READ_DATA, args, sizeof(args), 6, true))
        return 0xFF;
    return cam->camerabuff[5];
}

bool vc0706_set_image_size(vc0706_t *cam, uint8_t x) {
    uint8_t args[] = {0x05, 0x04, 0x01, 0x00, 0x19, x};
    if (x < VC0706_1024x768) args[1] = 0x04; /* standard */
    else                      args[1] = 0x05; /* extended */
    return vc0706_run_command(cam, VC0706_WRITE_DATA, args, sizeof(args), 5, true);
}

uint8_t vc0706_get_downsize(vc0706_t *cam) {
    uint8_t args[] = {0x00};
    if (!vc0706_run_command(cam, VC0706_DOWNSIZE_STATUS, args, 1, 6, true))
        return 0xFF;
    return cam->camerabuff[5];
}

bool vc0706_set_downsize(vc0706_t *cam, uint8_t newsize) {
    uint8_t args[] = {0x01, newsize};
    return vc0706_run_command(cam, VC0706_DOWNSIZE_CTRL, args, 2, 5, true);
}

char* vc0706_get_version(vc0706_t *cam) {
    uint8_t args[] = {0x01};
    vc0706_send_command(cam, VC0706_GEN_VERSION, args, 1);
    if (!vc0706_read_response(cam, VC0706_CAMERABUFFSIZ, 200))
        return NULL;
    cam->camerabuff[cam->buffer_len] = 0;
    return (char*)cam->camerabuff;
}

bool vc0706_cmd_set_baud_9600(vc0706_t *cam) {
    uint8_t args[] = {0x03, 0x01, 0xAE, 0xC8};
    vc0706_send_command(cam, VC0706_SET_PORT, args, sizeof(args));
    return vc0706_read_response(cam, VC0706_CAMERABUFFSIZ, 200) != 0;
}
bool vc0706_cmd_set_baud_19200(vc0706_t *cam) {
    uint8_t args[] = {0x03, 0x01, 0x56, 0xE4};
    vc0706_send_command(cam, VC0706_SET_PORT, args, sizeof(args));
    return vc0706_read_response(cam, VC0706_CAMERABUFFSIZ, 200) != 0;
}
bool vc0706_cmd_set_baud_38400(vc0706_t *cam) {
    uint8_t args[] = {0x03, 0x01, 0x2A, 0xF2};
    vc0706_send_command(cam, VC0706_SET_PORT, args, sizeof(args));
    return vc0706_read_response(cam, VC0706_CAMERABUFFSIZ, 200) != 0;
}
bool vc0706_cmd_set_baud_57600(vc0706_t *cam) {
    uint8_t args[] = {0x03, 0x01, 0x1C, 0x1C};
    vc0706_send_command(cam, VC0706_SET_PORT, args, sizeof(args));
    return vc0706_read_response(cam, VC0706_CAMERABUFFSIZ, 200) != 0;
}
bool vc0706_cmd_set_baud_115200(vc0706_t *cam) {
    uint8_t args[] = {0x03, 0x01, 0x0D, 0xA6};
    vc0706_send_command(cam, VC0706_SET_PORT, args, sizeof(args));
    return vc0706_read_response(cam, VC0706_CAMERABUFFSIZ, 200) != 0;
}

bool vc0706_set_ptz(vc0706_t *cam, uint16_t wz, uint16_t hz, uint16_t pan, uint16_t tilt) {
    uint8_t args[] = {
        0x08, (uint8_t)(wz >> 8),  (uint8_t)wz,
              (uint8_t)(hz >> 8),  (uint8_t)hz,
              (uint8_t)(pan >> 8), (uint8_t)pan,
              (uint8_t)(tilt >> 8),(uint8_t)tilt
    };
    return vc0706_run_command(cam, VC0706_SET_ZOOM, args, sizeof(args), 5, true);
}

bool vc0706_get_ptz(vc0706_t *cam, uint16_t *w, uint16_t *h, uint16_t *wz,
                    uint16_t *hz, uint16_t *pan, uint16_t *tilt) {
    uint8_t args[] = {0x00};
    if (!vc0706_run_command(cam, VC0706_GET_ZOOM, args, sizeof(args), 16, true))
        return false;

    if (w)   { *w   = ((uint16_t)cam->camerabuff[5]  << 8) | cam->camerabuff[6];  }
    if (h)   { *h   = ((uint16_t)cam->camerabuff[7]  << 8) | cam->camerabuff[8];  }
    if (wz)  { *wz  = ((uint16_t)cam->camerabuff[9]  << 8) | cam->camerabuff[10]; }
    if (hz)  { *hz  = ((uint16_t)cam->camerabuff[11] << 8) | cam->camerabuff[12]; }
    if (pan) { *pan = ((uint16_t)cam->camerabuff[13] << 8) | cam->camerabuff[14]; }
    if (tilt){ *tilt= ((uint16_t)cam->camerabuff[15] << 8) | cam->camerabuff[16]; }
    return true;
}

bool vc0706_tvon(vc0706_t *cam) {
    uint8_t args[] = {0x01, 0x01};
    return vc0706_run_command(cam, VC0706_TVOUT_CTRL, args, sizeof(args), 5, true);
}

bool vc0706_tvoff(vc0706_t *cam) {
    uint8_t args[] = {0x01, 0x00};
    return vc0706_run_command(cam, VC0706_TVOUT_CTRL, args, sizeof(args), 5, true);
}

bool vc0706_set_compression(vc0706_t *cam, uint8_t c) {
    uint8_t args[] = {0x05, 0x01, 0x01, 0x12, 0x04, c};
    return vc0706_run_command(cam, VC0706_WRITE_DATA, args, sizeof(args), 5, true);
}

uint8_t vc0706_get_compression(vc0706_t *cam) {
    uint8_t args[] = {0x04, 0x01, 0x01, 0x12, 0x04};
    vc0706_run_command(cam, VC0706_READ_DATA, args, sizeof(args), 6, true);
    return cam->camerabuff[5];
}

bool vc0706_take_picture(vc0706_t *cam) {
    cam->frameptr = 0;
    return vc0706_camera_framebuff_ctrl(cam, VC0706_STOPCURRENTFRAME);
}

bool vc0706_resume_video(vc0706_t *cam) {
    return vc0706_camera_framebuff_ctrl(cam, VC0706_RESUMEFRAME);
}

bool vc0706_camera_framebuff_ctrl(vc0706_t *cam, uint8_t command) {
    uint8_t args[] = {0x01, command};
    return vc0706_run_command(cam, VC0706_FBUF_CTRL, args, sizeof(args), 5, true);
}

uint32_t vc0706_frame_length(vc0706_t *cam) {
    uint8_t args[] = {0x01, 0x00};
    if (!vc0706_run_command(cam, VC0706_GET_FBUF_LEN, args, sizeof(args), 9, true))
        return 0;
    uint32_t len = 0;
    len  = ((uint32_t)cam->camerabuff[5]  << 24);
    len |= ((uint32_t)cam->camerabuff[6]  << 16);
    len |= ((uint32_t)cam->camerabuff[7]  << 8);
    len |=  (uint32_t)cam->camerabuff[8];
    return len;
}

uint8_t vc0706_available(vc0706_t *cam) {
    return cam->buffer_len;
}

uint8_t* vc0706_read_picture(vc0706_t *cam, uint8_t n, uint16_t timeout_ms) {
    /* Read n bytes starting at frameptr */
    uint8_t args[] = {
        0x0C, 0x00, 0x0A,
        (uint8_t)((cam->frameptr >> 24) & 0xFF),
        (uint8_t)((cam->frameptr >> 16) & 0xFF),
        (uint8_t)((cam->frameptr >> 8)  & 0xFF),
        (uint8_t)(cam->frameptr & 0xFF),
        0, 0, 0,               /* dummy 3 bytes per datasheet */
        n,                     /* number of bytes to read */
        (VC0706_CAMERADELAY_MS >> 8) & 0xFF,
        (VC0706_CAMERADELAY_MS) & 0xFF
    };
    if (!vc0706_run_command(cam, VC0706_READ_FBUF, args, sizeof(args), 5, false))
        return NULL;

    if (vc0706_read_response(cam, n + 5, timeout_ms) == 0)
        return NULL;

    cam->frameptr += n;
    return cam->camerabuff;
}

/* Stream entire captured frame via a callback sink */
bool vc0706_capture_to_sink(vc0706_t *cam, uint16_t chunk_bytes,
                            vc0706_sink_t sink, void *user,
                            uint32_t *total_out) {
    if (!cam || !sink || chunk_bytes == 0) return false;
    if (!vc0706_take_picture(cam)) return false;

    uint32_t total = vc0706_frame_length(cam);
    if (total_out) *total_out = total;

    uint32_t remaining = total;
    while (remaining > 0) {
        uint16_t n = (remaining < chunk_bytes) ? (uint16_t)remaining : chunk_bytes;
        uint8_t *buf = vc0706_read_picture(cam, (uint8_t)n, 500);
        if (!buf) return false;
        size_t wrote = sink(buf + 5, n, user); /* skip 5-byte header */
        if (wrote != n) return false;
        remaining -= n;
    }
    (void)vc0706_resume_video(cam);
    return true;
}

/* === Low-level === */
bool vc0706_run_command(vc0706_t *cam, uint8_t cmd, const uint8_t *args,
                        uint8_t argn, uint8_t resplen, bool flush_before) {
    if (flush_before) {
        vc0706_read_response(cam, 100, 10);
    }
    vc0706_send_command(cam, cmd, args, argn);
    if (vc0706_read_response(cam, resplen, 200) != resplen)
        return false;
    if (!vc0706_verify_response(cam, cmd))
        return false;
    return true;
}

void vc0706_send_command(vc0706_t *cam, uint8_t cmd, const uint8_t *args, uint8_t argn) {
    uint8_t header[3] = {0x56, cam->serial_num, cmd};
    HAL_UART_Transmit(cam->huart, header, sizeof(header), 50);
    if (argn && args) {
        HAL_UART_Transmit(cam->huart, (uint8_t*)args, argn, 50);
    }
}

uint8_t vc0706_read_response(vc0706_t *cam, uint8_t numbytes, uint16_t timeout_ms) {
    uint16_t counter = 0;
    cam->buffer_len = 0;

    while ((counter < timeout_ms) && (cam->buffer_len != numbytes)) {
        uint8_t byte;
        if (HAL_UART_Receive(cam->huart, &byte, 1, 1) == HAL_OK) {
            cam->camerabuff[cam->buffer_len++] = byte;
            counter = 0; /* got a byte, reset "no data" counter */
        } else {
            HAL_Delay(1);
            counter++;
        }
    }
    return cam->buffer_len;
}

bool vc0706_verify_response(vc0706_t *cam, uint8_t command) {
    if (cam->buffer_len < 4) return false;
    if (cam->camerabuff[0] != 0x76) return false;
    if (cam->camerabuff[1] != cam->serial_num) return false;
    if (cam->camerabuff[2] != command) return false;
    if (cam->camerabuff[3] != 0x00) return false;
    return true;
}

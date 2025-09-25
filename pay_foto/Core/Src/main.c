/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "vc0706_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define VC0706_READ_CHUNK 64

#define LINK_UART_HANDLE  huart1          // <--- change to your UART handle (huart3, huart7, etc.)
#define SEND_FILENAME     "im5.JPG"

/* ------------------------ */

#define CHUNK          256u              // multiple of 512 recommended
#define SOF0           0x55
#define SOF1           0xAA
#define ACK            0x06
#define NAK            0x15
#define UART_TMO_MS    2500
#define MAX_RETRY      8

#define UART_TX_TMO_MS  2500

static vc0706_t cam;
extern UART_HandleTypeDef LINK_UART_HANDLE;


//void myprintf(const char *fmt, ...);
static int generar_nombre_unico(char *out, size_t out_sz);
char filename[40];


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


static inline void uart_flush_rx_polling(UART_HandleTypeDef *huart)
{
    uint8_t dump;
    while (HAL_UART_Receive(huart, &dump, 1, 0) == HAL_OK) {
        /* discard */
    }
    /* If an overrun happened, clear it so RX keeps working */
    __HAL_UART_CLEAR_OREFLAG(huart);
}

int camera_init(void) {
    //myprintf("\r\ninicializando camara...\r\n");

    // Bind to USART2 at 38400 (will HAL_UART_Init with that baud if needed)
    if (!vc0706_begin(&cam, &huart2, 38400)) {
        //myprintf("no se detecto la camara o error de comunicacion\r\n");
        return -1;
    }

    // Version probe (good connectivity test)
    char *ver = vc0706_get_version(&cam);
    if (!ver) {
        //myprintf("get_version fallo\r\n");
        return -2;
    }
    // The buffer is NUL-terminated; print a trimmed line
    //myprintf("version: %s\r\n", ver);

    // Set resolution 640x480 (same as cam.setImageSize(VC0706_640x480))
    if (!vc0706_set_image_size(&cam, VC0706_640x480)) {
        //myprintf("no se pudo fijar resolucion 640x480\r\n");
        return -3;
    }
    // Read back to confirm
    uint8_t sz = vc0706_get_image_size(&cam);
    if (sz != VC0706_640x480) {
        //myprintf("aviso: resolucion leida = 0x%02X (esperado 0x00)\r\n", sz);
    } else {
        //myprintf("resolucion establecida a 640x480\r\n");
    }

    // (Opcional) ajustar compresion JPEG: 0x00 (max calidad) .. 0xFF (max compresion)
    // (void)vc0706_set_compression(&cam, 0x36);

    //myprintf("camara lista\r\n");
    return 0;
}

/*
void myprintf(const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart4, (uint8_t*)buffer, len, -1);

}
*/


/* ===== CRC16-CCITT (poly 0x1021, init 0xFFFF) ===== */
static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ===== UART helpers (blocking) ===== */
static int uart_send(const void *p, uint16_t n)
{
    return (HAL_UART_Transmit(&LINK_UART_HANDLE, (uint8_t*)p, n, UART_TX_TMO_MS) == HAL_OK) ? 0 : -1;
}
static int uart_recv(void *p, uint16_t n, uint32_t tmo)
{
    return (HAL_UART_Receive(&LINK_UART_HANDLE, (uint8_t*)p, n, tmo) == HAL_OK) ? 0 : -1;
}

/* ===== Protocol: header + framed data ===== */
static int send_header(uint32_t fsz)
{
    uint8_t hdr[6] = { 'S','Z', (uint8_t)(fsz), (uint8_t)(fsz>>8), (uint8_t)(fsz>>16), (uint8_t)(fsz>>24) };


    if (uart_send(hdr, sizeof(hdr)) != 0) return -1;
    //myprintf("[TX] uart_send end\r\n");

    uint8_t ack = 0;
    if (uart_recv(&ack, 1, UART_TMO_MS) != 0) return -2;
    //myprintf("uart ack = %d \r\n; ");
    //myprintf("[TX] header_ok\r\n");
    //myprintf("El ACK es 0x%02X\r\n", (unsigned)ack);   // Hex con cero a la izquierda
    return (ack == ACK) ? 0 : -3;
}

static int send_frame(uint16_t seq, const uint8_t *data, uint16_t len)
{
	//myprintf("Enviando x4 \r\n");
	HAL_Delay(100);
    static uint8_t tx[CHUNK + 8]; // SOF(2) + seq(2) + len(2) + payload + crc(2)
    tx[0] = SOF0; tx[1] = SOF1;
    tx[2] = (uint8_t)(seq); tx[3] = (uint8_t)(seq >> 8);
    tx[4] = (uint8_t)(len); tx[5] = (uint8_t)(len >> 8);
    memcpy(&tx[6], data, len);
    uint16_t crc = crc16_ccitt(data, len);
    tx[6 + len]     = (uint8_t)(crc);
    tx[6 + len + 1] = (uint8_t)(crc >> 8);

    for (int attempt = 0; attempt < MAX_RETRY; attempt++) {
        if (uart_send(tx, (uint16_t)(len + 8)) != 0) return -10;

        uint8_t dump;
        while (HAL_UART_Receive(&LINK_UART_HANDLE, &dump, 1, 0) == HAL_OK) {
            /* discard */
        }

        uint8_t resp[3];
        if (uart_recv(resp, 3, UART_TMO_MS) == 0 && resp[0] == ACK) {
            uint16_t seq_echo = (uint16_t)resp[1] | ((uint16_t)resp[2] << 8);
            if (seq_echo == seq) return 0;
        }
        // retry on timeout or NAK
    }
    return -11;
}

/* ===== Public API: send a file ===== */
int send_file_over_uart(const char *path)
{
    FRESULT fr;
    FATFS fs;
    FIL   f;
    UINT  br;
    static uint8_t buf[CHUNK];

    // Ensure FatFs is wired to SPI SD in your project
    MX_FATFS_Init();
    fr = f_mount(&fs, "", 1);
    if (fr != FR_OK) return -100;


    fr = f_open(&f, path, FA_READ | FA_OPEN_EXISTING);
    if (fr != FR_OK) { f_mount(NULL, "", 0); return -101; }

    uint32_t fsz = (uint32_t)f_size(&f);
    //myprintf("[TX] uart_send(len=%u) begin\r\n", (unsigned)(br + 8));
    if (send_header(fsz) != 0) { f_close(&f); f_mount(NULL, "", 0); return -102; }


    uint16_t seq = 0;
    uint32_t sent = 0;
    //myprintf("Enviando x2 \r\n");
    //HAL_GPIO_TogglePin(WDG_GPIO_Port, WDG_Pin);

    while (sent < fsz) {
    	HAL_GPIO_TogglePin(WDG_GPIO_Port, WDG_Pin);
    	HAL_Delay(100);
    	//myprintf("Enviando x3 \r\n");

        UINT need = (fsz - sent > CHUNK) ? CHUNK : (UINT)(fsz - sent);
        fr = f_read(&f, buf, need, &br);
        //myprintf("[TX] f_read fr=%d br=%u need=%u\r\n", (int)fr, (unsigned)br, (unsigned)need);
        //if (fr != FR_OK || br == 0) { return -105; }
        if (fr != FR_OK) { f_close(&f); f_mount(NULL, "", 0); return -103; }
        if (br == 0) break;

        //myprintf("[TX] send_frame seq=%u len=%u\r\n", (unsigned)seq, (unsigned)br);
        if (send_frame(seq, buf, (uint16_t)br) != 0) { f_close(&f); f_mount(NULL, "", 0); return -104; }
        sent += br;
        seq++;
    }

    f_close(&f);
    f_mount(NULL, "", 0);
    return 0;
}

/* ===== Button-driven send (polling) =====
 * Requires USER button as input (e.g., Nucleo B1 on PC13).
 */
static volatile bool g_sending  = false;
static bool btn_prev = true; // idle-high on many Nucleo boards
/*

static bool button_pressed_edge(void)
{
    bool now = (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_RESET);
    bool pressed = (btn_prev && !now); // high -> low
    btn_prev = now;
    return pressed;
}
/*




/* ================== END F446ZE SENDER ================== */



/* fixed date like your sketch; swap for RTC later */
static void fecha_yyyymmdd(char *dst) { strcpy(dst, "20250101"); }

/* IMG_YYYYMMDD_NNNNNN.JPG with FatFS existence check */
static int generar_nombre_unico(char *out, size_t out_sz) {
    if (!out || out_sz < 32) return -1;
    char ymd[9]; fecha_yyyymmdd(ymd);
    FILINFO fno;
    for (unsigned i = 0; i < 1000000U; i++) {
        int n = snprintf(out, out_sz, "IMG_%s_%06u.JPG", ymd, i);
        if (n <= 0 || (size_t)n >= out_sz) return -2;
        FRESULT fr = f_stat(out, &fno);
        if (fr == FR_NO_FILE) return 0;      // available
        if (fr != FR_OK && fr != FR_EXIST) return -3;
    }
    return -4;
}

int capturar_imagen_a_sd_manual(vc0706_t *cam) {
    if (!cam) return -1;

    FATFS fs; FIL file; FRESULT fr; UINT bw;

    MX_FATFS_Init();
    fr = f_mount(&fs, "", 1);
    if (fr != FR_OK) { return -100; }

    if (!vc0706_take_picture(cam)) {
        //myprintf("take_picture FAIL\r\n");
        f_mount(NULL, "", 0);
        return -200;
    }

    uint32_t jpglen = vc0706_frame_length(cam);
    if (jpglen == 0) {
        //myprintf("frame_length=0\r\n");
        (void)vc0706_resume_video(cam);
        f_mount(NULL, "", 0);
        return -201;
    }
    //myprintf("size=%lu bytes\r\n", (unsigned long)jpglen);

// FILENAME CHAR FILENAME
    int rc = generar_nombre_unico(filename, sizeof(filename));
    if (rc != 0) {
        //myprintf("name err=%d\r\n", rc);
        (void)vc0706_resume_video(cam);
        f_mount(NULL, "", 0);
        return -202;
    }


    fr = f_open(&file, filename, FA_WRITE | FA_CREATE_NEW);
    if (fr != FR_OK) {
        //myprintf("f_open=%d\r\n", fr);
        (void)vc0706_resume_video(cam);
        f_mount(NULL, "", 0);
        return -203;
    }
    //myprintf("writing %s ...\r\n", filename);

    uint32_t written = 0;
    int payload_offset = -1; // will be 0 or 5 after first chunk

    while (written < jpglen) {
    	HAL_GPIO_TogglePin(WDG_GPIO_Port, WDG_Pin);
        uint32_t remain = jpglen - written;
        uint8_t n = (remain < VC0706_READ_CHUNK ) ? (uint8_t)remain : (uint8_t)VC0706_READ_CHUNK ;
        if (n > 240) n = 240;  // safety: VC0706 count is uint8_t

        uint8_t *buf = vc0706_read_picture(cam, n, 500);   // returns [5-byte header] + n data

        if (!buf) {
            //myprintf("read_picture timeout/null\r\n");
            f_close(&file);
            (void)vc0706_resume_video(cam);
            f_mount(NULL, "", 0);
            return -204;
        }
        if (payload_offset < 0) {
            if (buf[0] == 0xFF && buf[1] == 0xD8) {
                // Looks like JPEG SOI at start => payload already at buf
                payload_offset = 0;
            } else if (buf[0] == 0x76 && buf[2] == 0x32) {
                // 0x76, serial, 0x32 (= READ_FBUF reply) => header present
                payload_offset = 5;
            } else {
                // fallback: if looks like header length >= 5, assume 5; else assume 0
                payload_offset = 5;
            }
        }

        /* write ONLY JPEG payload; skip 5-byte VC0706 header */
        fr = f_write(&file, buf + payload_offset, n, &bw);
        if (fr != FR_OK || bw != n) {
            //myprintf("f_write err=%d bw=%u n=%u\r\n", fr, (unsigned)bw, (unsigned)n);
            f_close(&file);
            (void)vc0706_resume_video(cam);
            f_mount(NULL, "", 0);
            return -205;
        }

        written += n;
        //if ((written % (10*1024)) < VC0706_READ_CHUNK ) myprintf(".");
    }
    //myprintf("\r\n");

    f_sync(&file);
    f_close(&file);
    (void)vc0706_resume_video(cam);
    f_mount(NULL, "", 0);

    //myprintf("done: %s (%lu bytes)\r\n", filename, (unsigned long)written);
    return 0;
}
/*
void user_loop_sender_sd(void)
{
    if (!g_sending && button_pressed_edge()) {
    	//myprintf("Enviando \r\n");
        g_sending = true;
        (void)send_file_over_uart(SEND_FILENAME);
        g_sending = false;
    }
}
*/

/*

void user_loop_sender_uart(int status,bool status16, bool status32)
{
    //if (!g_sending && button_pressed_edge()) {
    if (status==1 && status16==true && status32==true) {
    	//myprintf("Enviando \r\n");
        g_sending = true;
        (void)send_file_over_uart(SEND_FILENAME);
        g_sending = false;
    }else{
    	//myprintf("Comando mal recibido \n");
    }
}
*/


/*
void user_loop_sender_cam_sd(void)
{
    if (!g_sending && button_pressed_edge()) {
        g_sending = true;

        char filename[40];
        if (generar_nombre_unico(filename, sizeof(filename)) != 0) {
            //myprintf("name gen failed\r\n");
            g_sending = false;
            return;
        }

        //myprintf("Capturando y guardando: %s\r\n", filename);
        int rc = capturar_imagen_a_sd_manual(&cam);
        if (rc != 0) {
            //myprintf("capture/save failed rc=%d\r\n", rc);
            g_sending = false;
            return;
        }

        //myprintf("Enviando por UART: %s\r\n", filename);

        HAL_Delay(500);
        (void)send_file_over_uart(filename);
        g_sending = false;



    }
}
*/


void envia_defrente(void)
{

	HAL_GPIO_TogglePin(WDG_GPIO_Port, WDG_Pin);

	/* COMENTADO HHABIA CHAR FILENAME
	if (generar_nombre_unico(filename, sizeof(filename)) != 0) {
		//myprintf("name gen failed\r\n");
		return;
	}
	*/

	//myprintf("Capturando y guardando: %s\r\n", filename);
	int rc = capturar_imagen_a_sd_manual(&cam);
	if (rc != 0) {
		//myprintf("capture/save failed rc=%d\r\n", rc);
		return;
	}


	//HAL_Delay(2000);
	//(void)send_file_over_uart(filename);

}




/* **************************************** Programacion de comandos ********************************************/

/************************************************
 Declaracion de cabeceras, payload y variables
 ************************************************/

uint8_t TA=0x05; /*TA=TargetAddress*/
uint8_t SA=0x88; /*SA=SourceAddress*/

/*PPID=PayloadProtocolID
(Define las funciones
 Toma de foto camara 1: id=0x01
 Toma de foto camara 2: id=0x02)*/
uint8_t PPID;

/*PS=PayloadSize (Tamaño del payload)
 PayloadComando= 1-2bytes
 PayloadData= 1-255bytes*/
uint8_t PS;

/*Payload*/
uint8_t payload_total[255];
uint8_t payload_envio[]={};

/*CRC32*/
uint32_t CRC32;

/*Variable*/
uint8_t count;
uint8_t val;


/**************************************
 * Generacion PS (payload size)
 ***************************************/


uint8_t countPay(uint8_t* payload_envio){
	/*Define la cantidad de bytes (size) del payload*/
	  for (int i=0; i<255; i++){
		  val = payload_envio[i] & 0b11111111;
		  if (val != 0x00){
			  count=count+1;
		  }
	  }

	  /*cumple con darnos la cantidad de bytes(size) del payload*/
	  PS=count;
	  return PS;
}


/****************************************
 * Funciones generan CRC16 y CRC32
 ***************************************/

bool gener_crc16(uint8_t *cabecera,uint8_t size_cabecera,uint8_t *cab){

		    uint16_t G_X = 0x1021; // Polynomial generator
		    uint16_t CRC16 = 0xFFFF;
		    uint8_t InvArreglo[4] = {};
		    uint8_t Temp;
		    uint8_t InvBits[4] ={}; //Arreglo de bits invertidos de la cabecera  (LSB first)
		    uint8_t Fin=0x00;

		    //Se invierten los elementos del array
		    for(int i=0; i<size_cabecera; i++){
		    	InvArreglo[i]=cabecera[size_cabecera-i-1];
		    }

		    //Se invierten los bits de los elementos del array
		    for(int j=0; j<size_cabecera;j++){
		    	for(int i=0; i<8; i++){
		    		Temp=InvArreglo[j]&(0x80);
		    		//InvBits[0]=InvArreglo[1];
		    		//HAL_UART_Transmit(&huart2,InvBits,sizeof(InvBits),100);// Sending in normal mode
		    		//HAL_Delay(1000);
		    		if(Temp!=0x00){
		    			Fin=(Fin>>1);
		    			Fin=Fin|(0x80);
		    			InvArreglo[j]=(InvArreglo[j]<<1);
		    			// InvBits[0]=Fin;
		    			// HAL_UART_Transmit(&huart2,InvBits,sizeof(InvBits),100);// Sending in normal mode
		    			// HAL_Delay(1000);
		    		}else{
		    			Fin=(Fin>>1);
		    			InvArreglo[j]=(InvArreglo[j]<<1);
		    			//InvBits[0]=InvArreglo[0];
		    			//HAL_UART_Transmit(&huart2,InvBits,sizeof(InvBits),100);// Sending in normal mode
		    			//HAL_Delay(1000);
		    		}
		    	}
		    	InvBits[j]=Fin;
		    	Fin=0x00;
		    }

		    //Algoritmo para la generación del CRC16 de la cabecera invertida
		    for(int i=0; i<sizeof(InvBits)/sizeof(InvBits[0]); i++){
		    	  CRC16 ^= (uint16_t)(InvBits[i]<<8);

		    	  for (int i=0; i<8; i++){
		    		  if ((CRC16 & 0x8000) != 0){
		    			  CRC16 = (uint16_t)((CRC16 <<1)^G_X);
		    		  }else{
		    			  CRC16 <<=1;
		    		  }
		    	  }
		    }

		    // El CRC16 de 0x8805 = 0xC294
		    // El CRC16 de 0x88050101 = 0x901B
		    // El CRC16 de 0x8080A011 (0x88050101 invertido) = 0x7DCC
		    uint8_t CRC16_0 = (uint8_t)(CRC16 & 0x00FF);
		    uint8_t CRC16_1 = (uint8_t)((CRC16 >> 8)& 0x00FF);

		    bool status_crc16;
		    if(CRC16_0==cab[4] && CRC16_1==cab[5]){
		  	  status_crc16= true;
		  	  //myprintf("El CRC16 se recibio correctamente ... \n");
		  	  //myprintf("%02X %02X\n",cab[4],cab[5]);
		  	  return status_crc16;
		    }else{
		  	  status_crc16= false;
		  	  //myprintf("El CRC16 no esta correcto ... \n");
		  	  //myprintf("%02X %02X\n",cab[4],cab[5]);
		  	  return status_crc16;
		    }
}


bool gener_crc32(uint8_t *payload_cabecera,uint8_t size_cabecera,uint8_t *cab){

			//Algoritmo para la generación del CRC32 de la cabecera invertida
		    uint32_t G_Y = 0x04C11DB7; // Polynomial generator
		    uint32_t CRC32 = 0xFFFFFFFF;
		    uint8_t InvArreglo2[7] = {};
		    uint8_t Temp2;
		    uint8_t Fin2=0x00;
		    uint8_t InvBits2[7] ={};

		    //Se invierten los elementos del array
		    //Frame actual: 0x01 0xCC 0x7D 0x01 0x01 0x05 0x88
		    for(int i=0; i<size_cabecera; i++){
		    	InvArreglo2[i]=payload_cabecera[size_cabecera-i-1];
		    }

		    //Se invierten los bits de los elementos del array
		    //Frame LBS: 0x80 0x33 0xBE 0x80 0x80 0xA0 0x11
		    for(int j=0; j<size_cabecera;j++){
		    	for(int i=0; i<8; i++){
		    		Temp2=InvArreglo2[j]&(0x80);
		    		//InvBits3[0]=InvArreglo2[j];
		    		//HAL_UART_Transmit(&huart2,InvBits3,sizeof(InvBits3),100);// Sending in normal mode
		    		//HAL_Delay(1000);
		    		if(Temp2!=0x00){
		    			Fin2=(Fin2>>1);
		    			Fin2=Fin2|(0x80);
		    			InvArreglo2[j]=(InvArreglo2[j]<<1);
		    			//InvBits3[0]=Fin2;
		    			//HAL_UART_Transmit(&huart2,InvBits3,sizeof(InvBits3),100);// Sending in normal mode
		    			//HAL_Delay(1000);
		    		}else{
		    			Fin2=(Fin2>>1);
		    			InvArreglo2[j]=(InvArreglo2[j]<<1);
		    			//InvBits[0]=InvArreglo[0];
		    			//HAL_UART_Transmit(&huart2,InvBits,sizeof(InvBits),100);// Sending in normal mode
		    			//HAL_Delay(1000);
		    		}
		    	}
		    	InvBits2[j]=Fin2;
		    	Fin2=0x00;
		    }

		    for(int j=0; j<sizeof(InvBits2)/sizeof(InvBits2[0]); j++){
		    	CRC32 ^= (uint32_t)(InvBits2[j]<<24);

		    	  for (int j=0; j<8; j++){
		    		  if ((CRC32 & 0x80000000) != 0){
		    			  CRC32 = (uint32_t)((CRC32 <<1)^G_Y);
		    		  }else{
		    			  CRC32 <<=1;
		    		  }
		    	  }
		    }

		    uint8_t CRC32_byte3 = (uint8_t)((CRC32 >> 24) & 0xFF);//MSB
		    uint8_t CRC32_byte2 = (uint8_t)((CRC32 >> 16) & 0xFF);
		    uint8_t CRC32_byte1 = (uint8_t)((CRC32 >> 8) & 0xFF);
		    uint8_t CRC32_byte0 = (uint8_t)(CRC32 & 0xFF);//LSB

		    bool status_crc32;

		    //myprintf(" los crc32 son %02X %02X %02X %02X\n",CRC32_byte0,CRC32_byte1,CRC32_byte2,CRC32_byte3);
		    if(CRC32_byte0==cab[7] && CRC32_byte1==cab[8] && CRC32_byte2==cab[9] && CRC32_byte3==cab[10]){
		  	  status_crc32= true;
		  	  //myprintf("El CRC32 se recibio correctamente ... \n");
		  	  //myprintf("%02X %02X %02X %02X\n",cab[7],cab[8],cab[9],cab[10]);
		  	  return status_crc32;
		    }else{
		  	  status_crc32= false;
		  	  //myprintf("El CRC32 no esta correcto ... \n");
		  	  //myprintf("%02X ..\n",cab[7]);
		  	  //myprintf("%02X %02X %02X %02X\n",cab[7],cab[8],cab[9],cab[10]);
		  	  return status_crc32;
		    }
}



/*Funcion generadora del CRC16 */
uint16_t gener_crc16_rx (uint8_t TA,uint8_t SA,uint8_t PPID,uint8_t PS){

	 /***************************************************
	  * Definicion de variables y algoritmo para el CRC16
	  * Con inversion de bits
	  * CRC-CCITT-BR (bit reverse): LSB first
	  * CRC16, G(x)=0x1021 (algoritmo)
	  * CRC initial: 0XFFFF
	  * Cabecera inicial: 0x88 0x05 0x01 0x01
	  * Cabecera invertida: 0x80 0x80 0xA0 0x11
	  ***************************************************/
	  uint16_t G_X = 0x1021; // Polynomial generator
	  uint16_t CRC16 = 0xFFFF;
	  uint8_t arreglo[4] = {TA,SA,PPID,PS};//Se aumento PPID,PS
	  uint8_t InvArreglo[4] = {};
	  uint8_t InvBits[4] ={}; //Arreglo de bits invertidos de la cabecera  (LSB first)
	  uint8_t Temp;
	  uint8_t Fin=0x00;
	  uint8_t n = sizeof(arreglo)/sizeof(arreglo[0]);

	  //Se invierten los elementos del array
	  //Cabecera actual: 0x01 0x01 0x05 0x88
	  for(int i=0; i<n; i++){
		  InvArreglo[i]=arreglo[n-i-1];
	  }

	  //InvArreglo[0]=0x35;

	  //Se invierten los bits de los elementos del array
	  //Cabecera LBS: 0x80 0x80 0xA0 0x11
	  for(int j=0; j<n;j++){
		  for(int i=0; i<8; i++){
			  Temp=InvArreglo[j]&(0x80);
			  //InvBits[0]=InvArreglo[1];
			  //HAL_UART_Transmit(&huart2,InvBits,sizeof(InvBits),100);// Sending in normal mode
			  //HAL_Delay(1000);
			  if(Temp!=0x00){
				  Fin=(Fin>>1);
				  Fin=Fin|(0x80);
				  InvArreglo[j]=(InvArreglo[j]<<1);
				  // InvBits[0]=Fin;
				  // HAL_UART_Transmit(&huart2,InvBits,sizeof(InvBits),100);// Sending in normal mode
				  // HAL_Delay(1000);
			  }else{
				  Fin=(Fin>>1);
				  InvArreglo[j]=(InvArreglo[j]<<1);
				  //InvBits[0]=InvArreglo[0];
				  //HAL_UART_Transmit(&huart2,InvBits,sizeof(InvBits),100);// Sending in normal mode
				  //HAL_Delay(1000);
			  }
		  }
		  InvBits[j]=Fin;
		  Fin=0x00;
	  }

	  //Algoritmo para la generación del CRC16 de la cabecera invertida
	  for(int i=0; i<sizeof(InvBits)/sizeof(InvBits[0]); i++){
	  	  CRC16 ^= (uint16_t)(InvBits[i]<<8);

	  	  for (int i=0; i<8; i++){
	  		  if ((CRC16 & 0x8000) != 0){
	  			  CRC16 = (uint16_t)((CRC16 <<1)^G_X);
	  		  }else{
	  			  CRC16 <<=1;
	  		  }
	  	  }
	   }

	  return CRC16;

	  // El CRC16 de 0x8805 = 0xC294
	  // El CRC16 de 0x88050101 = 0x901B
	  // El CRC16 de 0x8080A011 (0x88050101 invertido) = 0x7DCC

	  /*uint8_t CRC16_0 = (uint8_t)(CRC16 & 0x00FF);
	  uint8_t CRC16_1 = (uint8_t)((CRC16 >> 8)& 0x00FF);*/
}

/*Funcion generadora del CRC32*/
uint32_t gener_crc32_rx (uint8_t TA,uint8_t SA,uint8_t PPID,uint8_t PS,uint8_t CRC16_1, uint8_t CRC16_0,uint8_t payload_envio_0){

	/***************************************************
	   * Definicion de variables y algoritmo para el CRC32
	   * Con inversion de bits
	   * CRC-CRC32-ISO3309, LSB first
	   * CRC32, G(x)= 0x04C11DB7 (algoritmo)
	   * CRC initial: 0xFFFFFFFF
	   * Prueba inicial: 0x88 05 01 01 7D CC 01
	  **************************************************/
	  uint32_t G_Y = 0x04C11DB7; // Polynomial generator
	  uint32_t CRC32 = 0xFFFFFFFF;
	  uint8_t arreglo2[7]={TA,SA,PPID,PS,CRC16_1,CRC16_0,payload_envio_0};

	  uint8_t InvArreglo2[7] = {};
	  uint8_t InvBits2[7] ={}; //Arreglo de bits invertidos de la cabecera  (LSB first)
	  //uint8_t InvBits3[1]={};
	  uint8_t Temp2;
	  uint8_t Fin2=0x00;
	  uint8_t n2 = sizeof(arreglo2)/sizeof(arreglo2[0]);

	  //Se invierten los elementos del array
	  //Frame actual: 0x01 0xCC 0x7D 0x01 0x01 0x05 0x88
	  for(int i=0; i<n2; i++){
		  InvArreglo2[i]=arreglo2[n2-i-1];
	  }

	  //Se invierten los bits de los elementos del array
	  //Frame LBS: 0x80 0x33 0xBE 0x80 0x80 0xA0 0x11
	  for(int j=0; j<n2;j++){
	  	  for(int i=0; i<8; i++){
	  		  Temp2=InvArreglo2[j]&(0x80);
	    	   	  //InvBits3[0]=InvArreglo2[j];
	    	   	  //HAL_UART_Transmit(&huart2,InvBits3,sizeof(InvBits3),100);// Sending in normal mode
	    	   	  //HAL_Delay(1000);
	  		  if(Temp2!=0x00){
	  			  Fin2=(Fin2>>1);
	  			  Fin2=Fin2|(0x80);
	  			  InvArreglo2[j]=(InvArreglo2[j]<<1);
	    	   		  //InvBits3[0]=Fin2;
	    	   		  //HAL_UART_Transmit(&huart2,InvBits3,sizeof(InvBits3),100);// Sending in normal mode
	    	   		  //HAL_Delay(1000);
	  		  }else{
	  			  Fin2=(Fin2>>1);
	  			  InvArreglo2[j]=(InvArreglo2[j]<<1);
	  			  //InvBits[0]=InvArreglo[0];
	    	   		  //HAL_UART_Transmit(&huart2,InvBits,sizeof(InvBits),100);// Sending in normal mode
	    	   		  //HAL_Delay(1000);
	  		  }
	  	  }
	  	  InvBits2[j]=Fin2;
	  	  Fin2=0x00;
	  }

	  //InvBits3[0]=InvBits2[6];
	  //HAL_UART_Transmit(&huart2,InvBits3,sizeof(InvBits3),100);// Sending in normal mode
	  //HAL_Delay(1000);

	  //Algoritmo para la generación del CRC32 de la cabecera invertida
	  for(int j=0; j<sizeof(InvBits2)/sizeof(InvBits2[0]); j++){
	  	  CRC32 ^= (uint32_t)(InvBits2[j]<<24);

	  	  for (int j=0; j<8; j++){
	  		  if ((CRC32 & 0x80000000) != 0){
	  			  CRC32 = (uint32_t)((CRC32 <<1)^G_Y);
	  		  }else{
	  			  CRC32 <<=1;
	  		  }
	  	  }
	  }

	  return CRC32;

	  // El CRC32 de 0x88050101 = 0x486ABA9C
	  // El CRC32 de 0x8033BE8080A011 = 0x81C46721
	  /*uint8_t CRC32_byte3 = (uint8_t)((CRC32 >> 24) & 0xFF);//MSB
	  uint8_t CRC32_byte2 = (uint8_t)((CRC32 >> 16) & 0xFF);
	  uint8_t CRC32_byte1 = (uint8_t)((CRC32 >> 8) & 0xFF);
	  uint8_t CRC32_byte0 = (uint8_t)(CRC32 & 0xFF);//LSB*/
}


/*******************************************************************************************************************/

uint8_t recibir_comando(void){
	int a =1;
	uint8_t cab[11]={};
	uint8_t comando;
	while (a==1)
	{

		  //************************************* Recibiendo comando *************************************************************
		  //Armado de Protocolo |TA|SA|PPID|PS|CRC16_0|CRC16_1|Payload[0]|CRC32_0|CRC32_1|CRC32_2|CRC32_3|
		  //**********************************************************************************************************************

		    //myprintf("Esperando comando ... \n");
		    HAL_StatusTypeDef status_rec = HAL_UART_Receive(&LINK_UART_HANDLE,cab,11,2500);

		    //myprintf("\r\n MicroSD inicializo BIEN \r\n\r\n");
		    //HAL_UART_Transmit(&huart4,cab,11,2500);
		    //myprintf("%02X ..\n",cab[7]);
		    //myprintf("%02X %02X %02X %02X \n",cab[0],cab[1],cab[2],cab[3]);
		    //myprintf("%02X %02X %02X %02X \n",cab[4],cab[5],cab[6],cab[7]);
		    //myprintf("%02X %02X %02X  \n",cab[8],cab[9],cab[10]);
		    //myprintf("%02X ..\n",cab[6]);
		    //Verificar si logro recibir el comando

		    //int status;
		    HAL_GPIO_TogglePin(WDG_GPIO_Port, WDG_Pin);
		    HAL_GPIO_TogglePin(LED_TEST_GPIO_Port, LED_TEST_Pin);

		    if(status_rec==HAL_OK){
		    	//status=1;
		    	//myprintf("Comando Recibido \n");
		    	//HAL_UART_Transmit(&huart4,cab,11,2500);
		    	//myprintf("%11X \n",cab);
		    	//myprintf("Status %d \n",status);
			    uint8_t cabecera[4] = {cab[0],cab[1],cab[2],cab[3]};
			    uint8_t size_cabecera = sizeof(cabecera)/sizeof(cabecera[0]);

			    bool status_crc16= gener_crc16(cabecera,size_cabecera,cab);
			    //********************** Verificacion CRC32
			    uint8_t payload_cabecera[7] = {cab[0],cab[1],cab[2],cab[3],cab[5],cab[4],cab[6]};
			    uint8_t size_payload_cabe = sizeof(payload_cabecera)/sizeof(payload_cabecera[0]);
			    //myprintf("entrando al crc32 x1... \n");
				bool status_crc32 = gener_crc32(payload_cabecera,size_payload_cabe,cab);
				if (status_crc16 == 1 && status_crc32 ==1 ){
					comando =  cab[6];
					a=0;
					break;
				}
				else {
					comando =  0xFF;
					a=0;
					break;
				}
				HAL_Delay(100);



				break;

		    }else{
		    	//status=0;
		    	//myprintf("Status %d \n",status);
		    	//myprintf("Comando no recibido\n");
		    	comando = 0xFF;
		    }
		}
		return comando;
}












void comando_recibido(uint8_t cab6){
  switch(cab6){
   case 0x01:
	//payload_total[0]=0x01; /*TomaFotoSD*/
	   //myprintf("Comando toma foto recibido\n");
	   envia_defrente();

	   enviar_comando(0X01);
	   break;
   case 0x02:
	//payload_total[0]=0x02; /*EnvíoFotoPaySTM32*/
	   //myprintf("Comando toma enviar foto\n");
	   HAL_GPIO_TogglePin(WDG_GPIO_Port, WDG_Pin);
	   HAL_Delay(3000);
	   (void)send_file_over_uart(filename);
	   enviar_comando(0X02);

	   break;
   case 0x03:
	   enviar_comando(0X03);
	   //envia_defrente();
	   break;
   case 0x05:
	   enviar_comando(0X05);
	   break;
   case 0x08:
	   enviar_comando(0X08);
	   break;
   case 0x06:
	//myprintf("\r\n Camara inicializo BIEN \r\n\r\n");
	   break;
   case 0x00:
	   enviar_comando(0X00);
	   break;
   case 0xFF:
	   enviar_comando(0XFF);
	   break;
   default:
	   enviar_comando(0XFF);

  }


}

void enviar_comando(uint8_t indicador){

	//PPID=indicador; // halla el PPID del frame recibido

	//payload_total[0] = comando_enviado(PPID); //Halla el payload_total con respecto al PPID

	PPID=indicador;
    // TODO: por ahiora etsa binem pero deben cambiar por el tema del payload
	payload_total[0] = indicador;

	for (int i=0; i<255; i++){
	  val = payload_total[i] & 0b11111111;
	  if (val != 0x00){
		  count=count+1;
	  }
	}

	//payload_envio[0] = payload_total[0];

    //PS = countPay(payload_envio);
	//PS = count;
	payload_envio[0] = payload_total[0];
	PS = 0X01;


    /******************************
    * Hallando el CRC16 y CRC32
    ******************************/
    uint16_t CRC16 = gener_crc16_rx(TA,SA,PPID,PS);

    uint8_t CRC16_0 = (uint8_t)(CRC16 & 0x00FF);
    uint8_t CRC16_1 = (uint8_t)((CRC16 >> 8)& 0x00FF);

    uint32_t CRC32 = gener_crc32_rx(TA,SA,PPID,PS,CRC16_1,CRC16_0,payload_envio[0]);

    uint8_t CRC32_byte3 = (uint8_t)((CRC32 >> 24) & 0xFF);//MSB
    uint8_t CRC32_byte2 = (uint8_t)((CRC32 >> 16) & 0xFF);
    uint8_t CRC32_byte1 = (uint8_t)((CRC32 >> 8) & 0xFF);
    uint8_t CRC32_byte0 = (uint8_t)(CRC32 & 0xFF);//LSB

    uint8_t resp[] = {TA,SA,PPID,PS,CRC16_0,CRC16_1,payload_envio[0],CRC32_byte0,CRC32_byte1,CRC32_byte2,CRC32_byte3};

    //myprintf("%02X %02X %02X %02X \n",resp[0],resp[1],resp[2],resp[3]);
    //myprintf("%02X %02X %02X %02X \n",resp[4],resp[5],resp[6],resp[7]);
    //myprintf("%02X %02X %02X  \n",resp[8],resp[9],resp[10]);

    HAL_Delay(500);
	//myprintf("Enviando comando ... \n");
	//HAL_UART_Transmit(&huart4,resp,11,2500);
	//myprintf("%X \n",resp);
	//uint8_t cab[] = {CRC32_byte3,CRC32_byte2,CRC32_byte1,CRC32_byte0};
    //myprintf("\r\n ENVIANDO \r\n\r\n");
	HAL_StatusTypeDef status_rec = HAL_UART_Transmit(&LINK_UART_HANDLE,resp,11,2500);// Sending in normal mode
	//myprintf("\r\n ENVIADO \r\n\r\n");
	HAL_Delay(500);
	count = 0;

	//myprintf("\r\n  \r\n\r\n");
	//HAL_UART_Transmit(&huart4,payload_total[0],1,2500);
	//myprintf("\r\n %d \r\n\r\n",payload_envio[0]);
	//myprintf("\r\n %d \r\n\r\n",sizeof(payload_envio)/sizeof(payload_envio[0]));

}

void comando_enviado(uint8_t PPID){
	 /**********************************
	   Definicion payload de respuesta
	   Comando de foto en camara 1: 0x01
	  **********************************/
	  //uint8_t payload_total[255];
	  uint8_t val_pay;

	  switch(PPID){
	   case 0x01:
		val_pay=0x02; /*Rpta TomaFotoSD*/
	  	break;
	   case 0x02:
		val_pay=0x04; /*Rpta EnvíoFotoPaySTM32*/
	    break;
	   case 0x03:
		val_pay=0x06; /*Rpta TomaFotoALmacYenviaPayload*/
	    break;
	   default:
		val_pay=0xFF;
	  }

	  //return payload_total;
	  return val_pay;
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */


  HAL_Delay(2000); //a short delay is important to let the SD card settle

  //some variables for FatFs
  FATFS FatFs; 	//Fatfs handle
  FIL fil; 		//File handle
  FRESULT fres; //Result after operations


  HAL_Delay(2000);

  HAL_GPIO_WritePin(WDG_GPIO_Port, WDG_Pin, GPIO_PIN_RESET);

  //Habilitar la sd y la camara
  HAL_GPIO_WritePin(GPIOF, SD_PWR_Pin,                   GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, OE_SD_Pin  | EN_CAM_Pin,      GPIO_PIN_RESET);

  //Led apagado
  HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin,                   GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOG, EN_SD_Pin,                   GPIO_PIN_RESET);
  //Deshabilitar la FRAM
  HAL_GPIO_WritePin(GPIOG, EN_FR_Pin ,    GPIO_PIN_SET);


  //Led prendido
  HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin,                   GPIO_PIN_SET);

  HAL_Delay(200);

  HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin,                   GPIO_PIN_RESET);

  HAL_Delay(200);

  HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin,                   GPIO_PIN_SET);


  //Watchdog togglepin
  HAL_GPIO_TogglePin(WDG_GPIO_Port, WDG_Pin);

  HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin,                   GPIO_PIN_RESET);



  //Led apagado
  HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin,                   GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin,                   GPIO_PIN_RESET);

  int estado = 0;
  int inicia = 0;
  int inicia2 = 0;
  int inicia3= 0;
  int bandera = 1;
  uint8_t comando = 0x00;





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */\

	  while (estado == 0){
		  HAL_GPIO_TogglePin(WDG_GPIO_Port, WDG_Pin);
		  comando = recibir_comando();
		  if (comando == 0x01 || comando== 0x02 ){
			  estado = 1;
			  break;
		  }
		  else if (comando == 0x08){
			  estado = 2;
			  break;
		  }


	  }


	  while (estado == 1){
		  HAL_GPIO_TogglePin(WDG_GPIO_Port, WDG_Pin);

		  //uint8_t comando = recibir_comando();
		  HAL_Delay(100);
		  uart_flush_rx_polling(&LINK_UART_HANDLE);
		  comando_recibido(comando);
		  estado = 0;
		  comando = 0x00;
		  break;


	  }
	  while (estado == 2){
		  HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin,                   GPIO_PIN_SET);
		  HAL_GPIO_TogglePin(WDG_GPIO_Port, WDG_Pin);
		  while(bandera == 1){

			  fres = f_mount(&FatFs, "", 1); //1=mount now
			  HAL_Delay(1000);
			  HAL_GPIO_TogglePin(LED_TEST_GPIO_Port, LED_TEST_Pin);
			  HAL_GPIO_TogglePin(WDG_GPIO_Port, WDG_Pin);

			  if (fres == FR_OK) {
				  inicia = 3;
				  bandera = 0;
				  break;
			  }
			  else {
				  inicia = 0;
			  }
		  }

		  HAL_GPIO_TogglePin(WDG_GPIO_Port, WDG_Pin);
		  int rc = camera_init();
		  HAL_Delay(1000);
		  if (rc == 0) {
			  inicia2 = 5;
		  }
		  else {
			  inicia2 = 0;
		  }
		  HAL_GPIO_TogglePin(WDG_GPIO_Port, WDG_Pin);
		  //enviar_comando(fres);
		  //comando_recibido(fres);

		  inicia3 =  inicia + inicia2;

		  if (inicia3 == 8){
			  comando_recibido(0x08);
			  inicia3 = 0;
			  comando = 0x00;
		  }
		  else if(inicia3 == 3){
			  comando_recibido(0x03);
			  inicia3 = 0;
			  comando = 0x00;
		  }
		  else if(inicia3 == 5){
			  comando_recibido(0x05);
			  inicia3 = 0;
			  comando = 0x00;
		  }
		  else{
			  comando_recibido(0x00);
			  inicia3 = 0;
			  comando = 0x00;
		  }

		  estado = 0;
		  break;
	  }


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin|LED_TEST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_PWR_GPIO_Port, SD_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, EN_FR_Pin|EN_SD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OE_SD_Pin|EN_CAM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WDG_GPIO_Port, WDG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI1_CS_Pin LED_TEST_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|LED_TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_PWR_Pin */
  GPIO_InitStruct.Pin = SD_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_PWR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_FR_Pin EN_SD_Pin */
  GPIO_InitStruct.Pin = EN_FR_Pin|EN_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : OE_SD_Pin EN_CAM_Pin */
  GPIO_InitStruct.Pin = OE_SD_Pin|EN_CAM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : WDG_Pin */
  GPIO_InitStruct.Pin = WDG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WDG_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

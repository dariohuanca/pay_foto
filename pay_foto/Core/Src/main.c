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

#define CHUNK          64u              // multiple of 512 recommended
#define SOF0           0x55
#define SOF1           0xAA
#define ACK            0x06
#define NAK            0x15
#define UART_TMO_MS    2500
#define MAX_RETRY      8

#define UART_TX_TMO_MS  2500

static vc0706_t cam;
extern UART_HandleTypeDef LINK_UART_HANDLE;


void myprintf(const char *fmt, ...);
static int generar_nombre_unico(char *out, size_t out_sz);
char filename[40];


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

int camera_init(void) {
    myprintf("\r\ninicializando camara...\r\n");

    // Bind to USART2 at 38400 (will HAL_UART_Init with that baud if needed)
    if (!vc0706_begin(&cam, &huart2, 38400)) {
        myprintf("no se detecto la camara o error de comunicacion\r\n");
        return -1;
    }

    // Version probe (good connectivity test)
    char *ver = vc0706_get_version(&cam);
    if (!ver) {
        myprintf("get_version fallo\r\n");
        return -2;
    }
    // The buffer is NUL-terminated; print a trimmed line
    myprintf("version: %s\r\n", ver);

    // Set resolution 640x480 (same as cam.setImageSize(VC0706_640x480))
    if (!vc0706_set_image_size(&cam, VC0706_640x480)) {
        myprintf("no se pudo fijar resolucion 640x480\r\n");
        return -3;
    }
    // Read back to confirm
    uint8_t sz = vc0706_get_image_size(&cam);
    if (sz != VC0706_640x480) {
        myprintf("aviso: resolucion leida = 0x%02X (esperado 0x00)\r\n", sz);
    } else {
        myprintf("resolucion establecida a 640x480\r\n");
    }

    // (Opcional) ajustar compresion JPEG: 0x00 (max calidad) .. 0xFF (max compresion)
    // (void)vc0706_set_compression(&cam, 0x36);

    myprintf("camara lista\r\n");
    return 0;
}



void myprintf(const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart5, (uint8_t*)buffer, len, -1);

}


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
    myprintf("[TX] uart_send end\r\n");

    uint8_t ack = 0;
    if (uart_recv(&ack, 1, UART_TMO_MS) != 0) return -2;
    //myprintf("uart ack = %d \r\n; ");
    myprintf("[TX] header_ok\r\n");
    myprintf("El ACK es 0x%02X\r\n", (unsigned)ack);   // Hex con cero a la izquierda
    return (ack == ACK) ? 0 : -3;
}

static int send_frame(uint16_t seq, const uint8_t *data, uint16_t len)
{
	myprintf("Enviando x4 \r\n");
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
    myprintf("Enviando x2 \r\n");

    while (sent < fsz) {
    	myprintf("Enviando x3 \r\n");

        UINT need = (fsz - sent > CHUNK) ? CHUNK : (UINT)(fsz - sent);
        fr = f_read(&f, buf, need, &br);
        //myprintf("[TX] f_read fr=%d br=%u need=%u\r\n", (int)fr, (unsigned)br, (unsigned)need);
        //if (fr != FR_OK || br == 0) { myprintf("[TX] f_read_fail\r\n"); return -105; }
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


//////////PARA LA NUCLEO////////////

/*
static bool button_pressed_edge(void)
{
    bool now = (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_RESET);
    bool pressed = (btn_prev && !now); // high -> low
    btn_prev = now;
    return pressed;
}

*/



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
    if (fr != FR_OK) { myprintf("f_mount=%d\r\n", fr); return -100; }

    if (!vc0706_take_picture(cam)) {
        myprintf("take_picture FAIL\r\n");
        f_mount(NULL, "", 0);
        return -200;
    }

    uint32_t jpglen = vc0706_frame_length(cam);
    if (jpglen == 0) {
        myprintf("frame_length=0\r\n");
        (void)vc0706_resume_video(cam);
        f_mount(NULL, "", 0);
        return -201;
    }
    myprintf("size=%lu bytes\r\n", (unsigned long)jpglen);

    //char filename[40];
    int rc = generar_nombre_unico(filename, sizeof(filename));
    if (rc != 0) {
        myprintf("name err=%d\r\n", rc);
        (void)vc0706_resume_video(cam);
        f_mount(NULL, "", 0);
        return -202;
    }


    fr = f_open(&file, filename, FA_WRITE | FA_CREATE_NEW);
    if (fr != FR_OK) {
        myprintf("f_open=%d\r\n", fr);
        (void)vc0706_resume_video(cam);
        f_mount(NULL, "", 0);
        return -203;
    }
    myprintf("writing %s ...\r\n", filename);

    uint32_t written = 0;
    int payload_offset = -1; // will be 0 or 5 after first chunk

	//Watchdog togglepin
	HAL_GPIO_TogglePin(WDG_GPIO_Port, WDG_Pin);

    while (written < jpglen) {
        uint32_t remain = jpglen - written;
        uint8_t n = (remain < VC0706_READ_CHUNK ) ? (uint8_t)remain : (uint8_t)VC0706_READ_CHUNK ;
        if (n > 240) n = 240;  // safety: VC0706 count is uint8_t

        uint8_t *buf = vc0706_read_picture(cam, n, 500);   // returns [5-byte header] + n data

        if (!buf) {
            myprintf("read_picture timeout/null\r\n");
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
            myprintf("f_write err=%d bw=%u n=%u\r\n", fr, (unsigned)bw, (unsigned)n);
            f_close(&file);
            (void)vc0706_resume_video(cam);
            f_mount(NULL, "", 0);
            return -205;
        }

        written += n;
        if ((written % (10*1024)) < VC0706_READ_CHUNK ) myprintf(".");

    	//Watchdog togglepin
    	HAL_GPIO_TogglePin(WDG_GPIO_Port, WDG_Pin);
    }
    myprintf("\r\n");

    f_sync(&file);
    f_close(&file);
    (void)vc0706_resume_video(cam);
    f_mount(NULL, "", 0);

    myprintf("done: %s (%lu bytes)\r\n", filename, (unsigned long)written);
    return 0;
}

///////////// PARA LA NUCLEO ///////////

/*

void user_loop_sender_sd(void)
{
    if (!g_sending && button_pressed_edge()) {
    	myprintf("Enviando \r\n");
        g_sending = true;
        (void)send_file_over_uart(SEND_FILENAME);
        g_sending = false;
    }
}

*/



void user_loop_sender_cam_sd(void)
{

	char filename[40];
	if (generar_nombre_unico(filename, sizeof(filename)) != 0) {
		myprintf("name gen failed\r\n");

		return;
	}

	myprintf("Capturando y guardando: %s\r\n", filename);
	int rc = capturar_imagen_a_sd_manual(&cam);
	if (rc != 0) {
		myprintf("capture/save failed rc=%d\r\n", rc);
		return;
	}

	//Watchdog togglepin
	HAL_GPIO_TogglePin(WDG_GPIO_Port, WDG_Pin);


	myprintf("Enviando por UART: %s\r\n", filename);

	HAL_Delay(500);
	(void)send_file_over_uart(filename);
	myprintf("Termino envio: %s\r\n", filename);

}





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART5_Init(void);
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
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  myprintf("\r\n~ SD card demo by kiwih ~\r\n\r\n");
  HAL_Delay(2000);

  HAL_GPIO_WritePin(WDG_GPIO_Port, WDG_Pin, GPIO_PIN_RESET);

  //Habilitar la sd y la camara
  HAL_GPIO_WritePin(GPIOF, SD_PWR_Pin,                   GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, OE_SD_Pin  | EN_CAM_Pin,      GPIO_PIN_RESET);

  //Led apagado
  HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin,                   GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PTE_GPIO_Port, PTE_Pin,                   GPIO_PIN_RESET);

  //Deshabilitar la FRAM
  HAL_GPIO_WritePin(GPIOG, EN_FR_Pin  | EN_SD_Pin,       GPIO_PIN_SET);


  //Led prendido
  HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin,                   GPIO_PIN_SET);
  HAL_GPIO_WritePin(PTE_GPIO_Port, PTE_Pin,                   GPIO_PIN_SET);
  HAL_Delay(2000);

  HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin,                   GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PTE_GPIO_Port, PTE_Pin,                   GPIO_PIN_RESET);
  HAL_Delay(2000);

  HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin,                   GPIO_PIN_SET);



  myprintf("\r\n~ SD card demo by kiwih ~\r\n\r\n");

  HAL_Delay(2000); //a short delay is important to let the SD card settle

  //Watchdog togglepin
  HAL_GPIO_TogglePin(WDG_GPIO_Port, WDG_Pin);

  HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin,                   GPIO_PIN_RESET);

  //some variables for FatFs
  FATFS FatFs; 	//Fatfs handle
  FIL fil; 		//File handle
  FRESULT fres; //Result after operations

  //Open the file system
  fres = f_mount(&FatFs, "", 1); //1=mount now
  if (fres != FR_OK) {
	myprintf("f_mount error (%i)\r\n", fres);
	//while(1);
  }

  //Let's get some statistics from the SD card
  DWORD free_clusters, free_sectors, total_sectors;

  FATFS* getFreeFs;

  fres = f_getfree("", &free_clusters, &getFreeFs);
  if (fres != FR_OK) {
	myprintf("f_getfree error (%i)\r\n", fres);
	//while(1);
  }

  //Formula comes from ChaN's documentation
  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
  free_sectors = free_clusters * getFreeFs->csize;

  myprintf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);
  HAL_Delay(1000);

  //Watchdog togglepin
  HAL_GPIO_TogglePin(WDG_GPIO_Port, WDG_Pin);


  int rc = camera_init();

  //Watchdog togglepin
  HAL_GPIO_TogglePin(WDG_GPIO_Port, WDG_Pin);

  HAL_Delay(1000);


  if (rc != 0) {
      myprintf("camera_init failed (%d)\r\n", rc);
      // opcional: parpadear LED o quedarse en loop de error
      // while (1) { HAL_Delay(250); }
  } else {
      myprintf("camera ok\r\n");
  }

  //Led apagado
  HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin,                   GPIO_PIN_SET);
  HAL_Delay(5000);
  HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin,                   GPIO_PIN_RESET);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  myprintf("camara lista\r\n");
	  //HAL_Delay(5000);
	  //HAL_GPIO_TogglePin(WDG_GPIO_Port, WDG_Pin);
	  //user_loop_sender_cam_sd();
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
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
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 1200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart1.Init.BaudRate = 1200;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin|LED_TEST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_PWR_GPIO_Port, SD_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, EN_FR_Pin|EN_SD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, PTE_Pin|WDG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OE_SD_Pin|EN_CAM_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PTE_Pin WDG_Pin */
  GPIO_InitStruct.Pin = PTE_Pin|WDG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OE_SD_Pin EN_CAM_Pin */
  GPIO_InitStruct.Pin = OE_SD_Pin|EN_CAM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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

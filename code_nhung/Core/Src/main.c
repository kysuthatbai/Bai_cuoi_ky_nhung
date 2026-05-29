#include "stm32f1xx.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// prototype encoder
void Encoder_Init(void);
int16_t Encoder_ReadCount(void);
int16_t Encoder_ReadSpeed(void);

uint8_t LCD_ShowSpeed(void);
uint8_t LCD_Command(uint8_t cmd);

void Speed_Update(void);
void Warning_Update(void);

void PID_Control_Update(void);

void TIM4_10ms_Init(void);

void SystemClock_Config(void);
void Error_Handler(void);

#define LED_PIN     0       // PB0

#define LCD_BL 0x08
#define LCD_EN 0x04
#define LCD_RS 0x01

uint8_t LCD_ADDR = 0x27;

#define PWM_PIN        1   // PA1 - TIM2_CH2
#define BTN_UP_PIN     2   // PA2
#define BTN_DOWN_PIN   3   // PA3
#define BTN_RESET_PIN  4   // PA4

#define L298N_IN1_PIN  5   // PA5
#define L298N_IN2_PIN  6   // PA6

#define SPEED_STEP     10
#define SPEED_MAX      100
#define SPEED_MIN      0

volatile uint8_t motor_speed = 0;

#define ENCODER_CPR  1120   // encoder của bạn: khoảng 1234.3 xung/vòng

#define SETPOINT_RPM_STEP  50
#define SETPOINT_RPM_MAX   300
#define SETPOINT_RPM_MIN   0

#define PWM_MAX 100
#define PWM_MIN 0
#define PWM_START 25

#define SPEED_ERROR_LIMIT_RPM 20

int16_t encoder_count = 0;
int16_t encoder_last_count = 0;

volatile int16_t speed_setpoint_rpm = 0;
volatile int16_t speed_measured_rpm = 0;

volatile int16_t speed_error_signed_rpm = 0;
volatile int16_t speed_error_rpm = 0;

float pid_kp = 0.5f;
float pid_ki = 0.07f;
float pid_kd = 0.000f;

float pid_output_pwm = 0.0f;
float pid_integral = 0.0f;
float derivative = 0.0f;
float pid_last_error = 0.0f;

#define PID_DT 0.01f   // 10ms = 0.01s
#define DEBOUNCE_MS 10

volatile uint8_t speed_warning = 0;
volatile uint8_t motor_stuck_warning = 0;

volatile uint8_t flag_send_uart = 0;
volatile uint8_t flag_update_lcd = 0;

volatile uint16_t uart_tick = 0;
volatile uint16_t lcd_tick = 0;

volatile uint32_t system_ms = 0;

#define STUCK_LED_PIN              7

#define ERROR_PERCENT_LIMIT        5
#define STUCK_SPEED_MIN_RPM        5
#define STUCK_TIME_MS              500
#define STUCK_TIME_COUNT           (STUCK_TIME_MS / 10)

#define BUTTON_DEBOUNCE_MS 30
//----------------------------------- debug ------------------------------------

void delay_ms(uint32_t ms)
{
    for(uint32_t i = 0; i < ms; i++)
        for(volatile uint32_t j = 0; j < 8000; j++);
}

void LED_On(void)
{
    GPIOB->BSRR = (1 << LED_PIN);
}

void LED_Off(void)
{
    GPIOB->BRR = (1 << LED_PIN);
}
void Motor_Stuck_LED_On(void)
{
    GPIOA->BSRR = (1 << STUCK_LED_PIN);
}

void Motor_Stuck_LED_Off(void)
{
    GPIOA->BRR = (1 << STUCK_LED_PIN);
}
void GPIO_Init_All(void)
{
    RCC->APB2ENR |= 1 << 2; // port A
    RCC->APB2ENR |= 1 << 3; // port B
    RCC->APB2ENR |= 1 << 0;

    // set up pin PA0
    GPIOA->CRL &= ~(0xF << 0); // clear register with bit 0-3
    GPIOA->CRL |= 1 << 3;
    GPIOA->ODR |=  1 << 0; // set pull down

    // PB0 output push-pull 2MHz
    GPIOB->CRL &= ~(0xF << 0); // clear bit 0-3
    GPIOB->CRL |= (1 << 1);    // set output

    // PB6 SCL, PB7 SDA alternate function open-drain 50MHz
    GPIOB->CRL &= ~((0xF << 24) | (0xF << 28));
    GPIOB->CRL |=  ((0xF << 24) | (0xF << 28));
}
//----------------------------- lcd -----------------------------------
void I2C1_Init(void)
{
    RCC->APB1ENR |= 1 << 21; // Init i2c1

    I2C1->CR1 |= (1 << 15); // reset i2c
    I2C1->CR1 &= ~(1 << 15); // exit reset

    I2C1->CR2 = 8;
    I2C1->CCR = 40;
    I2C1->TRISE = 9;

    I2C1->CR1 |= I2C_CR1_PE;
}

#define I2C_TIMEOUT 5000

void I2C1_Recover(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;

    I2C1->CR1 &= ~I2C_CR1_PE;

    I2C1->CR1 |= I2C_CR1_SWRST;
    for(volatile uint32_t i = 0; i < 1000; i++);
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    I2C1->CR2 = 8;
    I2C1->CCR = 40;
    I2C1->TRISE = 9;

    I2C1->CR1 |= I2C_CR1_PE;
}
uint8_t I2C1_Write(uint8_t addr, uint8_t data)
{
    uint32_t timeout;

    // Nếu bus đang bận quá lâu thì reset I2C
    timeout = I2C_TIMEOUT;
    while(I2C1->SR2 & I2C_SR2_BUSY)
    {
        if(--timeout == 0)
        {
            I2C1_Recover();
            return 0;
        }
    }

    // START
    I2C1->CR1 |= I2C_CR1_START;

    timeout = I2C_TIMEOUT;
    while(!(I2C1->SR1 & I2C_SR1_SB))
    {
        if(--timeout == 0)
        {
            I2C1_Recover();
            return 0;
        }
    }

    // Send address
    I2C1->DR = addr << 1;

    timeout = I2C_TIMEOUT;
    while(!(I2C1->SR1 & I2C_SR1_ADDR))
    {
        if(I2C1->SR1 & I2C_SR1_AF)
        {
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
            return 0;
        }

        if(--timeout == 0)
        {
            I2C1_Recover();
            return 0;
        }
    }

    volatile uint32_t temp;
    temp = I2C1->SR1;
    temp = I2C1->SR2;
    (void)temp;

    // Chờ TXE
    timeout = I2C_TIMEOUT;
    while(!(I2C1->SR1 & I2C_SR1_TXE))
    {
        if(--timeout == 0)
        {
            I2C1_Recover();
            return 0;
        }
    }

    I2C1->DR = data;

    // Chờ truyền xong
    timeout = I2C_TIMEOUT;
    while(!(I2C1->SR1 & I2C_SR1_BTF))
    {
        if(--timeout == 0)
        {
            I2C1_Recover();
            return 0;
        }
    }

    I2C1->CR1 |= I2C_CR1_STOP;
    return 1;
}

uint8_t LCD_Pulse(uint8_t data)
{
    if(!I2C1_Write(LCD_ADDR, data | LCD_BL | LCD_EN))
        return 0;

    if(!I2C1_Write(LCD_ADDR, data | LCD_BL))
        return 0;

    return 1;
}

uint8_t LCD_Send4Bit(uint8_t data)
{
    return LCD_Pulse(data);
}

uint8_t LCD_Data(uint8_t data)
{
    if(!LCD_Send4Bit((data & 0xF0) | LCD_RS))
        return 0;

    if(!LCD_Send4Bit(((data << 4) & 0xF0) | LCD_RS))
        return 0;

    return 1;
}

void LCD_Init(void)
{
    delay_ms(100);

    LCD_Send4Bit(0x30);
    delay_ms(10);

    LCD_Send4Bit(0x30);
    delay_ms(10);

    LCD_Send4Bit(0x30);
    delay_ms(10);

    LCD_Send4Bit(0x20);
    delay_ms(10);

    LCD_Command(0x28);
    LCD_Command(0x0C);
    LCD_Command(0x06);
    LCD_Command(0x01);
    delay_ms(10);
}

uint8_t LCD_SetCursor(uint8_t row, uint8_t col)
{
    if(row == 0)
        return LCD_Command(0x80 + col);
    else
        return LCD_Command(0xC0 + col);
}

uint8_t LCD_Print(char *s)
{
    while(*s)
    {
        if(!LCD_Data(*s++))
            return 0;
    }

    return 1;
}

uint8_t LCD_Command(uint8_t cmd)
{
    if(!LCD_Send4Bit(cmd & 0xF0))
        return 0;

    if(!LCD_Send4Bit((cmd << 4) & 0xF0))
        return 0;

    if(cmd == 0x01 || cmd == 0x02)
        delay_ms(2);

    return 1;
}

uint8_t LCD_ShowSpeed(void)
{
    char line[32];

    if(!LCD_SetCursor(0, 0))
        return 0;

    snprintf(line, sizeof(line), "SP:%4d ME:%4d ", speed_setpoint_rpm, speed_measured_rpm);

    if(!LCD_Print(line))
        return 0;

    if(!LCD_SetCursor(1, 0))
        return 0;

    snprintf(line, sizeof(line), "E:%5d PWM:%3d", speed_error_signed_rpm, motor_speed);

    if(!LCD_Print(line))
        return 0;

    return 1;
}

//------------------------------------------ control motor ----------------------------
// PA1 PWM TIM2_CH2, PA2 PA3 PA4 button, PA5 PA6 direction
void Motor_GPIO_Init(void)
{
    // PA1 alternate function push-pull 50MHz
    GPIOA->CRL &= ~(0xF << 4);
    GPIOA->CRL |=  (0xB << 4);

    // PA2 PA3 PA4 input pull-up
    GPIOA->CRL &= ~((0xF << 8) | (0xF << 12) | (0xF << 16)); // clear bit with port
    GPIOA->CRL |=  ((0x8 << 8) | (0x8 << 12) | (0x8 << 16)); // set up mode and nefc for port

    GPIOA->ODR |= (1 << BTN_UP_PIN); // set pull-up
    GPIOA->ODR |= (1 << BTN_DOWN_PIN); // same up
    GPIOA->ODR |= (1 << BTN_RESET_PIN); // same up

    // PA5 PA6 PA7 output push-pull 2MHz
    GPIOA->CRL &= ~((15 << 20) | (15 << 24) | (15 << 28));
    GPIOA->CRL |=  ((2 << 20)  | (2 << 24)  | (2 << 28));
}

// PWM PA1 TIM2_CH2, tần số khoảng 1kHz
void Motor_PWM_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //  enable clock for time 2, it is bit 0 as 1 << 0

    TIM2->PSC = 8 - 1;      // clock APB1 = 8MHz => f = 8Mhz / 1 + 7 = 1MHz
    TIM2->ARR = 1000 - 1; // set Arr = 999, it is mean count to 1000

    // TIM2 CH2 PWM mode 1
    TIM2->CCMR1 &= ~(0xFF << 8);      // clear 3 bit for channels Ch2, OC2M
    TIM2->CCMR1 |=  (6 << 12);        // OC2M = 110 PWM mode 1
    TIM2->CCMR1 |=  (1 << 11);        // OC2PE enable
    TIM2->CCER |= TIM_CCER_CC2E;      // Enable CH2 output

    TIM2->CR1 |= TIM_CR1_ARPE;
    TIM2->EGR |= TIM_EGR_UG;
    TIM2->CR1 |= TIM_CR1_CEN;

    TIM2->CCR2 = 0; // duty
}

void Motor_SetSpeed(uint8_t duty)
{
    if(duty > 100) duty = 100;

    motor_speed = duty;

    TIM2->CCR2 = duty * 10;
}

void Motor_SetForward(void)
{
    GPIOA->BSRR = (1 << L298N_IN1_PIN);  // IN1 = 1
    GPIOA->BRR  = (1 << L298N_IN2_PIN);  // IN2 = 0
}

void Motor_Stop(void)
{
    Motor_SetSpeed(0);
}

void Motor_Init_All(void)
{
    Motor_GPIO_Init();
    Motor_PWM_Init();
    Motor_SetForward();
    Motor_SetSpeed(0);
}

//--------------------------------- button ---------------------------------------
void Motor_Button_Update(void)
{
    static uint8_t up_last    = 1;
    static uint8_t down_last  = 1;
    static uint8_t reset_last = 1;

    uint8_t up_now;
    uint8_t down_now;
    uint8_t reset_now;

    up_now    = (GPIOA->IDR & (1 << BTN_UP_PIN))    ? 1 : 0;
    down_now  = (GPIOA->IDR & (1 << BTN_DOWN_PIN))  ? 1 : 0;
    reset_now = (GPIOA->IDR & (1 << BTN_RESET_PIN)) ? 1 : 0;

    if(up_last == 1 && up_now == 0)
    {
        if(speed_setpoint_rpm + SETPOINT_RPM_STEP <= SETPOINT_RPM_MAX)
        {
            speed_setpoint_rpm += SETPOINT_RPM_STEP;
        }
        else
        {
            speed_setpoint_rpm = SETPOINT_RPM_MAX;
        }
    }

    if(down_last == 1 && down_now == 0)
    {
        if(speed_setpoint_rpm >= SETPOINT_RPM_STEP)
        {
            speed_setpoint_rpm -= SETPOINT_RPM_STEP;
        }
        else
        {
            speed_setpoint_rpm = SETPOINT_RPM_MIN;
        }
    }

    if(reset_last == 1 && reset_now == 0)
    {
        speed_setpoint_rpm = 0;
        Motor_Stop();
    }

    up_last    = up_now;
    down_last  = down_now;
    reset_last = reset_now;
}

//------------------------------------------ encoder ----------------------------------
void Encoder_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Tắt JTAG, giữ SWD để dùng PB4
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

    // TIM3 partial remap: CH1 -> PB4, CH2 -> PB5
    AFIO->MAPR &= ~AFIO_MAPR_TIM3_REMAP;
    AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_1;

    // PB4 PB5 input floating
    GPIOB->CRL &= ~((0xF << 16) | (0xF << 20));
    GPIOB->CRL |=  ((0x4 << 16) | (0x4 << 20));

    TIM3->ARR = 0xFFFF;
    TIM3->CNT = 0;

    // Encoder mode 3: đếm cả 2 kênh A/B
    TIM3->SMCR &= ~TIM_SMCR_SMS;
    TIM3->SMCR |=  TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;

    // CC1 và CC2 là input
    TIM3->CCMR1 &= ~((0x3 << 0) | (0x3 << 8));
    TIM3->CCMR1 |=  ((0x1 << 0) | (0x1 << 8));

    TIM3->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);

    TIM3->CR1 |= TIM_CR1_CEN;
}

int16_t Encoder_ReadCount(void)
{
    return (int16_t)TIM3->CNT;
}

//----------------------------------------- warning -----------------------------------
void Speed_Update(void)
{
    int16_t count_now;
    int16_t pulse;
    int32_t pulse_abs;
    int32_t speed_cps;
    int16_t rpm_raw;

    count_now = (int16_t)TIM3->CNT;
    pulse = count_now - encoder_last_count;
    encoder_last_count = count_now;

    if(pulse < 0)
        pulse_abs = -pulse;
    else
        pulse_abs = pulse;

    speed_cps = pulse_abs * 100;

    rpm_raw = (speed_cps * 60) / ENCODER_CPR;

    speed_measured_rpm = (speed_measured_rpm * 7 + rpm_raw * 3) / 10;

    // Sai số có dấu
    speed_error_signed_rpm = speed_setpoint_rpm - speed_measured_rpm;

    // Sai số tuyệt đối
    if(speed_error_signed_rpm < 0)
        speed_error_rpm = -speed_error_signed_rpm;
    else
        speed_error_rpm = speed_error_signed_rpm;
}

void Warning_Update(void)
{
    static uint16_t stuck_count = 0;
    uint16_t error_percent = 0;

    // ================= CẢNH BÁO SAI SỐ TỐC ĐỘ =================
    if(speed_setpoint_rpm > 0)
    {
        error_percent = (speed_error_rpm * 100) / speed_setpoint_rpm;

        if(error_percent > ERROR_PERCENT_LIMIT)
        {
            speed_warning = 1;
            LED_On();      // LED PB0 cảnh báo sai số tốc độ
        }
        else
        {
            speed_warning = 0;
            LED_Off();
        }
    }
    else
    {
        speed_warning = 0;
        LED_Off();
    }

    // ================= CẢNH BÁO ĐỘNG CƠ BỊ KẸT =================
    if(speed_setpoint_rpm > 0 &&
       motor_speed >= PWM_START &&
       speed_measured_rpm < STUCK_SPEED_MIN_RPM)
    {
        stuck_count++;

        if(stuck_count >= STUCK_TIME_COUNT)
        {
            motor_stuck_warning = 1;
            Motor_Stuck_LED_On();    // LED PA7 sáng
        }
    }
    else
    {
        stuck_count = 0;
        motor_stuck_warning = 0;
        Motor_Stuck_LED_Off();       // LED PA7 tắt
    }
}

//------------------------------------------ pid --------------------------------------
void PID_Control_Update(void)
{
    float error;
    float pwm_base = 20.0f;   // PWM tối thiểu để motor bắt đầu quay

    if(speed_setpoint_rpm <= 0)
    {
        pid_integral = 0;
        pid_last_error = 0;
        pid_output_pwm = 0;
        Motor_SetSpeed(0);
        return;
    }

    error = (float)speed_error_signed_rpm;

    pid_integral += error * PID_DT;
    derivative = error / PID_DT;

    pid_output_pwm = pwm_base + pid_kp * error + pid_ki * pid_integral + derivative * pid_kd;

    if(pid_output_pwm > 100.0f)
        pid_output_pwm = 100.0f;

    if(pid_output_pwm < 0.0f)
        pid_output_pwm = 0.0f;

    Motor_SetSpeed((uint8_t)pid_output_pwm);
}
//----------------------------------------- interup ----------------------------------
void TIM4_10ms_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    TIM4->PSC = 8000 - 1;
    TIM4->ARR = 10 - 1;

    TIM4->CNT = 0;

    TIM4->DIER |= TIM_DIER_UIE;

    TIM4->EGR |= TIM_EGR_UG;

    NVIC_SetPriority(TIM4_IRQn, 1);
    NVIC_EnableIRQ(TIM4_IRQn);

    TIM4->CR1 |= TIM_CR1_CEN;
}

void TIM4_IRQHandler(void)
{
    if(TIM4->SR & TIM_SR_UIF)
    {
        TIM4->SR &= ~TIM_SR_UIF;

        system_ms += 10;

        Speed_Update();
        PID_Control_Update();

        lcd_tick++;
        if(lcd_tick >= 50)
        {
            lcd_tick = 0;
            flag_update_lcd = 1;
        }
    }
}

//------------------------------------------ main -------------------------------------
int main(void)
{
	SystemClock_Config();
    GPIO_Init_All();
    I2C1_Init();
    LCD_Init();
    Encoder_Init();
    Motor_Init_All();

    TIM4_10ms_Init();

    while(1)
    {
    	Motor_Button_Update();
    	Warning_Update();

    	if(flag_update_lcd)
    	{
    	    flag_update_lcd = 0;

    	    if(!LCD_ShowSpeed())
    	    {
    	        I2C1_Recover();
    	    }
    	}

    }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

#include "stm32f1xx.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// prototype encoder
void Encoder_Init(void);
int16_t Encoder_ReadCount(void);
int16_t Encoder_ReadSpeed(void);

uint8_t LCD_Command(uint8_t cmd);

uint8_t LCD_ShowAngle(void);

void Angle_Update(void);
void Warning_Update(void);

void PID_Control_Update(void);

void TIM4_10ms_Init(void);

void SystemClock_Config(void);
void Error_Handler(void);

#define LED_PIN     0       // PB0
#define BUTTON_PIN  0       // PA0

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

// PWM L298N ENA  -> PA1  (TIM2_CH2)
// L298N IN1      -> PA5
// L298N IN2      -> PA6

// Button tăng    -> PA2 nối xuống GND
// Button giảm    -> PA3 nối xuống GND
// Button reset   -> PA4 nối xuống GND

// ================= ENCODER =================
// Encoder A -> PB4
// Encoder B -> PB5
// Dùng TIM3 Encoder Mode

#define ENCODER_CPR  905   // encoder của bạn: khoảng 1234.3 xung/vòng

#define ANGLE_STEP_DEG       10
#define ANGLE_MAX_DEG        360
#define ANGLE_MIN_DEG       -360

#define ANGLE_TOLERANCE_DEG  2
#define ANGLE_ERROR_LIMIT_DEG 5

#define PWM_MAX 100
#define PWM_MIN 0
#define PWM_START 25

uint16_t encoder_last_count = 0;

volatile int32_t encoder_position_count = 0;

volatile int32_t angle_setpoint_deg = 0;
volatile int32_t angle_measured_deg = 0;

volatile int32_t angle_error_signed_deg = 0;
volatile int32_t angle_error_deg = 0;

/*
   PID điều khiển góc.
   Ban đầu nên dùng P hoặc PD, chưa nên dùng I nhiều vì dễ quá đà.
*/
float pid_kp = 1.2f;
float pid_ki = 0.0f;
float pid_kd = 0.02f;

float pid_output_pwm = 0.0f;
float pid_integral = 0.0f;
float pid_last_error = 0.0f;

#define PID_DT 0.01f   // 10ms = 0.01s
#define DEBOUNCE_MS 10

volatile uint8_t angle_warning = 0;

volatile uint8_t flag_update_lcd = 0;
volatile uint16_t lcd_tick = 0;

volatile uint32_t system_ms = 0;

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

void GPIO_Init_All(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    // PA0 input pull-up: button nối PA0 xuống GND
    GPIOA->CRL &= ~(0xF << 0);
    GPIOA->CRL |=  (0x8 << 0);
    GPIOA->ODR |=  (1 << BUTTON_PIN);

    // PB0 output push-pull 2MHz
    GPIOB->CRL &= ~(0xF << 0);
    GPIOB->CRL |=  (0x2 << 0);

    // PB6 SCL, PB7 SDA alternate function open-drain 50MHz
    GPIOB->CRL &= ~((0xF << 24) | (0xF << 28));
    GPIOB->CRL |=  ((0xF << 24) | (0xF << 28));
}
//----------------------------- lcd -----------------------------------
void I2C1_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

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

uint8_t I2C_CheckAddress(uint8_t addr)
{
    return I2C1_Write(addr, LCD_BL);
}

uint8_t LCD_FindAddress(void)
{
    if(I2C_CheckAddress(0x27))
    {
        LCD_ADDR = 0x27;
        return 1;
    }

    if(I2C_CheckAddress(0x3F))
    {
        LCD_ADDR = 0x3F;
        return 1;
    }

    if(I2C_CheckAddress(0x20))
    {
        LCD_ADDR = 0x20;
        return 1;
    }

    if(I2C_CheckAddress(0x21))
    {
        LCD_ADDR = 0x21;
        return 1;
    }

    return 0;
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

uint8_t LCD_ShowAngle(void)
{
    char line[32];

    if(!LCD_SetCursor(0, 0))
        return 0;

    snprintf(line, sizeof(line), "SP:%4ld ME:%4ld",
             (long)angle_setpoint_deg,
             (long)angle_measured_deg);

    if(!LCD_Print(line))
        return 0;

    if(!LCD_SetCursor(1, 0))
        return 0;

    snprintf(line, sizeof(line), "ER:%4ld PWM:%3d",
             (long)angle_error_signed_deg,
             motor_speed);

    if(!LCD_Print(line))
        return 0;

    return 1;
}

//------------------------------------------ control motor ----------------------------
// PA1 PWM TIM2_CH2, PA2 PA3 PA4 button, PA5 PA6 direction
void Motor_GPIO_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    // PA1 alternate function push-pull 50MHz
    GPIOA->CRL &= ~(0xF << 4);
    GPIOA->CRL |=  (0xB << 4);

    // PA2 PA3 PA4 input pull-up
    GPIOA->CRL &= ~((0xF << 8) | (0xF << 12) | (0xF << 16));
    GPIOA->CRL |=  ((0x8 << 8) | (0x8 << 12) | (0x8 << 16));

    GPIOA->ODR |= (1 << BTN_UP_PIN);
    GPIOA->ODR |= (1 << BTN_DOWN_PIN);
    GPIOA->ODR |= (1 << BTN_RESET_PIN);

    // PA5 PA6 output push-pull 2MHz
    GPIOA->CRL &= ~((0xF << 20) | (0xF << 24));
    GPIOA->CRL |=  ((0x2 << 20) | (0x2 << 24));
}

// PWM PA1 TIM2_CH2, tần số khoảng 1kHz
void Motor_PWM_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 8 - 1;
    TIM2->ARR = 1000 - 1;

    // TIM2 CH2 PWM mode 1
    TIM2->CCMR1 &= ~(0xFF << 8);
    TIM2->CCMR1 |=  (6 << 12);        // OC2M = 110 PWM mode 1
    TIM2->CCMR1 |=  (1 << 11);        // OC2PE enable

    TIM2->CCER |= TIM_CCER_CC2E;      // Enable CH2 output

    TIM2->CR1 |= TIM_CR1_ARPE;
    TIM2->EGR |= TIM_EGR_UG;
    TIM2->CR1 |= TIM_CR1_CEN;

    TIM2->CCR2 = 0;
}

// duty: 0 -> 100 %
void Motor_SetSpeed(uint8_t duty)
{
    if(duty > 100) duty = 100;

    motor_speed = duty;

    TIM2->CCR2 = duty * 10;
}

// Cố định quay 1 chiều
void Motor_SetForward(void)
{
    GPIOA->BSRR = (1 << L298N_IN1_PIN);  // IN1 = 1
    GPIOA->BRR  = (1 << L298N_IN2_PIN);  // IN2 = 0
}
void Motor_SetReverse(void)
{
    GPIOA->BRR  = (1 << L298N_IN1_PIN);  // IN1 = 0
    GPIOA->BSRR = (1 << L298N_IN2_PIN);  // IN2 = 1
}

void Motor_Coast(void)
{
    Motor_SetSpeed(0);

    GPIOA->BRR = (1 << L298N_IN1_PIN);
    GPIOA->BRR = (1 << L298N_IN2_PIN);
}

void Motor_SetSignedPWM(float pwm)
{
    if(pwm > 0.0f)
    {
        Motor_SetForward();

        if(pwm > PWM_MAX)
            pwm = PWM_MAX;

        Motor_SetSpeed((uint8_t)pwm);
    }
    else if(pwm < 0.0f)
    {
        Motor_SetReverse();

        pwm = -pwm;

        if(pwm > PWM_MAX)
            pwm = PWM_MAX;

        Motor_SetSpeed((uint8_t)pwm);
    }
    else
    {
        Motor_Coast();
    }
}

void Motor_Stop(void)
{
    Motor_Coast();
}

void Motor_Init_All(void)
{
    Motor_GPIO_Init();
    Motor_PWM_Init();
    Motor_Stop();
}
//--------------------------------- button ---------------------------------------
void Motor_Button_Update(void)
{
    static uint8_t up_last_raw = 1;
    static uint8_t down_last_raw = 1;
    static uint8_t reset_last_raw = 1;

    static uint8_t up_stable = 1;
    static uint8_t down_stable = 1;
    static uint8_t reset_stable = 1;

    static uint32_t up_last_change = 0;
    static uint32_t down_last_change = 0;
    static uint32_t reset_last_change = 0;

    uint32_t now = system_ms;

    uint8_t up_raw;
    uint8_t down_raw;
    uint8_t reset_raw;

    up_raw    = (GPIOA->IDR & (1 << BTN_UP_PIN))    ? 1 : 0;
    down_raw  = (GPIOA->IDR & (1 << BTN_DOWN_PIN))  ? 1 : 0;
    reset_raw = (GPIOA->IDR & (1 << BTN_RESET_PIN)) ? 1 : 0;

    // ================= NÚT UP =================
    if(up_raw != up_last_raw)
    {
        up_last_raw = up_raw;
        up_last_change = now;
    }

    if((uint32_t)(now - up_last_change) >= BUTTON_DEBOUNCE_MS)
    {
        if(up_raw != up_stable)
        {
            up_stable = up_raw;

            // Nhấn nút: vì dùng pull-up nên nhấn = 0
            if(up_stable == 0)
            {
            	if(angle_setpoint_deg + ANGLE_STEP_DEG <= ANGLE_MAX_DEG)
            	    angle_setpoint_deg += ANGLE_STEP_DEG;
            	else
            	    angle_setpoint_deg = ANGLE_MAX_DEG;
            }
        }
    }

    // ================= NÚT DOWN =================
    if(down_raw != down_last_raw)
    {
        down_last_raw = down_raw;
        down_last_change = now;
    }

    if((uint32_t)(now - down_last_change) >= BUTTON_DEBOUNCE_MS)
    {
        if(down_raw != down_stable)
        {
            down_stable = down_raw;

            if(down_stable == 0)
            {
            	if(angle_setpoint_deg - ANGLE_STEP_DEG >= ANGLE_MIN_DEG)
            	    angle_setpoint_deg -= ANGLE_STEP_DEG;
            	else
            	    angle_setpoint_deg = ANGLE_MIN_DEG;
            }
        }
    }

    // ================= NÚT RESET =================
    if(reset_raw != reset_last_raw)
    {
        reset_last_raw = reset_raw;
        reset_last_change = now;
    }

    if((uint32_t)(now - reset_last_change) >= BUTTON_DEBOUNCE_MS)
    {
        if(reset_raw != reset_stable)
        {
            reset_stable = reset_raw;

            if(reset_stable == 0)
            {
            	angle_setpoint_deg = 0;
            	pid_integral = 0;
            	pid_last_error = 0;
            }
        }
    }
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
void Angle_Update(void)
{
    uint16_t count_now;
    int16_t delta_count;

    count_now = (uint16_t)TIM3->CNT;

    /*
       Lấy số xung thay đổi trong 10ms.
       Cách này vẫn xử lý được khi bộ đếm TIM3 bị tràn 0 -> 65535.
    */
    delta_count = (int16_t)(count_now - encoder_last_count);

    encoder_last_count = count_now;

    /*
       Cộng dồn vị trí encoder.
       Đây là vị trí tuyệt đối tính từ lúc bật mạch.
    */
    encoder_position_count += delta_count;

    /*
       Đổi xung encoder sang độ.
       ENCODER_CPR là số xung cho 1 vòng 360 độ.
    */
    angle_measured_deg = (encoder_position_count * 360L) / ENCODER_CPR;

    angle_error_signed_deg = angle_setpoint_deg - angle_measured_deg;

    if(angle_error_signed_deg < 0)
        angle_error_deg = -angle_error_signed_deg;
    else
        angle_error_deg = angle_error_signed_deg;
}

void Warning_Update(void)
{
    if(angle_error_deg > ANGLE_ERROR_LIMIT_DEG)
    {
        angle_warning = 1;
        LED_On();
    }
    else
    {
        angle_warning = 0;
        LED_Off();
    }
}

//------------------------------------------ pid --------------------------------------
void PID_Control_Update(void)
{
    float error;
    float derivative;
    float control_pwm;

    error = (float)angle_error_signed_deg;

    /*
       Nếu sai số nhỏ hơn ngưỡng cho phép thì dừng motor.
       Ví dụ đặt 90 độ, motor tới 88-92 độ thì coi như đạt.
    */
    if(angle_error_deg <= ANGLE_TOLERANCE_DEG)
    {
        pid_integral = 0;
        pid_last_error = error;
        pid_output_pwm = 0;
        Motor_Stop();
        return;
    }

    pid_integral += error * PID_DT;

    if(pid_integral > 300.0f)
        pid_integral = 300.0f;

    if(pid_integral < -300.0f)
        pid_integral = -300.0f;

    derivative = (error - pid_last_error) / PID_DT;

    control_pwm = pid_kp * error
                + pid_ki * pid_integral
                + pid_kd * derivative;

    pid_last_error = error;

    if(control_pwm > PWM_MAX)
        control_pwm = PWM_MAX;

    if(control_pwm < -PWM_MAX)
        control_pwm = -PWM_MAX;

    /*
       Nếu PWM quá nhỏ motor không quay nổi,
       ép lên PWM_START.
    */
    if(control_pwm > 0.0f && control_pwm < PWM_START)
        control_pwm = PWM_START;

    if(control_pwm < 0.0f && control_pwm > -PWM_START)
        control_pwm = -PWM_START;

    pid_output_pwm = control_pwm;

    Motor_SetSignedPWM(control_pwm);
}
//----------------------------------------- interup ----------------------------------
void TIM4_10ms_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    TIM4->PSC = 8000 - 1;
    TIM4->ARR = 10 - 1;

    TIM4->CNT = 0;

    // Cho phép ngắt update
    TIM4->DIER |= TIM_DIER_UIE;

    // Cập nhật lại thanh ghi
    TIM4->EGR |= TIM_EGR_UG;

    // Bật ngắt TIM4 trong NVIC
    NVIC_SetPriority(TIM4_IRQn, 1);
    NVIC_EnableIRQ(TIM4_IRQn);

    // Bật timer
    TIM4->CR1 |= TIM_CR1_CEN;
}

void TIM4_IRQHandler(void)
{
    if(TIM4->SR & TIM_SR_UIF)
    {
        TIM4->SR &= ~TIM_SR_UIF;

        system_ms += 10;
        Motor_Button_Update();

        Angle_Update();
        PID_Control_Update();
        Warning_Update();

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
        if(flag_update_lcd)
        {
            flag_update_lcd = 0;

            if(!LCD_ShowAngle())
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

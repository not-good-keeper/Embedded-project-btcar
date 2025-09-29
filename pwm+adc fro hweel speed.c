#include "stm32f4xx.h"

#define POT_PIN      0    // PA0 ADC input
#define BTN_PIN      10   // PB10 toggle mode button

// PWM outputs for motors (ENA, ENB)
#define ENA_PIN      1    // PB1 = TIM3_CH4
#define ENB_PIN      0    // PB0 = TIM3_CH3

// Direction control inputs (Left motors replaced PC6,7 with PB4,5)
#define IN1_PIN      4    // PB4 (Left IN1)
#define IN2_PIN      5    // PB5 (Left IN2)

#define IN3_PIN      12   // PB12 (Right IN1)
#define IN4_PIN      13   // PB13 (Right IN2)

volatile uint32_t msTicks;

void SysTick_Handler(void) { msTicks++; }
static void Delay_ms(uint32_t ms) {
    uint32_t start = msTicks;
    while ((msTicks - start) < ms);
}

static void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    // PA0 analog for ADC
    GPIOA->MODER |= (3 << (POT_PIN*2));

    // PB0,PB1 AF2 for TIM3_CH3 and TIM3_CH4 PWM outputs
    GPIOB->MODER &= ~((3 << (ENA_PIN*2)) | (3 << (ENB_PIN*2)));
    GPIOB->MODER |= ((2 << (ENA_PIN*2)) | (2 << (ENB_PIN*2)));
    GPIOB->AFR[0] |= ((2 << (ENA_PIN*4)) | (2 << (ENB_PIN*4)));

    // PB4,PB5 digital outputs (direction left motor)
    GPIOB->MODER &= ~((3<<(IN1_PIN*2)) | (3<<(IN2_PIN*2)));
    GPIOB->MODER |= ((1<<(IN1_PIN*2)) | (1<<(IN2_PIN*2)));
    GPIOB->OTYPER &= ~((1<<IN1_PIN) | (1<<IN2_PIN));
    GPIOB->OSPEEDR |= ((2<<(IN1_PIN*2)) | (2<<(IN2_PIN*2)));

    // PB12,PB13 digital outputs (direction right motor)
    GPIOB->MODER &= ~((3<<(IN3_PIN*2)) | (3<<(IN4_PIN*2)));
    GPIOB->MODER |= ((1<<(IN3_PIN*2)) | (1<<(IN4_PIN*2)));
    GPIOB->OTYPER &= ~((1<<IN3_PIN) | (1<<IN4_PIN));
    GPIOB->OSPEEDR |= ((2<<(IN3_PIN*2)) | (2<<(IN4_PIN*2)));

    // PB10 input with pull-up (button)
    GPIOB->MODER &= ~(3 << (BTN_PIN*2));
    GPIOB->PUPDR &= ~(3 << (BTN_PIN*2));
    GPIOB->PUPDR |=  (1 << (BTN_PIN*2));
}

static void SysTick_Init(void) {
    SysTick->LOAD = 16000 - 1;  // 1 ms @16 MHz
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
}

static void ADC1_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC->CCR = (1 << 16);  // PCLK2/4 prescaler
    ADC1->SMPR2 |= (7 << (POT_PIN*3));  // 480 cycles
    ADC1->SQR1 &= ~ADC_SQR1_L;
    ADC1->SQR3 = POT_PIN;
    ADC1->CR2 |= ADC_CR2_ADON;
    Delay_ms(1);
}

static uint16_t ADC1_Read(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}

static void TIM3_PWM_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 15;      // 1 MHz timer clock
    TIM3->ARR = 999;     // 1 kHz PWM frequency
    TIM3->CCMR2 = (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE | (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;  // PWM on CH3, CH4
    TIM3->CCER = TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

int main(void) {
    GPIO_Init();
    SysTick_Init();
    ADC1_Init();
    TIM3_PWM_Init();

    const uint16_t rawMin = 2400, rawMax = 4091;
    uint8_t mode = 0;      // 0 = forward, 1 = reverse, 2 = off
    uint8_t lastBtn = 0;

    while (1) {
        // Button toggle on PB10 (active high, pull-up)
        uint8_t btn = (GPIOB->IDR >> BTN_PIN) & 1;
        if (btn && !lastBtn) {
            mode = (mode + 1) % 3;
            Delay_ms(50);
        }
        lastBtn = btn;

        // Read ADC and normalize PWM duty cycle 0–999
        uint16_t raw = ADC1_Read();
        uint32_t duty = 0;
        if (raw > rawMin) {
            duty = (raw >= rawMax) ? 999 : (uint32_t)(raw - rawMin) * 1000 / (rawMax - rawMin);
        }
        TIM3->CCR3 = duty;
        TIM3->CCR4 = duty;

        // Set directions based on mode
        if (mode == 0) {
            // Forward: Left IN1=1,IN2=0; Right IN3=1,IN4=0
            GPIOB->BSRR = (1 << IN1_PIN) | (1 << IN3_PIN);
            GPIOB->BSRR = (1 << (IN2_PIN + 16)) | (1 << (IN4_PIN + 16));
        }
        else if (mode == 1) {
            // Reverse: Left IN1=0,IN2=1; Right IN3=0,IN4=1
            GPIOB->BSRR = (1 << IN2_PIN) | (1 << IN4_PIN);
            GPIOB->BSRR = (1 << (IN1_PIN + 16)) | (1 << (IN3_PIN + 16));
        }
        else {
            // Off: all directions low, PWM 0
            GPIOB->BSRR = (1 << (IN1_PIN + 16)) | (1 << (IN2_PIN + 16)) | (1 << (IN3_PIN + 16)) | (1 << (IN4_PIN + 16));
            TIM3->CCR3 = 0;
            TIM3->CCR4 = 0;
        }
    }
}

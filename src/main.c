#include "stm32l432xx.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// Timing parameters for 100kHz with HSI16
#define PRESC   0x0
#define SCLDEL  0x4
#define SDADEL  0x2
#define SCLH    0x50
#define SCLL    0x3F

// SSD1306 7-bit slave address
#define SSD1306_ADDR 0x3C

// SSD1306 Control bytes
#define SSD1306_CONTROL_COMMAND         0x00
#define SSD1306_CONTROL_DATA            0x40

// SSD1306 Commands
#define SSD1306_COMMAND_DISPLAY_ON      0xAF
#define SSD1306_COMMAND_DISPLAY_OFF     0xAE
#define SSD1306_COMMAND_MUX_RATIO       0xA8
#define SSD1306_COMMAND_DISPLAY_OFFSET  0xD3
#define SSD1306_COMMAND_START_LINE      0x40
#define SSD1306_COMMAND_SEGMENT_REMAP   0xA1
#define SSD1306_COMMAND_COM_SCAN_DIR    0xC8
#define SSD1306_COMMAND_COM_PIN_CONFIG  0xDA
#define SSD1306_COMMAND_CONTRAST        0x81
#define SSD1306_COMMAND_RESUME          0xA4
#define SSD1306_COMMAND_OSC_FREQ        0xD5
#define SSD1306_COMMAND_CHARGE_PUMP     0x8D

// Additional SSD1306 Commands
#define SSD1306_COMMAND_ADDRESSING_MODE 0x20
#define SSD1306_COMMAND_COLUMN_ADDR     0x21
#define SSD1306_COMMAND_PAGE_ADDR       0x22

// Display dimensions
#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64
#define SSD1306_PAGES  8

//=============================================================================
// Initialization Functions
//=============================================================================

// Enable clocks for I2C1 and GPIOB (which will be used for PB6 and PB7)
void RCC_Init() {
    // Enable GPIOB clock (for PB6 and PB7)
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    // Enable I2C1 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;

    // Enable HSI16 if it is not already enabled
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {
        // Wait for HSI16 to be ready
    }

    // Select HSI16 as I2C1 clock source
    RCC->CCIPR &= ~RCC_CCIPR_I2C1SEL;
    RCC->CCIPR |= (0b10 << RCC_CCIPR_I2C1SEL_Pos);
}

// Configure PB6 (SCL) and PB7 (SDA) for I2C1
void GPIOB_Init() {
    // Configure PB6 and PB7 to alternate function mode (AF4 for I2C1)
    GPIOB->MODER &= ~(GPIO_MODER_MODE6);
    GPIOB->MODER &= ~(GPIO_MODER_MODE7);
    GPIOB->MODER |= (0b10 << GPIO_MODER_MODE6_Pos);
    GPIOB->MODER |= (0b10 << GPIO_MODER_MODE7_Pos);

    // Set alternate function AF4 for PB6 and PB7
    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFRL6);
    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFRL7);
    GPIOB->AFR[0] |= (0b0100 << 24);
    GPIOB->AFR[0] |= (0b0100 << 28);

    // Configure open-drain output
    GPIOB->OTYPER |= (GPIO_OTYPER_OT6);
    GPIOB->OTYPER |= (GPIO_OTYPER_OT7);

    // Enable pull-up resistors
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD7);
    GPIOB->PUPDR |= (0b01 << GPIO_PUPDR_PUPD6_Pos);
    GPIOB->PUPDR |= (0b01 << GPIO_PUPDR_PUPD7_Pos);

    // Set PB6 and PB7 to low speed (00 in OSPEEDR)
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR6);
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR7);
}

void I2C1_Init() {
    // Disable I2C1 first
    I2C1->CR1 &= ~I2C_CR1_PE;
    
    // Wait until I2C is disabled with timeout
    uint32_t timeout = 10000;
    while((I2C1->CR1 & I2C_CR1_PE) && --timeout);
    if(timeout == 0) return;
    
    // Software Reset
    I2C1->CR1 |= I2C_CR1_SWRST;
    delay_ms(1);
    I2C1->CR1 &= ~I2C_CR1_SWRST;
    
    // Configure timing
    I2C1->TIMINGR = (PRESC << I2C_TIMINGR_PRESC_Pos) | 
                    (SCLDEL << I2C_TIMINGR_SCLDEL_Pos) | 
                    (SDADEL << I2C_TIMINGR_SDADEL_Pos) | 
                    (SCLH << I2C_TIMINGR_SCLH_Pos) | 
                    (SCLL << I2C_TIMINGR_SCLL_Pos);
    
    // Enable analog filter and disable digital filter
    I2C1->CR1 &= ~I2C_CR1_ANFOFF;
    I2C1->CR1 &= ~I2C_CR1_DNF;
    
    // Enable error interrupts
    I2C1->CR1 |= I2C_CR1_ERRIE;
    
    // Enable I2C1
    I2C1->CR1 |= I2C_CR1_PE;
    
    // Wait until I2C is enabled with timeout
    timeout = 10000;
    while(!(I2C1->CR1 & I2C_CR1_PE) && --timeout);
}

bool I2C_Check_Init(void) {
    // Check if I2C is enabled
    if(!(I2C1->CR1 & I2C_CR1_PE)) {
        return false;
    }
    
    // Check if bus is not busy
    if(I2C1->ISR & I2C_ISR_BUSY) {
        return false;
    }
    
    return true;
}

// Configure an LED on PB4 for debug
void GPIOB_LED_Init(void) {
    // Enable GPIOB clock (for LED output on PB4)
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOBEN);
    // Configure PB4 as general purpose output
    GPIOB->MODER &= ~(0x3U << (4 * 2));
    GPIOB->MODER |=  (0x1U << (4 * 2));
    // Configure as push-pull and disable pull-ups/pull-downs
    GPIOB->OTYPER &= ~(1U << 4);
    GPIOB->PUPDR &= ~(0x3U << (4 * 2));
}

//=============================================================================
// Debug Functions
//=============================================================================

// Burn 1ms time
void delay_ms(uint32_t ms) {
    volatile uint32_t count;
    while (ms--) {
        count = 430;
        while (count--) {
            __NOP();
        }
    }
}

// 1s On, 1s Off, 2s total
void Blink_LED_PB4(void) {
    // Toggle LED on PB4
    GPIOB->ODR |= (1U << 4);
    delay_ms(1000);
    GPIOB->ODR &= ~(1U << 4);
    delay_ms(1000);
}

// Define test pin (PB3 to avoid conflict with I2C and LED)
#define TIMING_PIN_PORT    GPIOB
#define TIMING_PIN        3

void GPIOB_Timer_Init(void) {
    // Enable GPIOB clock if not already enabled
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    
    // Configure PB3 as general purpose output
    GPIOB->MODER &= ~(0x3U << (TIMING_PIN * 2));
    GPIOB->MODER |= (0x1U << (TIMING_PIN * 2));
    
    // Configure as push-pull and high speed
    GPIOB->OTYPER &= ~(1U << TIMING_PIN);
    GPIOB->OSPEEDR |= (0x3U << (TIMING_PIN * 2));  // High speed
    GPIOB->PUPDR &= ~(0x3U << (TIMING_PIN * 2));   // No pull-up/pull-down
}

/*
void Test_Delay_Timing(uint32_t ms) {
    // Set pin high before delay
    TIMING_PIN_PORT->BSRR = (1U << TIMING_PIN);
    
    // Execute delay
    delay_ms(ms);
    
    // Set pin low after delay
    TIMING_PIN_PORT->BRR = (1U << TIMING_PIN);
}


// Timing Test
int main(void) {
    GPIOB_LED_Init();
    GPIOB_Timer_Init();
    
    while (true) {
        // Test 1ms delay
        Test_Delay_Timing(1);
        // Wait
        delay_ms(10);
        
        // Test 10ms delay
        Test_Delay_Timing(10);
        // Wait
        delay_ms(10);
    }
    return 0;
}
*/

//=============================================================================
// Basic I2C Functions
//=============================================================================

void I2C_Start(void) {
    // Configure and send START condition
    I2C1->CR2 |= I2C_CR2_START;
    
    // Wait for start condition to be generated
    while(!(I2C1->ISR & I2C_ISR_BUSY));
}

void I2C_Stop(void) {
    // Generate STOP condition
    I2C1->CR2 |= I2C_CR2_STOP;
    
    // Wait for stop condition to complete
    while(I2C1->ISR & I2C_ISR_BUSY);
    
    // Clear STOP flag
    I2C1->ICR |= I2C_ICR_STOPCF;
}

void I2C_Write(uint8_t data) {
    // Wait until transmit buffer is empty
    while(!(I2C1->ISR & I2C_ISR_TXE));
    
    // Write data to transmit register
    I2C1->TXDR = data;
    
    // Wait for transfer to complete
    while(!(I2C1->ISR & I2C_ISR_TC));
}

uint8_t I2C_Read(void) {
    // Wait until receive buffer contains data
    while(!(I2C1->ISR & I2C_ISR_RXNE));
    
    // Read and return received data
    return (uint8_t)I2C1->RXDR;
}


// Helper function for address configuration
void I2C_SendAddress(uint8_t addr, uint8_t length, bool isRead) {
    I2C1->CR2 = 0;  // Clear CR2
    I2C1->CR2 = ((addr << 1) << I2C_CR2_SADD_Pos) |
                (length << I2C_CR2_NBYTES_Pos) |
                (isRead ? I2C_CR2_RD_WRN : 0);
}

// Reset the I2C peripheral
void I2C_ForceReset(void) {
    I2C1->CR1 |= I2C_CR1_SWRST;
    delay_ms(1);
    I2C1->CR1 &= ~I2C_CR1_SWRST;
}

// Clear all I2C status flags
void I2C_ClearFlags(void) {
    I2C1->ICR = 0xFFFFFFFF;
}

// Check if I2C bus is ready with timeout
bool I2C_WaitBusReady(uint32_t timeout) {
    while(I2C1->ISR & I2C_ISR_BUSY) {
        if(--timeout == 0) {
            I2C_ForceReset();
            return false;
        }
    }
    return true;
}

// Combined bus preparation function
bool I2C_PrepareBus(void) {
    I2C_ClearFlags();
    return I2C_WaitBusReady(10000);
}

// Send a transaction to addr, composed of data of length bytes
bool I2C_Transaction(uint8_t addr, uint8_t *data, uint8_t length) {
    if (!I2C_PrepareBus()) {
        return false;
    }
    
    // Configure the transfer
    I2C_SendAddress(addr, length, false);
    
    // Generate START condition
    I2C1->CR2 |= I2C_CR2_START;
    
    // Wait for START to be sent
    while(!(I2C1->ISR & I2C_ISR_TXIS) && !(I2C1->ISR & I2C_ISR_NACKF)) {
        if(I2C1->ISR & (I2C_ISR_BERR | I2C_ISR_ARLO)) {
            I2C_Stop();
            return false;
        }
    }
    
    // Check for NACK after address
    if(I2C1->ISR & I2C_ISR_NACKF) {
        I2C1->ICR |= I2C_ICR_NACKCF;
        I2C_Stop();
        return false;
    }
    
    // Send all data bytes
    for(uint8_t i = 0; i < length; i++) {
        // Wait for transmit buffer empty
        while(!(I2C1->ISR & I2C_ISR_TXIS)) {
            if(I2C1->ISR & I2C_ISR_NACKF) {
                I2C1->ICR |= I2C_ICR_NACKCF;
                I2C_Stop();
                return false;
            }
        }
        
        // Send byte
        I2C1->TXDR = data[i];
    }
    
    // Wait for transfer complete
    while(!(I2C1->ISR & I2C_ISR_TC)) {
        if(I2C1->ISR & (I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO)) {
            I2C_Stop();
            return false;
        }
    }
    
    // Generate STOP condition
    I2C1->CR2 |= I2C_CR2_STOP;
    
    return true;
}

//=============================================================================
// SSD1306 OLED I2C Functions
//=============================================================================

// Send a command byte to display
void SSD1306_SendCommand(uint8_t cmd) {
    uint8_t data[2] = {SSD1306_CONTROL_COMMAND, cmd};
    I2C_Transaction(SSD1306_ADDR, data, 2);
}

// Initialize display with the required startup sequence
void SSD1306_Init(void) {
    // Turn display off during initialization
    SSD1306_SendCommand(SSD1306_COMMAND_DISPLAY_OFF);
    
    // Set MUX Ratio
    SSD1306_SendCommand(SSD1306_COMMAND_MUX_RATIO);
    SSD1306_SendCommand(0x3F);
    
    // Set display offset
    SSD1306_SendCommand(SSD1306_COMMAND_DISPLAY_OFFSET);
    SSD1306_SendCommand(0x00);
    
    // Set start line
    SSD1306_SendCommand(SSD1306_COMMAND_START_LINE);

    // Set segment re-map
    SSD1306_SendCommand(SSD1306_COMMAND_SEGMENT_REMAP);

    // Set COM output scan direction
    SSD1306_SendCommand(SSD1306_COMMAND_COM_SCAN_DIR);

    // Set COM pin hardware configuration
    SSD1306_SendCommand(SSD1306_COMMAND_COM_PIN_CONFIG);
    SSD1306_SendCommand(0x12);

    
    // Set contrast
    SSD1306_SendCommand(SSD1306_COMMAND_CONTRAST);
    SSD1306_SendCommand(0x7F);

    
    // Resume to RAM content display
    SSD1306_SendCommand(SSD1306_COMMAND_RESUME);

    
    // Set oscillator frequency
    SSD1306_SendCommand(SSD1306_COMMAND_OSC_FREQ);
    SSD1306_SendCommand(0x80);

    // Enable charge pump
    SSD1306_SendCommand(SSD1306_COMMAND_CHARGE_PUMP);
    SSD1306_SendCommand(0x14);
    
    // Turn display on
    SSD1306_SendCommand(SSD1306_COMMAND_DISPLAY_ON);

}

// Enable or disable display
void SSD1306_Enable(bool on) {
    SSD1306_SendCommand(on ? SSD1306_COMMAND_DISPLAY_ON : SSD1306_COMMAND_DISPLAY_OFF);
}


// Set addressing mode (0x00 for horizontal)
void SSD1306_SetAddressingMode(uint8_t mode) {
    SSD1306_SendCommand(SSD1306_COMMAND_ADDRESSING_MODE);
    SSD1306_SendCommand(mode);
}

// Set column start and end address
void SSD1306_SetColumnAddress(uint8_t start, uint8_t end) {
    SSD1306_SendCommand(SSD1306_COMMAND_COLUMN_ADDR);
    SSD1306_SendCommand(start);
    SSD1306_SendCommand(end);
}

// Set page start and end address
void SSD1306_SetPageAddress(uint8_t start, uint8_t end) {
    SSD1306_SendCommand(SSD1306_COMMAND_PAGE_ADDR);
    SSD1306_SendCommand(start);
    SSD1306_SendCommand(end);
}

// Send display data
void SSD1306_SendData(uint8_t* data, uint16_t length) {
    uint8_t buffer[length + 1];
    buffer[0] = SSD1306_CONTROL_DATA;
    for(uint16_t i = 0; i < length; i++) {
        buffer[i + 1] = data[i];
    }
    I2C_Transaction(SSD1306_ADDR, buffer, length + 1);
}

// Display buffer - 8 pages, 128 bytes each
// Each byte represents 8 vertical pixels
static uint8_t displayBuffer[SSD1306_PAGES][SSD1306_WIDTH];

// Function to set a pixel in buffer
// x: 0-127
// y: 0-63
// color: true for on, false for off
// (0, 0) top left, (1127, 63) bottom right
void SSD1306_DrawToBuffer(uint8_t x, uint8_t y, bool color) {
    // Check bounds
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        return;
    }
    
    // Calculate which page the pixel is in
    uint8_t page = y / 8;
    
    // Calculate which bit in the page the pixel is
    uint8_t bit = y % 8;
    
    // Set or clear
    if (color) {
        displayBuffer[page][x] |= (1 << bit);  // Set
    } else {
        displayBuffer[page][x] &= ~(1 << bit); // Clear
    }
}

// Function to clear buffer black
void SSD1306_ClearBuffer(void) {
    memset(displayBuffer, 0, sizeof(displayBuffer));
}

// Function to fill buffer white
void SSD1306_FillBuffer(void) {
    memset(displayBuffer, 0xFF, sizeof(displayBuffer));
}

// Function to flush the buffer to display
void SSD1306_FlushBuffer(void) {
    SSD1306_SetAddressingMode(0x00); // Horizontal addressing mode
    SSD1306_SetColumnAddress(0, SSD1306_WIDTH - 1);
    SSD1306_SetPageAddress(0, SSD1306_PAGES - 1);
    
    // Send each page
    for (uint8_t page = 0; page < SSD1306_PAGES; page++) {
        SSD1306_SendData(displayBuffer[page], SSD1306_WIDTH);
    }
}

//=============================================================================
// Rotating Cube
//=============================================================================

// Testing
void DrawDiagonal(void) {
    for(int i; i < 64; i++) {
        SSD1306_DrawToBuffer(2*i, i, true);
        SSD1306_DrawToBuffer(2*i + 1, i, true);
    }
}

// Bresenham's
void DrawLine(int x0, int y0, int x1, int y1, bool color) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    while (true) {
        SSD1306_DrawToBuffer((uint8_t)x0, (uint8_t)y0, color);
        if (x0 == x1 && y0 == y1)
            break;
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

// 3D vector for cube vertices
typedef struct {
    float x, y, z;
} Vec3;

// Rotate the cube by angle on X, then Y
void RotateCube(float angle, Vec3 *outVertices) {
    // 8 vertices
    Vec3 cube[8] = {
        {-1, -1, -1},
        { 1, -1, -1},
        { 1,  1, -1},
        {-1,  1, -1},
        {-1, -1,  1},
        { 1, -1,  1},
        { 1,  1,  1},
        {-1,  1,  1}
    };
    
    for (int i = 0; i < 8; i++) {
        float x = cube[i].x;
        float y = cube[i].y;
        float z = cube[i].z;
        
        // Rotate around Y axis
        float xRot = x * cosf(angle) - z * sinf(angle);
        float zRot = x * sinf(angle) + z * cosf(angle);
        
        // Rotate around X axis
        float yRot = y * cosf(angle) - zRot * sinf(angle);
        float zFinal = y * sinf(angle) + zRot * cosf(angle);
        
        outVertices[i].x = xRot;
        outVertices[i].y = yRot;
        outVertices[i].z = zFinal;
    }
}

// Project a 3D point into 2D screen space using perspective projection
void ProjectVertex(const Vec3 *vertex, int *x2d, int *y2d) {
    // Translate cube along Z so it's in front of camera
    float distance = 3.0f; // Distance from camera to origin
    float z = vertex->z + distance;
    
    // f acts as a focal length
    float f = 64.0f;
    
    *x2d = (int)(SSD1306_WIDTH / 2 + (vertex->x * f) / z);
    *y2d = (int)(SSD1306_HEIGHT / 2 - (vertex->y * f) / z); // Invert y for screen coords
}


// Draw cube by drawing lines between its vertices
void DrawCube(float angle) {
    Vec3 vertices[8];
    RotateCube(angle, vertices);
    
    // Project 3D vertices to 2D screen coords
    int proj[8][2];
    for (int i = 0; i < 8; i++) {
        ProjectVertex(&vertices[i], &proj[i][0], &proj[i][1]);
    }
    
    // 12 edges as pairs of vertex indices
    int edges[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0}, // bottom face
        {4, 5}, {5, 6}, {6, 7}, {7, 4}, // top face
        {0, 4}, {1, 5}, {2, 6}, {3, 7}  // vertical edges
    };
    
    // Draw edges
    for (int i = 0; i < 12; i++) {
        int a = edges[i][0];
        int b = edges[i][1];
        DrawLine(proj[a][0], proj[a][1], proj[b][0], proj[b][1], true);
    }
}


//=============================================================================
// Main Function
//=============================================================================
int main(void) {
    // Initialize LED first for debug
    GPIOB_LED_Init();
    
    // Flash LED twice to show program started
    Blink_LED_PB4();
    Blink_LED_PB4();
    
    // Init stuff
    RCC_Init();
    GPIOB_Init();
    I2C1_Init();

    SSD1306_Init();

    SSD1306_ClearBuffer();
    SSD1306_FlushBuffer();

    float angle = 0.0f;

    // Update cube, draw to buffer, flush buffer
    while (true) {
        SSD1306_ClearBuffer();
        
        DrawCube(angle);
    
        SSD1306_FlushBuffer();
        angle += 0.05f;
        delay_ms(100);
    }

    return 0;
}

#include "usrMain.h"
#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "handles.h"
#include "math.h"

#define DRIVER_ID 106 // in range 101 to 199

#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))
#define abs(a) (((a)>0)?(a):-(a))

enum Mode {STOPPED, VELOCITY, POSITION, HOMING, HOMING_2};

#define UART_BUFFER_SIZE 128
volatile uint8_t uart_buffer[UART_BUFFER_SIZE] = "";
volatile uint8_t uart_message[UART_BUFFER_SIZE] = "";
volatile uint8_t uart_count = 0;
uint8_t uart_last_count = 0;
uint8_t uart_message_len = 0;

uint8_t buffer[100]; // buffer for sprintf
uint8_t count; //for sprintf

uint8_t command_error[] = "?\r";
uint8_t command_error_len = 2;

volatile uint32_t count_control, count_tick;

uint32_t intsqrt(uint32_t n)
{
    uint32_t c = 0x8000, g = 0x8000;
    for(;;)
    {
        if(g*g > n)
            g ^= c;
        c >>= 1;
        if(c == 0)
            return g;
        g |= c;
    }
}

#define NO_OF_MOTORS 3

#if NO_OF_MOTORS == 6
#define REPEAT(x) {x,x,x,x,x,x}
GPIO_TypeDef *NXT_Port[] = {NXT1_GPIO_Port, NXT2_GPIO_Port, NXT3_GPIO_Port, NXT4_GPIO_Port, NXT5_GPIO_Port, NXT6_GPIO_Port};
GPIO_TypeDef *DIR_Port[] = {DIR1_GPIO_Port, DIR2_GPIO_Port, DIR3_GPIO_Port, DIR4_GPIO_Port, DIR5_GPIO_Port, DIR6_GPIO_Port};
GPIO_TypeDef *CS_Port[] = {CS1_GPIO_Port, CS2_GPIO_Port, CS3_GPIO_Port, CS4_GPIO_Port, CS5_GPIO_Port, CS6_GPIO_Port};
GPIO_TypeDef *FLIMIT_Port[] = {FLIMIT1_GPIO_Port, FLIMIT2_GPIO_Port, FLIMIT3_GPIO_Port, FLIMIT4_GPIO_Port, FLIMIT5_GPIO_Port, FLIMIT6_GPIO_Port};
GPIO_TypeDef *RLIMIT_Port[] = {RLIMIT1_GPIO_Port, RLIMIT2_GPIO_Port, RLIMIT3_GPIO_Port, RLIMIT4_GPIO_Port, RLIMIT5_GPIO_Port, RLIMIT6_GPIO_Port};
const uint16_t NXT_Pin[] = {NXT1_Pin, NXT2_Pin, NXT3_Pin, NXT4_Pin, NXT5_Pin, NXT6_Pin};
const uint16_t DIR_Pin[] = {DIR1_Pin, DIR2_Pin, DIR3_Pin, DIR4_Pin, DIR5_Pin, DIR6_Pin};
const uint16_t CS_Pin[] = {CS1_Pin, CS2_Pin, CS3_Pin, CS4_Pin, CS5_Pin, CS6_Pin};
const uint16_t FLIMIT_Pin[] = {FLIMIT1_Pin, FLIMIT2_Pin, FLIMIT3_Pin, FLIMIT4_Pin, FLIMIT5_Pin, FLIMIT6_Pin};
const uint16_t RLIMIT_Pin[] = {RLIMIT1_Pin, RLIMIT2_Pin, RLIMIT3_Pin, RLIMIT4_Pin, RLIMIT5_Pin, RLIMIT6_Pin};
#endif

#if NO_OF_MOTORS == 3
#define REPEAT(x) {x,x,x}
GPIO_TypeDef *NXT_Port[] = {NXT1_GPIO_Port, NXT2_GPIO_Port, NXT3_GPIO_Port};
GPIO_TypeDef *DIR_Port[] = {DIR1_GPIO_Port, DIR2_GPIO_Port, DIR3_GPIO_Port};
GPIO_TypeDef *CS_Port[] = {CS1_GPIO_Port, CS2_GPIO_Port, CS3_GPIO_Port};
GPIO_TypeDef *FLIMIT_Port[] = {FLIMIT1_GPIO_Port, FLIMIT2_GPIO_Port, FLIMIT3_GPIO_Port};
GPIO_TypeDef *RLIMIT_Port[] = {RLIMIT1_GPIO_Port, RLIMIT2_GPIO_Port, RLIMIT3_GPIO_Port};
const uint16_t NXT_Pin[] = {NXT1_Pin, NXT2_Pin, NXT3_Pin};
const uint16_t DIR_Pin[] = {DIR1_Pin, DIR2_Pin, DIR3_Pin};
const uint16_t CS_Pin[] = {CS1_Pin, CS2_Pin, CS3_Pin};
const uint16_t FLIMIT_Pin[] = {FLIMIT1_Pin, FLIMIT2_Pin, FLIMIT3_Pin};
const uint16_t RLIMIT_Pin[] = {RLIMIT1_Pin, RLIMIT2_Pin, RLIMIT3_Pin};
#endif

#define TICK_PERIOD_NS (5000 * 2)
#define CONTROL_LOOP_FREQ 100
#define MICROSTEPS 64
#define default_step 1
#define default_acceleration_time 0.5
#define default_max_velocity 0
#define default_standard_velocity 0
#define default_hysteresis 0.0
#define default_limit_active_state 0
#define default_limit_type 0 // 0 - no switch, 2 - active shorted (active low), 3 - active disconnected, active high
uint8_t wr[NO_OF_MOTORS], cr0[NO_OF_MOTORS], cr1[NO_OF_MOTORS], cr2[NO_OF_MOTORS], cr3[NO_OF_MOTORS];
volatile enum Mode status[NO_OF_MOTORS];
volatile uint32_t clock[NO_OF_MOTORS], goal[NO_OF_MOTORS]; //in nanoseconds - clock and time when NXT pin state is to be changed
volatile uint8_t state[NO_OF_MOTORS], dir[NO_OF_MOTORS], last_dir[NO_OF_MOTORS]; // whether NXT pin state is high or low now
volatile int32_t current_position[NO_OF_MOTORS], target_position[NO_OF_MOTORS], real_position[NO_OF_MOTORS], target_real_position[NO_OF_MOTORS];
volatile float acceleration_time[NO_OF_MOTORS] = REPEAT(default_acceleration_time);
volatile float target_velocity[NO_OF_MOTORS];
volatile float current_velocity[NO_OF_MOTORS];
volatile float step[NO_OF_MOTORS] = REPEAT(default_step);
volatile float hysteresis[NO_OF_MOTORS] = REPEAT(default_hysteresis);
volatile int32_t hysteresis_ticks[NO_OF_MOTORS] = REPEAT(default_hysteresis / default_step * MICROSTEPS);
volatile float max_velocity[NO_OF_MOTORS] = REPEAT(default_max_velocity);
volatile float standard_velocity[NO_OF_MOTORS] = REPEAT(default_standard_velocity);
volatile uint8_t limit_active_state[NO_OF_MOTORS] = REPEAT(default_limit_active_state);
volatile uint8_t limit_type[NO_OF_MOTORS] = REPEAT(default_limit_type);

void writeReg(uint8_t motor, uint8_t address, uint8_t value)
{
    HAL_GPIO_WritePin(CS_Port[motor], CS_Pin[motor], 0);
    HAL_Delay(2);
    uint8_t addr = 0x80 | (address & 0b11111);
    uint8_t buf;
    HAL_SPI_TransmitReceive(&hspi1, &addr, &buf, 1, HAL_MAX_DELAY);
    HAL_SPI_TransmitReceive(&hspi1, &value, &buf, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_Port[motor], CS_Pin[motor], 1);
    HAL_Delay(2);
}

void applySettings(uint8_t motor)
{
    writeReg(motor, 0x3, cr2[motor]);
    writeReg(motor, 0x0, wr[motor]);
    writeReg(motor, 0x1, cr0[motor]);
    writeReg(motor, 0x2, cr1[motor]);
    writeReg(motor, 0x9, cr3[motor]);
}

void setMicrosteps(uint8_t motor, uint8_t mode)
{
    uint8_t esm = 0b000;
    uint8_t sm = 0b000;
    switch(mode)
    {
    case 1:
        esm = 0b011;
        break;
    case 2:
        sm = 0b100;
        break;
    case 4:
        sm = 0b011;
        break;
    case 8:
        sm = 0b010;
        break;
    case 16:
        sm = 0b001;
        break;
    case 32:
        sm = 0b000;
        break;
    case 64:
        esm = 0b010;
        break;
    case 128:
        esm = 0b001;
        break;
    }
    cr0[motor] = (cr0[motor] & ~0b11100000) | (sm << 5);
    cr3[motor] = (cr3[motor] & ~0b00000111) | esm;
}

void setCurrent(uint8_t motor, uint16_t current)
{
    uint8_t code = 0;
    if      (current >= 3000)
    {
        code = 0b11001;
    }
    else if (current >= 2845)
    {
        code = 0b11000;
    }
    else if (current >= 2700)
    {
        code = 0b10111;
    }
    else if (current >= 2440)
    {
        code = 0b10110;
    }
    else if (current >= 2240)
    {
        code = 0b10101;
    }
    else if (current >= 2070)
    {
        code = 0b10100;
    }
    else if (current >= 1850)
    {
        code = 0b10011;
    }
    else if (current >= 1695)
    {
        code = 0b10010;
    }
    else if (current >= 1520)
    {
        code = 0b10001;
    }
    else if (current >= 1405)
    {
        code = 0b10000;
    }
    else if (current >= 1260)
    {
        code = 0b01111;
    }
    else if (current >= 1150)
    {
        code = 0b01110;
    }
    else if (current >= 1060)
    {
        code = 0b01101;
    }
    else if (current >=  955)
    {
        code = 0b01100;
    }
    else if (current >=  870)
    {
        code = 0b01011;
    }
    else if (current >=  780)
    {
        code = 0b01010;
    }
    else if (current >=  715)
    {
        code = 0b01001;
    }
    else if (current >=  640)
    {
        code = 0b01000;
    }
    else if (current >=  585)
    {
        code = 0b00111;
    }
    else if (current >=  540)
    {
        code = 0b00110;
    }
    else if (current >=  485)
    {
        code = 0b00101;
    }
    else if (current >=  445)
    {
        code = 0b00100;
    }
    else if (current >=  395)
    {
        code = 0b00011;
    }
    else if (current >=  355)
    {
        code = 0b00010;
    }
    else if (current >=  245)
    {
        code = 0b00001;
    }
    cr0[motor] = (cr0[motor] & 0b11100000) | code;
}

void enableMotor(uint8_t motor)
{
    cr2[motor] |= 0b10000000;
}

void nxt_tick() // timer1 interrupt
{
    count_tick++;
    uint8_t motor;
    for (motor = 0; motor < NO_OF_MOTORS; motor++)
    {
        clock[motor] += TICK_PERIOD_NS;
        if (goal[motor] != 0 && clock[motor] > goal[motor])
        {
            clock[motor] %= goal[motor];
            if (state[motor])
            {
                state[motor] = 0;
                last_dir[motor] = dir[motor];
                HAL_GPIO_WritePin(NXT_Port[motor], NXT_Pin[motor], 0);
                HAL_GPIO_WritePin(DIR_Port[motor], DIR_Pin[motor], last_dir[motor]);
            }
            else if (status[motor] != STOPPED && !(status[motor] == POSITION && current_position[motor] - target_position[motor] == 0))
            {
                state[motor] = 1;
                if (!( limit_active_state[motor] == HAL_GPIO_ReadPin(RLIMIT_Port[motor], RLIMIT_Pin[motor]) &&
                       limit_active_state[motor] != HAL_GPIO_ReadPin(FLIMIT_Port[motor], FLIMIT_Pin[motor]) && last_dir[motor] && limit_type[motor])
                 && !( limit_active_state[motor] != HAL_GPIO_ReadPin(RLIMIT_Port[motor], RLIMIT_Pin[motor]) &&
                       limit_active_state[motor] == HAL_GPIO_ReadPin(FLIMIT_Port[motor], FLIMIT_Pin[motor]) && !last_dir[motor] && limit_type[motor]))
                {
                    HAL_GPIO_WritePin(NXT_Port[motor], NXT_Pin[motor], 1);
                    if(last_dir[motor])
                    {
                        current_position[motor]--;
                        if(current_position[motor] < real_position[motor])
                            real_position[motor] = current_position[motor];
                    }
                    else
                    {
                        current_position[motor]++;
                        if(current_position[motor] - hysteresis_ticks[motor] > real_position[motor])
                            real_position[motor] = current_position[motor] - hysteresis_ticks[motor];
                    }
                }
            }
        }
    }
}

void control_tick() // timer2 interrupt
{
    count_control++;
    uart_analyse_buffer();
    uint8_t motor;
    for (motor = 0; motor < NO_OF_MOTORS; motor++)
    {
        if (status[motor] == POSITION)
        {
            ////printf("c\n");
            target_velocity[motor] = standard_velocity[motor];
            target_velocity[motor] = min(standard_velocity[motor],
                                         intsqrt((float)(abs(current_position[motor] - target_position[motor])) / step[motor] * MICROSTEPS * 2 * standard_velocity[motor] / acceleration_time[motor]) * step[motor] / MICROSTEPS );
            if (current_position[motor] > target_position[motor])
                target_velocity[motor] *= -1;
            if (real_position[motor] == target_real_position[motor])
                status[motor] = STOPPED;
        }
        if (status[motor] == STOPPED)
        {
            target_velocity[motor] = current_velocity[motor] = 0;
        }
        else
        {
            if (target_velocity[motor] > current_velocity[motor])
                current_velocity[motor] = min(current_velocity[motor] + standard_velocity[motor] / acceleration_time[motor] / CONTROL_LOOP_FREQ, target_velocity[motor]);
            else
                current_velocity[motor] = max(current_velocity[motor] - standard_velocity[motor] / acceleration_time[motor] / CONTROL_LOOP_FREQ, target_velocity[motor]);
        }

        if (status[motor] == HOMING)
        {
            if (limit_active_state[motor] == (0 != HAL_GPIO_ReadPin(RLIMIT_Port[motor], RLIMIT_Pin[motor])))
            {
                status[motor] = HOMING_2;
                target_velocity[motor] = 0.05 * standard_velocity[motor];
            }
        }
        if (status[motor] == HOMING_2)
        {
            if (limit_active_state[motor] != (0 != HAL_GPIO_ReadPin(RLIMIT_Port[motor], RLIMIT_Pin[motor])))
            {
                status[motor] = STOPPED;
                target_velocity[motor] = current_velocity[motor] = 0;
                real_position[motor] = 0;
                target_real_position[motor] = 0;
                current_position[motor] = hysteresis_ticks[motor];
                target_position[motor] = hysteresis_ticks[motor];
            }
        }

        float value = current_velocity[motor] / step[motor] * MICROSTEPS;
        if (value < 1 && value > -1)
        {
            if(status[motor] == VELOCITY && target_velocity[motor] == 0)
                status[motor] = STOPPED;
            goal[motor] = 0;
        }
        else
        {
            if (value < 0)
            {
                value *= -1;
                dir[motor] = 1;
            }
            else
                dir[motor] = 0;
            goal[motor] = 1000000000 / value;
        }
    }
}

void uart_byte_received(uint8_t byte)
{
    uart_buffer[uart_count] = byte;
    uart_count++;
    uart_count %= UART_BUFFER_SIZE;
}

void uart_analyse_buffer()
{
    uint8_t uart_current_count = uart_count;
    uint8_t i = uart_last_count;

    while (i != uart_current_count)
    {
        uart_message[uart_message_len++] = uart_buffer[i++];
        i %= UART_BUFFER_SIZE;
        uart_last_count = i;
        uint8_t byte = uart_message[uart_message_len - 1];
        if (byte == '\r' && uart_message_len > 1)
        {
            uart_message[uart_message_len] = 0;
            uint8_t j = 0;

            while (uart_message[j] == ' ' || uart_message[j] == '\t' || uart_message[j] == '\r' || uart_message[j] == '\n') // trim whitespace
            {
                j++;
                if (j + 2 >= uart_message_len) // must be at least three characters left - two for command and one for endline
                {
                    uart_message_len = 0;
                    HAL_UART_Transmit(&huart2, (unsigned char*)"?\r", 2, 500);
                    return;
                }
            }

            uint8_t motor = 0;
            if(uart_message[j] >= '0' && uart_message[j] < '0' + NO_OF_MOTORS)  // motor number is specified
            {
                motor = uart_message[j] - '0';
                j++;
                while (uart_message[j] == ' ' || uart_message[j] == '\t' || uart_message[j] == '\r' || uart_message[j] == '\n') // trim following whitespace
                {
                    j++;
                    if (j + 2 >= uart_message_len) // must be at least three characters left - two for command and one for endline
                    {
                        uart_message_len = 0;
                        HAL_UART_Transmit(&huart2, (unsigned char*)"?\r", 2, 500);
                        return;
                    }
                }
            }

            float value = strtof((char *)(uart_message + j + 2), NULL); // try to read numerical value from part of buffer two bytes ahead

            if (uart_message[j] == 'h' && uart_message[j + 1] == 'm')
            {
                if(limit_type[motor] == 0)
                {
                    status[motor] = STOPPED;
                    current_position[motor] -= real_position[motor];
                    real_position[motor] = 0;
                }
                else
                {
                    target_velocity[motor] = -standard_velocity[motor]; // convert to pulses per second
                    status[motor] = HOMING;
                }
                count = sprintf((char*)buffer, "hm\r");
                HAL_UART_Transmit(&huart2, buffer, count, 500);
            }
            else if (uart_message[j] == 'm' && uart_message[j + 1] == 'v')
            {
                target_velocity[motor] = value;
                if(target_velocity[motor] > max_velocity[motor])
                    target_velocity[motor] = max_velocity[motor];
                else if (target_velocity[motor] < -max_velocity[motor])
                    target_velocity[motor] = -max_velocity[motor];
                status[motor] = VELOCITY;
                count = sprintf((char*)buffer, "mv\r");
                HAL_UART_Transmit(&huart2, buffer, count, 500);
            }
            else if (uart_message[j] == 's' && uart_message[j + 1] == 's')
            {
                step[motor] = value;
                status[motor] = STOPPED;
                hysteresis_ticks[motor] = hysteresis[motor] / step[motor] * MICROSTEPS;
                count = sprintf((char*)buffer, "ss\r");
                HAL_UART_Transmit(&huart2, buffer, count, 500);
            }
            else if (uart_message[j] == 's' && uart_message[j + 1] == 'v')
            {
                standard_velocity[motor] = value;
                count = sprintf((char*)buffer, "sv\r");
                HAL_UART_Transmit(&huart2, buffer, count, 500);
            }
            else if (uart_message[j] == 's' && uart_message[j + 1] == 'm')
            {
                max_velocity[motor] = value;
                count = sprintf((char*)buffer, "sm\r");
                HAL_UART_Transmit(&huart2, buffer, count, 500);
            }
            else if (uart_message[j] == 's' && uart_message[j + 1] == 'a')
            {
                acceleration_time[motor] = value;
                count = sprintf((char*)buffer, "sa\r");
                HAL_UART_Transmit(&huart2, buffer, count, 500);
            }
            else if (uart_message[j] == 's' && uart_message[j + 1] == 'l')
            {
                limit_type[motor] = value;
                limit_active_state[motor] = (value==3) ? 1 : 0;
                count = sprintf((char*)buffer, "sl\r");
                HAL_UART_Transmit(&huart2, buffer, count, 500);
            }
            else if ((uart_message[j] == 'm' && uart_message[j + 1] == 'a'))
            {
                target_real_position[motor] = value / step[motor] * MICROSTEPS;
                if(target_real_position[motor] < real_position[motor])
                    target_position[motor] = target_real_position[motor];
                else
                    target_position[motor] = target_real_position[motor] + hysteresis_ticks[motor];
                status[motor] = POSITION;
                count = sprintf((char*)buffer, "ma\r");
                HAL_UART_Transmit(&huart2, buffer, count, 500);
            }
            else if (uart_message[j] == 'm' && uart_message[j + 1] == 'r')
            {
                if (status[motor] == POSITION)
                    target_real_position[motor] += value / step[motor] * MICROSTEPS;
                else
                    target_real_position[motor] = real_position[motor] + value / step[motor] * MICROSTEPS;
                if(target_real_position[motor] < real_position[motor])
                    target_position[motor] = target_real_position[motor];
                else
                    target_position[motor] = target_real_position[motor] + hysteresis_ticks[motor];
                status[motor] = POSITION;
                count = sprintf((char*)buffer, "mr\r");
                HAL_UART_Transmit(&huart2, buffer, count, 500);
            }
            else if (uart_message[j] == 't' && uart_message[j + 1] == 'p')
            {
                count = sprintf((char*)buffer, "tp");
                float val = real_position[motor] * step[motor] / MICROSTEPS;
                if (val < 0)
                {
                    count += sprintf((char*)buffer + count, "-");
                    HAL_UART_Transmit(&huart2, buffer, count, 500);
                    val *= -1;
                }
                uint32_t int1 = val;
                count = sprintf((char*)buffer + count, "%d.%06d\r", (int)int1, (int)((val-int1) * 1000000));
                HAL_UART_Transmit(&huart2, buffer, count, 500);
            }
            else if (uart_message[j] == 't' && uart_message[j + 1] == 'a')
            {
                count = sprintf((char*)buffer, "ta");
                for (motor=0; motor < NO_OF_MOTORS; motor++)
                {
                    float val = real_position[motor] * step[motor] / MICROSTEPS;
                    if (val < 0)
                    {
                        count += sprintf((char*)buffer + count, "-");
                        val *= -1;
                    }
                    uint32_t int1 = val;
                    count += sprintf((char*)buffer + count, "%d.%06d ", (int)int1, (int)((val-int1) * 1000000));
                }
                for (motor=0; motor < NO_OF_MOTORS; motor++)
                    count += sprintf((char*)buffer + count, "%d", min(status[motor], 3));
                count += sprintf((char*)buffer + count, "\r");
                HAL_UART_Transmit(&huart2, buffer, count, 500);
            }
            else if (uart_message[j] == 'a' && uart_message[j + 1] == 'c')
            {
                count = sprintf((char*)buffer, "ac%d\r", NO_OF_MOTORS);
                HAL_UART_Transmit(&huart2, buffer, count, 500);
            }
            else if (uart_message[j] == 's' && uart_message[j + 1] == 'c')
            {
                uint16_t current = value;
                if (current > 0 && current <= 1500)
                {
                    setCurrent(motor, current);
                    applySettings(motor);
                }
                count = sprintf((char*)buffer, "sc\r");
                HAL_UART_Transmit(&huart2, buffer, count, 500);
            }
            else if (uart_message[j] == 's' && uart_message[j + 1] == 'h')
            {
                float loose = value;
                if (loose >= 0 && loose <= 1)
                {
                    hysteresis[motor] = loose;
                    hysteresis_ticks[motor] = loose / step[motor] * MICROSTEPS;
                }
                count = sprintf((char*)buffer, "sh\r");
                HAL_UART_Transmit(&huart2, buffer, count, 500);
            }
            else if (uart_message[j] == 't' && uart_message[j + 1] == 's')
            {
                count = sprintf((char*)buffer, "ts%d\r", min(status[motor], 3));
                HAL_UART_Transmit(&huart2, buffer, count, 500);
            }
            else if (uart_message[j] == 'r' && uart_message[j + 1] == 's')
            {
                count = sprintf((char*)buffer, "rsF%d R%d\r",
                                HAL_GPIO_ReadPin(FLIMIT_Port[motor], FLIMIT_Pin[motor]),
                                HAL_GPIO_ReadPin(RLIMIT_Port[motor], RLIMIT_Pin[motor]));
                HAL_UART_Transmit(&huart2, buffer, count, 500);
            }
            else if (uart_message[j] == 'i' && uart_message[j + 1] == 'd')
            {
                count = sprintf((char*)buffer, "id%d\r", DRIVER_ID);
                HAL_UART_Transmit(&huart2, buffer, count, 500);
            }
            /*else if (uart_message[j] == 'd' && uart_message[j + 1] == 'b')
            {
                count = sprintf((char*)buffer, "db curr:%d targ:%d loos:%d mposvel:%d currvel:%d real:%d goal:%d stat:%d\n\r",
                                (int)(current_position[motor]),
                                (int)(target_position[motor]),
                                (int)(loose_compensation_ticks[motor]),
                                (int)(1000*standard_velocity[motor]),
                                (int)(1000*current_velocity[motor]),
                                (int)(real_position[motor]),
                                (int)(goal[motor]),
                                (int)(status[motor]));
                HAL_UART_Transmit(&huart2, buffer, count, 500);
            }*/
            else
                HAL_UART_Transmit(&huart2, (unsigned char*)"?\r", 2, 500);

            uart_message_len = 0;
            return;
        }
    }
}

int usrMain()
{
    uint8_t motor;
    for (motor = 0; motor < NO_OF_MOTORS; motor++)
    {
        setMicrosteps(motor, MICROSTEPS);
        setCurrent(motor, 300);
        enableMotor(motor);
        applySettings(motor);
    }

    while(1)
    {
        HAL_Delay(1000);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
    }

    return 0;
}

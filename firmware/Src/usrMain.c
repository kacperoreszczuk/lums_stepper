#include "usrMain.h"
#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "handles.h"
#include "math.h"

/*
Token (last 40 bytes of configPhrase) will hopefully occur once in the
compiled binary. Finding it in binary will allow to change id config
byte without recompiling. First byte denotes hardware id (minus offset 
of 100). Second byte is reserved for XOR to maintain valid checksum 
after editing bin file.
*/

volatile uint8_t config_phrase[] =
    "\x08"   // ID (minus 100)
    "\x00"   // reserved, leave 0
    "uniquetoken45AD43383042B227439E97405EF5A";

uint8_t driver_id(){
    return config_phrase[0] + 100;
}

#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))
#define abs(a) (((a)>0)?(a):-(a))

enum Mode {STOPPED, VELOCITY, POSITION, HOMING, HOMING_2};

#define UART_BUFFER_SIZE 250
volatile uint8_t uart_buffer[UART_BUFFER_SIZE] = "";
volatile uint8_t uart_message[UART_BUFFER_SIZE] = "";
volatile uint8_t uart_tx_buffer[UART_BUFFER_SIZE] = "";
volatile uint16_t uart_tx_buffer_end = 0, uart_tx_buffer_begin = 0;
uint32_t uart_last_count = 0;
uint8_t uart_message_len = 0;

uint8_t buffer[100]; // buffer for sprintf
uint8_t count; //for sprintf

uint8_t command_error[] = "?\r";
uint8_t command_error_len = 2;
uint32_t counter = 0;
uint8_t limit_phase;

uint32_t t[10];

volatile uint32_t count_control, count_tick, counter2;
volatile uint32_t position_remainder;

#define NO_OF_MOTORS 3

#if NO_OF_MOTORS == 6
#define REPEAT(x) {x,x,x,x,x,x}
GPIO_TypeDef* const NXT_Port[] = {NXT1_GPIO_Port, NXT2_GPIO_Port, NXT3_GPIO_Port, NXT4_GPIO_Port, NXT5_GPIO_Port, NXT6_GPIO_Port};
GPIO_TypeDef* const DIR_Port[] = {DIR1_GPIO_Port, DIR2_GPIO_Port, DIR3_GPIO_Port, DIR4_GPIO_Port, DIR5_GPIO_Port, DIR6_GPIO_Port};
GPIO_TypeDef* const CS_Port[] = {CS1_GPIO_Port, CS2_GPIO_Port, CS3_GPIO_Port, CS4_GPIO_Port, CS5_GPIO_Port, CS6_GPIO_Port};
GPIO_TypeDef* const FLIMIT_Port[] = {FLIMIT1_GPIO_Port, FLIMIT2_GPIO_Port, FLIMIT3_GPIO_Port, FLIMIT4_GPIO_Port, FLIMIT5_GPIO_Port, FLIMIT6_GPIO_Port};
GPIO_TypeDef* const RLIMIT_Port[] = {RLIMIT1_GPIO_Port, RLIMIT2_GPIO_Port, RLIMIT3_GPIO_Port, RLIMIT4_GPIO_Port, RLIMIT5_GPIO_Port, RLIMIT6_GPIO_Port};
const uint16_t NXT_Pin[] = {NXT1_Pin, NXT2_Pin, NXT3_Pin, NXT4_Pin, NXT5_Pin, NXT6_Pin};
const uint16_t DIR_Pin[] = {DIR1_Pin, DIR2_Pin, DIR3_Pin, DIR4_Pin, DIR5_Pin, DIR6_Pin};
const uint16_t CS_Pin[] = {CS1_Pin, CS2_Pin, CS3_Pin, CS4_Pin, CS5_Pin, CS6_Pin};
const uint16_t FLIMIT_Pin[] = {FLIMIT1_Pin, FLIMIT2_Pin, FLIMIT3_Pin, FLIMIT4_Pin, FLIMIT5_Pin, FLIMIT6_Pin};
const uint16_t RLIMIT_Pin[] = {RLIMIT1_Pin, RLIMIT2_Pin, RLIMIT3_Pin, RLIMIT4_Pin, RLIMIT5_Pin, RLIMIT6_Pin};
#endif

#if NO_OF_MOTORS == 3
#define REPEAT(x) {x,x,x}
GPIO_TypeDef* const NXT_Port[] = {NXT1_GPIO_Port, NXT2_GPIO_Port, NXT3_GPIO_Port};
GPIO_TypeDef* const DIR_Port[] = {DIR1_GPIO_Port, DIR2_GPIO_Port, DIR3_GPIO_Port};
GPIO_TypeDef* const CS_Port[] = {CS1_GPIO_Port, CS2_GPIO_Port, CS3_GPIO_Port};
GPIO_TypeDef* const FLIMIT_Port[] = {FLIMIT1_GPIO_Port, FLIMIT2_GPIO_Port, FLIMIT3_GPIO_Port};
GPIO_TypeDef* const RLIMIT_Port[] = {RLIMIT1_GPIO_Port, RLIMIT2_GPIO_Port, RLIMIT3_GPIO_Port};
const uint16_t NXT_Pin[] = {NXT1_Pin, NXT2_Pin, NXT3_Pin};
const uint16_t DIR_Pin[] = {DIR1_Pin, DIR2_Pin, DIR3_Pin};
const uint16_t CS_Pin[] = {CS1_Pin, CS2_Pin, CS3_Pin};
const uint16_t FLIMIT_Pin[] = {FLIMIT1_Pin, FLIMIT2_Pin, FLIMIT3_Pin};
const uint16_t RLIMIT_Pin[] = {RLIMIT1_Pin, RLIMIT2_Pin, RLIMIT3_Pin};
#endif

#define TICK_PERIOD_NS (5200 * NO_OF_MOTORS)
uint8_t nxt_tick_phase = 0;
#define CONTROL_LOOP_FREQ 100
const float CONTROL_LOOP_PERIOD = 1.0f / CONTROL_LOOP_FREQ; // to avoid slow float division later
#define MICROSTEPS 64
const float MICROSTEP_SIZE = 1.0f / MICROSTEPS; // to avoid slow float division later
#define default_step 10.0f
#define default_acceleration_time_inv 3.0f
#define default_max_velocity 1000.0f
#define default_standard_velocity 500.0f
#define default_hysteresis 0.0f
uint8_t wr[NO_OF_MOTORS], cr0[NO_OF_MOTORS], cr1[NO_OF_MOTORS], cr2[NO_OF_MOTORS], cr3[NO_OF_MOTORS];
volatile enum Mode status[NO_OF_MOTORS];
volatile uint32_t clock[NO_OF_MOTORS], goal[NO_OF_MOTORS]; //in nanoseconds - clock and time when NXT pin state is to be changed
volatile uint8_t state[NO_OF_MOTORS], dir[NO_OF_MOTORS], last_dir[NO_OF_MOTORS]; // state - whether NXT pin state is high or low now
volatile int32_t current_position[NO_OF_MOTORS], target_position[NO_OF_MOTORS], real_position[NO_OF_MOTORS], target_real_position[NO_OF_MOTORS];
volatile float acceleration_time_inv[NO_OF_MOTORS] = REPEAT(default_acceleration_time_inv); // to avoid slow float division later
volatile float target_velocity[NO_OF_MOTORS];
volatile float current_velocity[NO_OF_MOTORS];
volatile float step[NO_OF_MOTORS] = REPEAT(default_step);
volatile float step_inv[NO_OF_MOTORS] = REPEAT(1.0f / default_step); // to avoid slow float division later
volatile float hysteresis[NO_OF_MOTORS] = REPEAT(default_hysteresis);
volatile int32_t hysteresis_ticks[NO_OF_MOTORS] = REPEAT(default_hysteresis / default_step * MICROSTEPS);
volatile float max_velocity[NO_OF_MOTORS] = REPEAT(default_max_velocity);
volatile float standard_velocity[NO_OF_MOTORS] = REPEAT(default_standard_velocity);
volatile uint8_t limit_active_state[NO_OF_MOTORS];
volatile uint8_t limit_enabled[NO_OF_MOTORS];
volatile uint8_t homing_enabled[NO_OF_MOTORS];
volatile uint8_t emergency_button[NO_OF_MOTORS];
volatile uint8_t reversed[NO_OF_MOTORS];
volatile float period[NO_OF_MOTORS];
volatile uint32_t period_ticks[NO_OF_MOTORS];
volatile uint8_t clone_axis[NO_OF_MOTORS];
volatile uint8_t homing_reversed[NO_OF_MOTORS];
volatile float homing_offset[NO_OF_MOTORS];
float limit_value_front[NO_OF_MOTORS] = REPEAT(0.5f);
float limit_value_rear[NO_OF_MOTORS] = REPEAT(0.5f);
volatile uint8_t limit_state_front[NO_OF_MOTORS];
volatile uint8_t limit_state_rear[NO_OF_MOTORS];
volatile uint8_t limit_state_home[NO_OF_MOTORS];
uint8_t limit_state_home_last[NO_OF_MOTORS];
volatile int32_t limit_on_position[NO_OF_MOTORS] = REPEAT(0x80000000);
volatile int32_t limit_off_position[NO_OF_MOTORS] = REPEAT(0x80000000);
const float limit_avg_remainder = 0.9f; // exponentially weighted mean filter
const float limit_avg_new = 0.1f;
const float limit_threshold_down = 0.2f; 
const float limit_threshold_up = 0.8f;

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
        code = 0b11001;
    else if (current >= 2845)
        code = 0b11000;
    else if (current >= 2700)
        code = 0b10111;
    else if (current >= 2440)
        code = 0b10110;
    else if (current >= 2240)
        code = 0b10101;
    else if (current >= 2070)
        code = 0b10100;
    else if (current >= 1850)
        code = 0b10011;
    else if (current >= 1695)
        code = 0b10010;
    else if (current >= 1520)
        code = 0b10001;
    else if (current >= 1405)
        code = 0b10000;
    else if (current >= 1260)
        code = 0b01111;
    else if (current >= 1150)
        code = 0b01110;
    else if (current >= 1060)
        code = 0b01101;
    else if (current >=  955)
        code = 0b01100;
    else if (current >=  870)
        code = 0b01011;
    else if (current >=  780)
        code = 0b01010;
    else if (current >=  715)
        code = 0b01001;
    else if (current >=  640)
        code = 0b01000;
    else if (current >=  585)
        code = 0b00111;
    else if (current >=  540)
        code = 0b00110;
    else if (current >=  485)
        code = 0b00101;
    else if (current >=  445)
        code = 0b00100;
    else if (current >=  395)
        code = 0b00011;
    else if (current >=  355)
        code = 0b00010;
    else if (current >=  245)
        code = 0b00001;

    cr0[motor] = (cr0[motor] & 0b11100000) | code;
}

void enableMotor(uint8_t motor)
{
    cr2[motor] |= 0b10000000;
}

inline void motor_tick_falling(const uint8_t motor)
{
    last_dir[motor] = dir[motor];
    HAL_GPIO_WritePin(NXT_Port[motor], NXT_Pin[motor], 0);
    if (clone_axis[motor])
        HAL_GPIO_WritePin(EIO_GPIO_Port, EIO_Pin, 0);
    HAL_GPIO_WritePin(DIR_Port[motor], DIR_Pin[motor], reversed[motor] != last_dir[motor]);
}

inline void motor_tick_rising(const uint8_t motor)
{
    clock[motor] += TICK_PERIOD_NS;
    if (goal[motor] != 0 && clock[motor] > goal[motor])
    {
        clock[motor] %= goal[motor];
        if (status[motor] != STOPPED && !(status[motor] == POSITION && 
            (/*(period_ticks[motor] && (current_position[motor] - target_position[motor]) % period_ticks[motor] == 0) ||
             (!period_ticks[motor] && */current_position[motor] - target_position[motor] == 0 )))//)
        {   
            if (!( limit_state_rear[motor] && last_dir[motor] && limit_enabled[motor])
             && !( limit_state_front[motor] && !last_dir[motor] && limit_enabled[motor]))
            {
                HAL_GPIO_WritePin(NXT_Port[motor], NXT_Pin[motor], 1);
                if (clone_axis[motor])
                    HAL_GPIO_WritePin(EIO_GPIO_Port, EIO_Pin, 1);
                if (last_dir[motor]) // logical XOR
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
                /*if (period_ticks[motor])
                {
                    if (current_position[motor] < 0)
                    {
                        current_position[motor] += period_ticks[motor];
                        real_position[motor] += period_ticks[motor];
                    }
                    else if (current_position[motor] > period_ticks[motor])
                    {
                        current_position[motor] -= period_ticks[motor];
                        real_position[motor] -= period_ticks[motor];
                    }
                } */
            }
        }
    }
}

inline void read_limit(const uint8_t motor)
{
    if (reversed[motor])
    {
        limit_value_rear[motor] = limit_avg_remainder * limit_value_rear[motor] + 
            limit_avg_new * (!limit_active_state[motor] == !HAL_GPIO_ReadPin(FLIMIT_Port[motor], FLIMIT_Pin[motor]));
        limit_value_front[motor] = limit_avg_remainder * limit_value_front[motor] + 
            limit_avg_new * (!limit_active_state[motor] == !HAL_GPIO_ReadPin(RLIMIT_Port[motor], RLIMIT_Pin[motor]));
    }
    else
    {
        limit_value_rear[motor] = limit_avg_remainder * limit_value_rear[motor] + 
            limit_avg_new * (!limit_active_state[motor] == !HAL_GPIO_ReadPin(RLIMIT_Port[motor], RLIMIT_Pin[motor]));
        limit_value_front[motor] = limit_avg_remainder * limit_value_front[motor] + 
            limit_avg_new * (!limit_active_state[motor] == !HAL_GPIO_ReadPin(FLIMIT_Port[motor], FLIMIT_Pin[motor]));
    }
    
    if (limit_value_rear[motor] < limit_threshold_down)
        limit_state_rear[motor] = 0;
    else if (limit_value_rear[motor] > limit_threshold_up)
        limit_state_rear[motor] = 1;

    if (limit_value_front[motor] > limit_threshold_up)
        limit_state_front[motor] = 1;
    else if (limit_value_front[motor] < limit_threshold_down)
        limit_state_front[motor] = 0;

    limit_state_home_last[motor] = limit_state_home[motor];
    if (homing_reversed[motor])
        limit_state_home[motor] = limit_state_front[motor];
    else
        limit_state_home[motor] = limit_state_rear[motor];

    if (limit_state_home_last[motor] && ! limit_state_home[motor]) // limit just was deactivated
        limit_off_position[motor] = current_position[motor]; 
    if (! limit_state_home_last[motor] && limit_state_home[motor]) // limit just was activated
        limit_on_position[motor] = current_position[motor];
}

void limit_switch_tick() // timer6 interrupt
{
    //HAL_GPIO_WritePin(EIO_GPIO_Port, EIO_Pin, 1);
    if (limit_phase == 0)
        read_limit(0);
    if (limit_phase == 1)
        read_limit(1);
    if (limit_phase == 2)
        read_limit(2);
    limit_phase = (limit_phase + 1) % NO_OF_MOTORS;
    //HAL_GPIO_WritePin(EIO_GPIO_Port, EIO_Pin, 0);
#if NO_OF_MOTORS != 3
    NOT IMPLEMENTED
#endif
}

void nxt_tick() // timer1 interrupt
{
    //count_tick++;
    if (nxt_tick_phase == 0)
    {
        motor_tick_falling(1);
        motor_tick_rising(0);
    }
    else if (nxt_tick_phase == 1)
    {
        motor_tick_falling(2);
        motor_tick_rising(1);
    }
    else if (nxt_tick_phase == 2)
    {
        motor_tick_falling(0);
        motor_tick_rising(2);
    }
    nxt_tick_phase = (nxt_tick_phase + 1) % 3;
    
#if NO_OF_MOTORS != 3
    NOT IMPLEMENTED
#endif
}

void control_tick() // timer2 interrupt
{
    t[0]=HAL_GetTick();

    uint32_t new_goal;
    uint8_t new_dir;
    uint8_t motor;

    count_control++;

    t[1]=HAL_GetTick();
    
    for (motor = 0; motor < NO_OF_MOTORS; motor++)
    {
        /*if (motor == 0)
        {
            count = sprintf((char*)buffer, "%i", status[motor]);
            HAL_UART_Transmit(&huart2, buffer, count, 500);
        }*/
        if (emergency_button[motor] && HAL_GPIO_ReadPin(EIO_GPIO_Port, EIO_Pin) == 0)
        {
            status[motor] = STOPPED;
        }

        if (status[motor] == POSITION)
        {
            /*if (period_ticks[motor])
            {
                target_velocity[motor] = min(standard_velocity[motor],
                    __builtin_sqrtf((float)(abs(abs(current_position[motor] - target_position[motor]) % period_ticks[motor] - period_ticks[motor] / 2)) 
                                    * 2 * standard_velocity[motor] * acceleration_time_inv[motor] * step[motor] * MICROSTEP_SIZE));
                if ((current_position[motor] - target_position[motor]) % period_ticks[motor] < period_ticks[motor] / 2)
                    target_velocity[motor] *= -1;
                if ((real_position[motor] - target_real_position[motor]) % period_ticks[motor] == 0)
                    status[motor] = STOPPED;
            }
            else*/
            {
                target_velocity[motor] = min(standard_velocity[motor],
                    __builtin_sqrtf((float)(abs(current_position[motor] - target_position[motor])) 
                                    * 2 * standard_velocity[motor] * acceleration_time_inv[motor] * step[motor] * MICROSTEP_SIZE));
                if (current_position[motor] > target_position[motor])
                    target_velocity[motor] *= -1;
                if (real_position[motor] == target_real_position[motor])
                    status[motor] = STOPPED;
            }
        }
        if (status[motor] == STOPPED)
        {
            target_velocity[motor] = current_velocity[motor] = 0;
        }
        else
        {
            if (target_velocity[motor] > current_velocity[motor])
                current_velocity[motor] = min(current_velocity[motor] + standard_velocity[motor] * acceleration_time_inv[motor] * CONTROL_LOOP_PERIOD, target_velocity[motor]);
            else
                current_velocity[motor] = max(current_velocity[motor] - standard_velocity[motor] * acceleration_time_inv[motor] * CONTROL_LOOP_PERIOD, target_velocity[motor]);
        }

        if (status[motor] == HOMING || status[motor] == HOMING_2)
        {
            if (status[motor] == HOMING)
            {
                if (limit_on_position[motor] != 0x80000000 || limit_state_home[motor] == 1)
                {
                    __disable_irq();
                    limit_off_position[motor] = 0x80000000;
                    __enable_irq();
                    status[motor] = HOMING_2;
                    target_velocity[motor] = 0;
                }
            }
            else
            {
                if (current_velocity[motor] == 0)
                {
                    limit_off_position[motor] = 0x80000000;
                    target_velocity[motor] = (homing_reversed[motor] ? -1 : 1) * 0.1f * standard_velocity[motor];
                }   
                else if (target_velocity[motor] != 0 && limit_off_position[motor] != 0x80000000) // else to introduce one control loop cycle delay
                {
                    __disable_irq();
                    status[motor] = POSITION;
                    real_position[motor] += -limit_off_position[motor] + homing_offset[motor] * step_inv[motor] * MICROSTEPS;
                    current_position[motor] = real_position[motor] + hysteresis_ticks[motor];
                    target_real_position[motor] = 0;
                    if(target_real_position[motor] < real_position[motor])
                        target_position[motor] = target_real_position[motor];
                    else
                        target_position[motor] = target_real_position[motor] + hysteresis_ticks[motor];
                    __enable_irq();
                }
            }
        }

        float value = current_velocity[motor] * step_inv[motor] * MICROSTEPS;
        if (value < 1 && value > -1)
        {
            if(status[motor] == VELOCITY && target_velocity[motor] == 0)
                status[motor] = STOPPED;
            new_goal = 0;
            new_dir = 0;
        }
        else
        {
            if (value < 0)
            {
                value *= -1;
                new_dir = 1;
            }
            else
                new_dir = 0;
            new_goal = 1000000000 / value;
        }
        __disable_irq();
        goal[motor] = new_goal;
        dir[motor] = new_dir;
        __enable_irq();
    }

    t[2] = HAL_GetTick();
    if (counter++ % 10 == 9)
    {
        /*count = sprintf((char*)buffer, "%d%d ",
                                   HAL_GPIO_ReadPin(FLIMIT_Port[0], FLIMIT_Pin[0]),
                                  HAL_GPIO_ReadPin(RLIMIT_Port[0], RLIMIT_Pin[0]),
                                   HAL_GPIO_ReadPin(FLIMIT_Port[1], FLIMIT_Pin[1]),
                                  HAL_GPIO_ReadPin(RLIMIT_Port[1], RLIMIT_Pin[1]),
                                   HAL_GPIO_ReadPin(FLIMIT_Port[2], FLIMIT_Pin[2]),
                                  HAL_GPIO_ReadPin(RLIMIT_Port[2], RLIMIT_Pin[2]));
        */     //HAL_UART_Transmit(&huart2, buffer, count, 500);
        //count = sprintf((char*)buffer, "on %i, off %i\r", t[1] - t[0], t[2] - t[1]);
        //count = sprintf((char*)buffer, "on %i, off %i, %i #\r", limit_on_position[0], limit_off_position[0], homing_offset[0]);
        //HAL_UART_Transmit(&huart2, buffer, count, 500);
        //count = sprintf((char*)buffer, "%u,%u ", (uint32_t)(limit_value_rear[0] * 1000), (uint32_t)(limit_value_front[0] * 1000));

        /*t[0]=HAL_GetTick();
        count = sprintf((char*)buffer, "asdfasdfuyasioyudfiaydfti8wqte876rthcx87qwe6crt87ah8rtx7awe6t8hdrt87ewawetrs8aswter876staw87e6rt87wawer");
        uart_transmit(buffer, count);
        t[1]=HAL_GetTick();
        count = sprintf((char*)buffer, " %i %i\r", t[1] - t[0], counter2);
        uart_transmit(buffer, count);*/
    }
}

void uart_byte_received(uint8_t byte)
{
    uart_buffer[uart_count] = byte;
    uart_count++;
    uart_count %= UART_BUFFER_SIZE;
}

void tx_cplt()
{
    tx_busy = 0;
    counter2 += 1;
    check_tx_buffer();
}

void check_tx_buffer()
{
    if (tx_busy)
        return;  // sending, this function will be executed again at tx finish callback
    uint8_t end = uart_tx_buffer_end;  // i make a copy in case of value change
    if (end > uart_tx_buffer_begin)  // ordinary send, no circular buffer overflow
    {
        tx_busy = 1;
        HAL_UART_Transmit_DMA(&huart2, (unsigned char*)(uart_tx_buffer + uart_tx_buffer_begin), end - uart_tx_buffer_begin);
        uart_tx_buffer_begin = end;
    }
    else if (end < uart_tx_buffer_begin)  // circular buffer overflow, now send only till the end of buffer
    {
        tx_busy = 1;
        HAL_UART_Transmit_DMA(&huart2, (unsigned char*)(uart_tx_buffer + uart_tx_buffer_begin), UART_BUFFER_SIZE - uart_tx_buffer_begin);
        uart_tx_buffer_begin = 0;
    }

}

void uart_transmit(uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart2, data, len, 500);
    return;
    uint8_t i;
    for (i = 0; i < len; i++)
    {
        uart_tx_buffer[uart_tx_buffer_end] = data[i];
        uart_tx_buffer_end = (uart_tx_buffer_end + 1) % UART_BUFFER_SIZE;
    }
    check_tx_buffer();
}

void uart_analyse_buffer()
{
    uint8_t uart_current_count;
    uint8_t i, limit_type;
    uint8_t motor = 0, result;
    uint16_t current;
    float loose;
    uint16_t command_signature;
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    uart_current_count = uart_count;
    i = uart_last_count;

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
                    uart_transmit(command_error, command_error_len);
                    //uart_analyse_buffer();
                    return;
                }
            }

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
                        uart_transmit(command_error, command_error_len);
                        //uart_analyse_buffer();
                        return;
                    }
                }
            }
            float value = strtof((char *)(uart_message + j + 2), NULL); // try to read numerical value from part of buffer two bytes ahead
            int8_t int_value = value + 0.5f;

            command_signature = 0x0100 * uart_message[j] + uart_message[j+1];

            switch(command_signature)
            {
                case 0x0100 * 'h' + 'm':
                    if(homing_enabled[motor] == 0)
                    {
                        status[motor] = STOPPED;
                        current_position[motor] += -real_position[motor] + homing_offset[motor] * step_inv[motor] * MICROSTEPS;
                        real_position[motor] = homing_offset[motor] * step_inv[motor] * MICROSTEPS;
                    }
                    else
                    {
                        __disable_irq();
                        limit_on_position[motor] = 0x80000000;
                        limit_off_position[motor] = 0x80000000;
                        limit_state_home_last[motor] = 0;
                        __enable_irq();
                        target_velocity[motor] = (homing_reversed[motor] ? -1 : 1) * (-standard_velocity[motor]); // convert to pulses per second
                        status[motor] = HOMING;
                    }
                    count = sprintf((char*)buffer, "hm\r");
                    uart_transmit(buffer, count);
                    break;
                case 0x0100 * 'h' + 'r':
                    homing_reversed[motor] = int_value;
                    count = sprintf((char*)buffer, "hr\r");
                    uart_transmit(buffer, count);
                    break;
                case 0x0100 * 'm' + 'v':
                    target_velocity[motor] = value;
                    if(target_velocity[motor] > max_velocity[motor])
                        target_velocity[motor] = max_velocity[motor];
                    else if (target_velocity[motor] < -max_velocity[motor])
                        target_velocity[motor] = -max_velocity[motor];
                    status[motor] = VELOCITY;
                    count = sprintf((char*)buffer, "mv\r");
                    uart_transmit(buffer, count);
                    break;
                case 0x0100 * 's' + 's':
                    step[motor] = value;
                    step_inv[motor] = 1.0f / value;
                    status[motor] = STOPPED;
                    hysteresis_ticks[motor] = hysteresis[motor] / step[motor] * MICROSTEPS;
                    //period_ticks[motor] = ((uint32_t)(period[motor] * step_inv[motor] + 0.5)) * MICROSTEPS;
                    count = sprintf((char*)buffer, "ss\r");
                    uart_transmit(buffer, count);
                    break;
                case 0x0100 * 's' + 'v':
                    standard_velocity[motor] = value;
                    count = sprintf((char*)buffer, "sv\r");
                    uart_transmit(buffer, count);
                    break;
                case 0x0100 * 's' + 'm':
                    max_velocity[motor] = value;
                    count = sprintf((char*)buffer, "sm\r");
                    uart_transmit(buffer, count);
                    break;
                case 0x0100 * 's' + 'a':
                    acceleration_time_inv[motor] = 1.0f / value;
                    count = sprintf((char*)buffer, "sa\r");
                    uart_transmit(buffer, count);
                    break;
                case 0x0100 * 's' + 'o':
                    homing_offset[motor] = value;
                    count = sprintf((char*)buffer, "so\r");
                    uart_transmit(buffer, count);
                    break;/*
                case 0x0100 * 's' + 'p':
                    period[motor] = value;
                    period_ticks[motor] = ((uint32_t)(period[motor] * step_inv[motor] + 0.5)) * MICROSTEPS;
                    count = sprintf((char*)buffer, "sp\r");
                    uart_transmit(buffer, count);
                    break;*/
                case 0x0100 * 'c' + 'a':  // undocumented feature, clones NXT pin of given axis to LD2 pin
                    for (j = 0; j < NO_OF_MOTORS; j++)
                        clone_axis[j] = 0;
                    clone_axis[motor] = 1;
                    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
                    GPIO_InitStruct.Pull = GPIO_NOPULL;
                    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
                    GPIO_InitStruct.Pin = EIO_Pin;
                    HAL_GPIO_Init(EIO_GPIO_Port, &GPIO_InitStruct);
                    count = sprintf((char*)buffer, "ca\r");
                    uart_transmit(buffer, count);
                    break;
                case 0x0100 * 's' + 'l': 
                    // 0 - no switch, 1 - active switch (high-active) 2 - active shorted, 
                    // 3 - active disconnected, 4,5,6 - like 2,3,1, but only for homing, 
                    limit_active_state[motor] = (int_value == 3 || int_value == 1 || int_value == 5 || int_value == 6) ? 1 : 0;
                    homing_enabled[motor] = (int_value == 1 || int_value == 2 || int_value == 3 || int_value == 4 || int_value == 5 || int_value == 6) ? 1 : 0;
                    limit_enabled[motor] = (int_value == 1 || int_value == 2 || int_value == 3) ? 1 : 0;
                    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
                    if (int_value == 2 || int_value == 3 || int_value == 4 || int_value == 5)
                        GPIO_InitStruct.Pull = GPIO_PULLUP;
                    else
                        GPIO_InitStruct.Pull = GPIO_NOPULL;
                    GPIO_InitStruct.Pin = FLIMIT_Pin[motor];
                    HAL_GPIO_Init(FLIMIT_Port[motor], &GPIO_InitStruct);
                    GPIO_InitStruct.Pin = RLIMIT_Pin[motor];
                    HAL_GPIO_Init(RLIMIT_Port[motor], &GPIO_InitStruct);

                    count = sprintf((char*)buffer, "sl\r");
                    uart_transmit(buffer, count);
                    break;
                case 0x0100 * 'm' + 'a':
                    target_real_position[motor] = value / step[motor] * MICROSTEPS;
                    /*if (period_ticks[motor])
                    {
                        if (target_real_position[motor] > 0)
                            target_real_position[motor] %= period_ticks[motor]
                        else
                            target_real_position[motor] = period_ticks[motor] - 1 - (-1 - target_real_position[motor]) % period_ticks[motor]
                    }*/
                    if(target_real_position[motor] < real_position[motor])
                        target_position[motor] = target_real_position[motor];
                    else
                        target_position[motor] = target_real_position[motor] + hysteresis_ticks[motor];
                    status[motor] = POSITION;
                    count = sprintf((char*)buffer, "ma\r");
                    uart_transmit(buffer, count);
                    break;
                case 0x0100 * 'm' + 'r':
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
                    uart_transmit(buffer, count);
                    break;
                case 0x0100 * 't' + 'p':
                    count = sprintf((char*)buffer, "tp");
                    int32_t pos = real_position[motor];
                    //if (period_ticks[motor] != 0)
                    //    pos = pos % period_ticks[motor];
                    float val = pos * step[motor] / MICROSTEPS;
                    if (val < 0)
                    {
                        count += sprintf((char*)buffer + count, "-");
                        val *= -1;
                    }
                    uint32_t int1 = val;
                    count += sprintf((char*)buffer + count, "%d.%06d\r", (int)int1, (int)((val-int1) * 1000000));
                    uart_transmit(buffer, count);
                    break;
                case 0x0100 * 't' + 'a':
                    count = sprintf((char*)buffer, "ta");
                    for (motor = 0; motor < NO_OF_MOTORS; motor++)
                    {
                        int32_t pos = real_position[motor];
                        /*if (period_ticks[motor] != 0)
                            pos = pos % period_ticks[motor];*/
                        float val = pos * step[motor] / MICROSTEPS;
                        if (val < 0)
                        {
                            count += sprintf((char*)buffer + count, "-");
                            val *= -1;
                        }
                        uint32_t int1 = val;
                        count += sprintf((char*)buffer + count, "%d.%06d ", (int)int1, (int)((val-int1) * 1000000));
                    }
                    for (motor = 0; motor < NO_OF_MOTORS; motor++)
                    {   
                        result = min(status[motor], 3);
                        if (emergency_button[motor] && !HAL_GPIO_ReadPin(EIO_GPIO_Port, EIO_Pin))
                            result = 4;
                        count += sprintf((char*)buffer + count, "%d", result);
                    }
                    count += sprintf((char*)buffer + count, "\r");
                    uart_transmit(buffer, count);
                    break;
                case 0x0100 * 'a' + 'c':
                    count = sprintf((char*)buffer, "ac%d\r", NO_OF_MOTORS);
                    uart_transmit(buffer, count);
                    break;
                case 0x0100 * 's' + 'c':
                    current = value;
                    if (current > 0 && current <= 1500)
                    {
                        setCurrent(motor, current);
                        applySettings(motor);
                    }
                    count = sprintf((char*)buffer, "sc\r");
                    uart_transmit(buffer, count);
                    break;
                case 0x0100 * 's' + 'h':
                    loose = value;
                    if (loose >= 0 && loose <= 1)
                    {
                        hysteresis[motor] = loose;
                        hysteresis_ticks[motor] = loose / step[motor] * MICROSTEPS;
                    }
                    count = sprintf((char*)buffer, "sh\r");
                    uart_transmit(buffer, count);
                    break;
                case 0x0100 * 't' + 's':
                    result = min(status[motor], 3);
                    if (emergency_button[motor] && !HAL_GPIO_ReadPin(EIO_GPIO_Port, EIO_Pin))
                        result = 4;
                    count = sprintf((char*)buffer, "ts%d\r", result);
                    uart_transmit(buffer, count);
                    break;
                case 0x0100 * 'r' + 's':
                    count = sprintf((char*)buffer, "rsF%d R%d\r",
                                    HAL_GPIO_ReadPin(FLIMIT_Port[motor], FLIMIT_Pin[motor]),
                                    HAL_GPIO_ReadPin(RLIMIT_Port[motor], RLIMIT_Pin[motor]));
                    uart_transmit(buffer, count);
                    break;
                case 0x0100 * 'i' + 'd':
                    count = sprintf((char*)buffer, "id%d\r", driver_id());
                    uart_transmit(buffer, count);
                    break;
                case 0x0100 * 's' + 'e':
                    emergency_button[motor] = (value != 0.0);
                    count = sprintf((char*)buffer, "se\r");
                    uart_transmit(buffer, count);
                    break;
                case 0x0100 * 's' + 'r':                  
                    reversed[motor] = (value != 0.0);
                    count = sprintf((char*)buffer, "sr\r");
                    uart_transmit(buffer, count);
                    break; 
                /*case 0x0100 * 'd' + 'b':
                    count = sprintf((char*)buffer, "db curr:%d targ:%d loos:%d mposvel:%d currvel:%d real:%d goal:%d stat:%d period: %d\n\r",
                                    (int)(current_position[motor]),
                                    (int)(target_position[motor]),
                                    (int)(hysteresis_ticks[motor]),
                                    (int)(1000*standard_velocity[motor]),
                                    (int)(1000*current_velocity[motor]),
                                    (int)(real_position[motor]),
                                    (int)(goal[motor]),
                                    (int)(status[motor]),
                                    (int)(period_ticks[motor]));
                    HAL_UART_Transmit(&huart2, buffer, count, 500);
                    break;*/
                default:
                    uart_transmit(command_error, command_error_len);
            }
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

    HAL_TIM_Base_Start_IT(&htim1); 
    HAL_TIM_Base_Start_IT(&htim2); 

    uint32_t swd_off_counter = 0;
    while(1)
    {
        HAL_Delay(1000);
    } 

    return 0;
}

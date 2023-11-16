#ifndef _POWER_MEASUER_H
#define _POWER_MEASUER_H

#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "stdint.h"

//使用软件IIC(使用硬件IIC时注释该行代码)
#define Use_software_i2c

//使用I2C1时，修改为hi2c1
#define Use_hi2c3
#define ina226_i2c hi2c3

#define Shunt_ohm 10 //单位毫欧
#define Curren_LSB 1 //单位mA，电流寄存器最多可以用15位即2^15

typedef struct
{
    uint16_t Config_Register;
    int16_t Shunt_Voltage_Register; //01h
    int16_t Bus_Voltage;
    uint16_t Power_Register;
    int16_t Current_Register;
    uint16_t Calibration_Register; // (05h)
    uint16_t ID;
} ina226_t_reg_t;

typedef struct
{
    float Bus_Voltage;   //1.25mv/msb
    float Shunt_Voltage; //2.5uv/msb
    float Power_cal_W;   //最后一个字母是单位
    float Power_read_W;
    float current_mA;
    float current_A;

    float Bus_Voltage_filter;//滤波后总线电压
    float Power_cal_W_first_order_filter;//一阶滤波后功率
    float Power_cal_W_window_filter;//窗口滤波后功率

} ina226_t;

#define CFG_REG 0x00 //

#define SV_REG 0x01 //分流电压， 此处分流电阻为 0.1欧

#define BV_REG 0x02 //总线电压

#define PWR_REG 0x03 //电源功率

#define CUR_REG 0x04 //电流

#define CAL_REG 0x05 //校准，设定满量程范围以及电流和功率测数的

#define ONFF_REG 0x06 //屏蔽 使能 警报配置和转换准备就绪

#define AL_REG 0x07 //包含与所选警报功能相比较的限定值

#define INA226_GET_ADDR 0XFF //包含唯一的芯片标识号


#define INA226_ADDR1 0x80
#define INA226_ADDR2 0x81
#define INA226_ADDR3 0x88

extern ina226_t ina226[3];

void INA226_Init(uint16_t Ina226_ID);
uint16_t INA226_Read_a_Reg(uint8_t reg, uint16_t Ina226_id);
void INA226_Read_Registers(uint16_t ina226_id);
#endif

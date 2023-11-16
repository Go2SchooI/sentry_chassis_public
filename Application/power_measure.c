/**
  ******************************************************************************
  * @file    power_measure
  * @author  Liu ZhiJie 
  * @brief   power_measure.
  ******************************************************************************
  * @attention
  * typedef struct
		{
			uint16_t  Config_Register;//00h配置寄存器
			
			int16_t  Shunt_Voltage_Register;//01h 2.5uv/msb
			
			int16_t	Bus_Voltage;//02h 1.25mv/msb
			
			uint16_t  Power_Register;//03h
			
			int16_t  Current_Register;//04h
			
			uint16_t  Calibration_Register;// (05h)
			
			uint16_t  ID;
		}ina226_t_reg_t;
  * 
  * 芯片手册中有表格可以查设备的地址，如果A0.A1引脚全部拉低则是0x0000
  * 主要就是calibration寄存器和电流和功率寄存器的关系
  * CAL =0.00512/Current_LSB*RSHUNT 比如你希望电流寄存器是1mA/bit然后采样电阻是10mΩ
  * 则CAL = 512即0x200h，这里的Current_LSB和RSHUNT都是以A和Ω为单位的。
  * 配置完了CAL寄存器，就可以读电流了。然后功率默认LSB是电流MSB的25倍。即电流是1mA/bit
  * 那么功率就是25mW/bit
  *
  *	
  ******************************************************************************
  */
#include "power_measure.h"
#include "cmsis_os.h"
#include "filter32.h"
#include "includes.h"
#include "core_cm4.h"
#include "bsp_i2c.h"

i2c_t inaIic; //创建IIC结构体

First_Order_Filter_t Ina226_0_Power;   //功率一阶滤波
Window_Filter_t Ina226_0_Vol;          //电压窗口滤波
Window_Filter_t Ina226_0_Power_Window; //功率窗口滤波

ina226_t_reg_t ina226_reg;
ina226_t ina226[3];

uint8_t id_buff[2];
uint8_t Votage_buff[2];
uint8_t reg_buff[7] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0xff};
uint16_t Receive_buff[7];
uint8_t INA226_I2C_Lost = 0;

uint32_t lost_count = 0;

static uint8_t INA_writeData(uint8_t dev, uint8_t reg, uint16_t data);
static uint8_t INA_readLen(uint8_t dev, uint8_t reg, uint8_t len, uint8_t *buf);

void INA226_Init(uint16_t Ina226_ID)
{

#ifdef Use_software_i2c //使用软件模拟IIC

#ifdef Use_hi2c3
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    inaIic.SCL_GPIO = GPIOA;
    inaIic.SCL_PIN = GPIO_PIN_8;
    inaIic.SDA_GPIO = GPIOC;
    inaIic.SDA_PIN = GPIO_PIN_9;
#endif

#ifdef Use_hi2c1
    //I2C1
    __HAL_RCC_GPIOB_CLK_ENABLE();
    inaIic.SCL_GPIO = GPIOB;
    inaIic.SCL_PIN = GPIO_PIN_8;
    inaIic.SDA_GPIO = GPIOB;
    inaIic.SDA_PIN = GPIO_PIN_9;
#endif

    IIC_Init(&inaIic); //IIC初始化
    INA_writeData(INA226_ADDR1, 0x00, 0X4207);
    INA_writeData(INA226_ADDR1, 0x05, 512);

#else //使用STM32硬件IIC

    uint8_t temp[2] = {0x42, 0x07};
    uint8_t cal[2];
    uint16_t CAL;

    CAL = 5120 / Curren_LSB / Shunt_ohm;
    cal[0] = CAL >> 8;
    cal[1] = CAL;
    HAL_I2C_Mem_Write(&ina226_i2c, Ina226_ID, CFG_REG, I2C_MEMADD_SIZE_8BIT, temp, 2, 0x10);
    HAL_I2C_Mem_Write(&ina226_i2c, Ina226_ID, CAL_REG, I2C_MEMADD_SIZE_8BIT, cal, 2, 0x10);
#endif
    //滤波初始化
    First_Order_Filter_Init(&Ina226_0_Power, 0.003, 0.01);
    Window_Filter_Init(&Ina226_0_Vol, 50);
    Window_Filter_Init(&Ina226_0_Power_Window, 50);
}

void INA226_Read_Registers(uint16_t ina226_id)
{

#ifdef Use_software_i2c //使用软件模拟IIC
    static uint8_t buff[2] = {0};
    //分线电压
    INA_readLen(INA226_ADDR1, SV_REG, 2, buff);
    ina226_reg.Shunt_Voltage_Register = (int16_t)(buff[0] << 8) | buff[1];
    //总线电压
    INA_readLen(INA226_ADDR1, BV_REG, 2, buff);
    ina226_reg.Bus_Voltage = (int16_t)(buff[0] << 8) | buff[1];
    //功率
    INA_readLen(INA226_ADDR1, PWR_REG, 2, buff);
    ina226_reg.Power_Register = (int16_t)(buff[0] << 8) | buff[1];
    //电流
    INA_readLen(INA226_ADDR1, CUR_REG, 2, buff);
    ina226_reg.Current_Register = (int16_t)(buff[0] << 8) | buff[1];
    //校准
    INA_readLen(INA226_ADDR1, CAL_REG, 2, buff);
    ina226_reg.Calibration_Register = (int16_t)(buff[0] << 8) | buff[1];
#else //使用STM32硬件IIC
    uint8_t i;
    for (i = 0; i < 7; i++)
    {
        Receive_buff[i] = INA226_Read_a_Reg(reg_buff[i], ina226_id);
    }
    ina226_reg.Config_Register = Receive_buff[0];
    ina226_reg.Shunt_Voltage_Register = Receive_buff[1];
    ina226_reg.Bus_Voltage = Receive_buff[2];
    ina226_reg.Power_Register = Receive_buff[3];
    ina226_reg.Current_Register = Receive_buff[4];
    ina226_reg.Calibration_Register = Receive_buff[5];
    ina226_reg.ID = Receive_buff[6];

#endif

    switch (ina226_id)
    {
    case INA226_ADDR1:
        ina226[0].Bus_Voltage = (float)ina226_reg.Bus_Voltage * 1.25f / 1000;
        ina226[0].Shunt_Voltage = (float)ina226_reg.Shunt_Voltage_Register * 2.5f / 1000; //单位mv满量程81.92mv
        ina226[0].Power_cal_W = ina226[0].Shunt_Voltage / Shunt_ohm * ina226[0].Bus_Voltage;
        ina226[0].Power_read_W = (float)ina226_reg.Power_Register * Curren_LSB * 25 / 1000;
        ina226[0].current_mA = ina226_reg.Current_Register * Curren_LSB;
        ina226[0].current_A = (float)ina226[0].current_mA / 1000;
        ina226[0].Bus_Voltage_filter = Window_Filter_Calculate(&Ina226_0_Vol, ina226[0].Bus_Voltage);
        ina226[0].Power_cal_W_first_order_filter = First_Order_Filter_Calculate(&Ina226_0_Power, ina226[0].Power_cal_W);
        ina226[0].Power_cal_W_window_filter = Window_Filter_Calculate(&Ina226_0_Power_Window, ina226[0].Power_cal_W);
        break;

    case INA226_ADDR2:
        ina226[1].Bus_Voltage = (float)ina226_reg.Bus_Voltage * 1.25f / 1000;
        ina226[1].Shunt_Voltage = (float)ina226_reg.Shunt_Voltage_Register * 2.5f / 1000; //单位mv满量程81.92mv
        ina226[1].Power_cal_W = ina226[1].Shunt_Voltage / Shunt_ohm * ina226[0].Bus_Voltage;
        ina226[1].Power_read_W = (float)ina226_reg.Power_Register * Curren_LSB * 25 / 1000;
        ina226[1].current_mA = ina226_reg.Current_Register * Curren_LSB;
        ina226[1].current_A = (float)ina226[1].current_mA / 1000;

        break;

    case INA226_ADDR3:
        ina226[2].Bus_Voltage = (float)ina226_reg.Bus_Voltage * 1.25f / 1000;
        ina226[2].Shunt_Voltage = (float)ina226_reg.Shunt_Voltage_Register * 2.5f / 1000; //单位mv满量程81.92mv
        ina226[2].Power_cal_W = ina226[2].Shunt_Voltage / Shunt_ohm * ina226[0].Bus_Voltage;
        ina226[2].Power_read_W = (float)ina226_reg.Power_Register * Curren_LSB * 25 / 1000;
        ina226[2].current_mA = ina226_reg.Current_Register * Curren_LSB;
        ina226[2].current_A = (float)ina226[2].current_mA / 1000;
        break;
    }
    if (lost_count > 30)
    {
        ina226[0].Bus_Voltage = 22.0f;
        ina226[0].Bus_Voltage_filter = 22.0f;
    }
}

//INA226读取寄存器（STM硬件IIC）
uint16_t INA226_Read_a_Reg(uint8_t reg, uint16_t Ina226_id)
{
    uint8_t temp[2];
    uint16_t value;
    static uint32_t lost_count = 0;
    if (HAL_I2C_Mem_Read(&ina226_i2c, Ina226_id, reg, I2C_MEMADD_SIZE_8BIT, temp, 2, 5) == HAL_OK)
    {
        //Detect_Hook(I2C_TOE);
        HAL_IWDG_Refresh(&hiwdg);
        lost_count = 0;
    }
    else
    {
        MX_I2C1_Init();
        MX_I2C3_Init();
        lost_count++;
        if (lost_count > 30)
        {
            Send_Motor_Current_1_4(&hcan1, 0, 0, 0, 0);
            HAL_NVIC_SystemReset();
        }
    }

    value = (temp[0] << 8) | temp[1];
    return value;
}

//INA226写入数据（软件模拟IIC）
static uint8_t INA_writeData(uint8_t dev, uint8_t reg, uint16_t data)
{

    IIC_Start(&inaIic);

    IIC_writeByte(&inaIic, dev);
    if (IIC_waitAck(&inaIic) == 1)
    {
        return 1;
    }

    IIC_writeByte(&inaIic, reg);
    if (IIC_waitAck(&inaIic) == 1)
    {
        return 2;
    }

    IIC_writeByte(&inaIic, data >> 8);
    if (IIC_waitAck(&inaIic) == 1)
    {
        return 3;
    }

    IIC_writeByte(&inaIic, data & 0xff);
    if (IIC_waitAck(&inaIic) == 1)
    {
        return 4;
    }

    IIC_Stop(&inaIic);
    return 0;
}
//INA266读取数据（软件模拟IIC）
static uint8_t INA_readLen(uint8_t dev, uint8_t reg, uint8_t len, uint8_t *buf)
{

    lost_count++;

    if (lost_count > 30)
    {
        INA226_Init(INA226_ADDR1);//重新初始化
    }

    IIC_Start(&inaIic);

    IIC_writeByte(&inaIic, dev);
    if (IIC_waitAck(&inaIic) == 1)
    {
        return 1;
    }

    IIC_writeByte(&inaIic, reg);
    if (IIC_waitAck(&inaIic) == 1)
    {
        return 2;
    }

    // IIC_Start(&inaIic);
    IIC_writeByte(&inaIic, dev + 1);
    if (IIC_waitAck(&inaIic) == 1)
    {
        return 3;
    }

    while (len)
    {
        if (len == 1)
        {
            *buf = IIC_readByte(&inaIic); //读数据,发送nACK
            IIC_ack(&inaIic, 1);
        }
        else
        {
            *buf = IIC_readByte(&inaIic); //读数据,发送ACK
            IIC_ack(&inaIic, 0);
        }

        len--;
        buf++;
    }

    IIC_Stop(&inaIic);
    
    lost_count = 0;
    return 0;
}

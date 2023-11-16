#ifndef CLIENT_INTERACT_H
#define CLIENT_INTERACT_H

#include "stm32f4xx_hal.h"
#include "string.h"
#include "stdarg.h"

typedef enum
{ // ����id����ʾ������������ͨ�Ż��ǿͻ��˻�ͼ
    ROBOT_COMMUICATE = 0x301,
    DEL_CHART_ID = 0x100,
    ROBOT_DRAW_CHART_1 = 0x101,
    ROBOT_DRAW_CHART_2 = 0x102,
    ROBOT_DRAW_CHART_5 = 0x103,
    ROBOT_DRAW_CHART_7 = 0x104,
    ROBOT_DRAW_CHARA = 0x110,
} robot_interact_content_e;

typedef enum
{
    //	ͼ�β���
    GRAPHIC_VOID = 0,   // �ղ���
    GRAPHIC_ADD = 1,    // ����
    GRAPHIC_MODIFY = 2, // �޸�
    GRAPHIC_DEL = 3,    // ɾ��

    //	ͼ������
    CLIENT_LINE = 0,        // ֱ��
    CLIENT_Rect = 1,        // ����
    CLIENT_CIRCULAR = 2,    // ��Բ
    CLIENT_Elliptical = 3,  // ��Բ
    CLIENT_CircularArc = 4, // Բ��
    CLIENT_Float = 5,       // ������
    CLIENT_Int = 6,         // ������
    CLIENT_CHAR = 7,        // �ַ�

    //	ͼ����ɫ
    CLIENT_RedOrBlue = 0, // ������ɫ
    CLIENT_Yellow = 1,    // ��ɫ
    CLIENT_GREEN = 2,     // ��ɫ
    CLIENT_Orange = 3,    // ��ɫ
    CLIENT_Amaranth = 4,  // �Ϻ�ɫ
    CLIENT_Pink = 5,      // ��ɫ
    CLIENT_Cyan = 6,      // ��ɫ
    CLIENT_Black = 7,     // ��ɫ
    CLIENT_White = 8,     // ��ɫA
} draw_graphic_e;

typedef __packed struct
{
    uint8_t Sof;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
    uint16_t cmd_id;
    uint16_t content_id;
    uint16_t sender_id;
    uint16_t receiver_id;
} frame_header_t;

typedef __packed struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye : 3;
    uint32_t graphic_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t start_angle : 9;
    uint32_t end_angle : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t radius : 10;
    uint32_t end_x : 11;
    uint32_t end_y : 11;
} graphic_data_struct_t;

// 1��ͼ��
typedef __packed struct
{
    frame_header_t frame_header;
    graphic_data_struct_t graphic;
    uint16_t frame_tail;
} client_frame_t;

// 2��ͼ��
typedef __packed struct
{
    frame_header_t frame_header;
    graphic_data_struct_t graphic[2];
    uint16_t frame_tail;
} client_frame_t_2;

// 5��ͼ��
typedef __packed struct
{
    frame_header_t frame_header;
    graphic_data_struct_t graphic[5];
    uint16_t frame_tail;
} client_frame_t_5;

// 7��ͼ��
typedef __packed struct
{
    frame_header_t frame_header;
    graphic_data_struct_t graphic[7];
    uint16_t frame_tail;
} client_frame_t_7;

// �ַ�
typedef __packed struct
{
    frame_header_t frame_header;
    graphic_data_struct_t graphic;
    uint8_t data[30];
    uint16_t frame_tail;
} client_char_t;

// ���ַ���
void client_draw_char(
    UART_HandleTypeDef *huart, client_char_t *client_char,
    uint8_t layer, uint8_t operate,
    uint16_t start_x, uint16_t start_y, uint8_t size,
    uint16_t sender, uint16_t receiver, uint8_t *buff, uint8_t buff_len, uint8_t color);

// void client_draw_char(
//     UART_HandleTypeDef *huart, client_frame_t *line,
//     uint8_t layer, uint8_t operate,
//     uint16_t start_x, uint16_t start_y, uint8_t size,
//     uint16_t sender, uint16_t receiver, uint8_t *buff, uint8_t buff_len, uint8_t color);
void client_draw_char_2(
    UART_HandleTypeDef *huart, client_frame_t *line,
    uint8_t layer, uint8_t operate,
    uint16_t start_x, uint16_t start_y, uint8_t size,
    uint16_t sender, uint16_t receiver, uint8_t *buff, uint8_t buff_len, uint8_t color);

typedef struct
{
    int16_t x;
    int16_t y;
    uint8_t zoomfactor;

} image_maps_t;

// ͼ�λ���
void client_draw(UART_HandleTypeDef *huart, uint8_t graphic_num, graphic_data_struct_t *graphic, uint16_t sender, uint16_t receiver);

// ֱ��
void Line_Generate(graphic_data_struct_t *line, uint8_t layer, uint8_t name, uint8_t operate, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y, uint8_t width, uint8_t color);
// ��Բ
void circular_Generate(graphic_data_struct_t *circular, uint8_t layer, uint8_t name, uint8_t operate, uint16_t center_x, uint16_t center_y, uint16_t radius, uint8_t width, uint8_t color);
// ����
void rect_Generate(graphic_data_struct_t *rect, uint8_t layer, uint8_t name, uint8_t operate, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y, uint8_t width, uint8_t color);

void Image_maps(image_maps_t *image_maps, int16_t x, uint16_t y, int16_t z);
#endif

#ifndef __SIM800C_H
#define __SIM800C_H

#include "stm32f4xx.h"
//�����Ƿ��������ж�  �����յ���������0x00������ѡ��0x01��Ϊû���ݽ���ı�־
#define NO_DATA             1
#define SEND_TYPE_NUM       2 //������������
#define SEND_CONNECT        0
#define WAIT_CONNECT        1
#define SEND_DATA           2
#define WAIT_SENDOK         3
#define CONNECT_CLEAR       4
#define SEND_TO_SIM800      USART2
#define RECEIVE_SIM800      USART3
void sim800c_main(void);

//++++++++++++++++++++++++����++++++++++++++++++++++++++

//�ַ����ȽϺ���
uint8_t compare_char(uint8_t *p1,uint8_t *contrast);//�ַ����ȽϺ���
//��ȡ���ݺ���
uint8_t deal_data(void);
//��������endsignal �����Ƿ�Ӻ�׺
void SendData(USART_TypeDef* USARTx,uint8_t *p,uint8_t endsignal);
//������һ��Ԫ�����
uint16_t get_next_number(uint16_t x,uint16_t max_val);
//�������ݳ���
uint16_t solve_data_length(uint16_t start,uint16_t end);
//���ƺ��������
void copy_clear_data(uint16_t start,uint16_t end);
//��������ת��Ϊ�ַ���
void data_convert2send(int datasend);
//�ж�SIM800���Ӵ���
uint8_t JudgeErrorClosed(void);
//++++++++++++++++++++++++����++++++++++++++++++++++++++
#endif
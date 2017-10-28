#include "sim800c.h"
#include "stm32f4xx_it.h"
#include "timer.h"
#include <stdlib.h>
#include<string.h>
#include "stm32f4xx_usart.h"
#include "gpio.h"


//++++++++++++++++++++++++����++++++++++++++++++++++++++

//�ж����ݴ�Ŷ���  500��ֹ��������ʾ��վ����
extern uint8_t Receive_Buffer[SIM800_BUFFER_SIZE]; 
//���ж϶�������ȡ�������ݳ���>2
uint8_t FeedBackData[300]={0};      
//TCP����������
unsigned char cipstart[]="AT+CIPSTART=\"TCP\",\"api.heclouds.com\",\"80\"\t";
//�ϴ���������
unsigned char cipsend[]="AT+CIPSEND\t";
//�������� cipdataheader������ͷ cipdata�����ݲ���
//unsigned char cipdataheader[]="POST /devices/19873426/datapoints HTTP/1.1\r\napi-key:TjrclfylUUXVhtjP8eGV64c4fcc=\r\nHost:api.heclouds.com\r\nContent-Length:66\r\n\r\n\t";
unsigned char cipdataheader[]="POST /devices/19873426/datapoints HTTP/1.1\r\napi-key:TjrclfylUUXVhtjP8eGV64c4fcc=\r\nHost:api.heclouds.com\r\nContent-Length:\t";
unsigned char cipdataheaderlength[20]={0};

unsigned char cipdatatop[]="{\"datastreams\":[{\"id\":\"temperature\",\"datapoints\":[{\"value\":\t";
unsigned char cipdataval[10]={0};
unsigned char cipdataend[]="}]}]}\t";

unsigned char cipdatatop1[]="{\"datastreams\":[{\"id\":\"temperature\",\"datapoints\":[{\"value\":\t";
unsigned char cipdataval1[10]={0};
unsigned char cipdataend1[]="}]}\t";

unsigned char cipdatatop2[]=",{\"id\":\"shidu\",\"datapoints\":[{\"value\":\t";
unsigned char cipdataval2[10]={0};
unsigned char cipdataend2[]="}]}\t";

unsigned char cipdatatop3[]=",{\"id\":\"voltage\",\"datapoints\":[{\"value\":\t";
unsigned char cipdataval3[10]={0};
unsigned char cipdataend3[]="}]}]}\t";

unsigned char stringtempe[10]={0};
unsigned char stringhumidity[10]={0};
unsigned char stringvoltage[10]={0};



//����ָ��
unsigned char cipat[]="AT\t";
//��¼����ɨ�����ʼ����λ��
uint16_t record_number=0; 
//0����δƥ��ɹ���1����ƥ��ɹ�
uint8_t CompareResult=0;
//�ȴ�connect ok ��־��Ϊ1�������ӳɹ� 
uint8_t WaitConnectOK=0;

//++++++++++++++++++++++++����++++++++++++++++++++++++++

extern uint8_t data_temperature[10];
extern uint8_t data_humidity[10];

unsigned char data_uint8_t_length=0;
int length_data=0;
extern long time_state_st;
extern int signal_count;


uint8_t state=0;	
uint8_t statelast=0;
void sim800c_main(void)
{
	//���͵�����
 	int datasend=0;
	int i=0;

		switch(state)
		{
			case SEND_CONNECT:
				SendData(RECEIVE_SIM800,"try connect\r\n\t",0);
				SendData(RECEIVE_SIM800,"try connect\r\n\t",0);
				SendData(RECEIVE_SIM800,"try connect\r\n\t",0);

				//����������������			
				SendData(SEND_TO_SIM800,cipstart,1);	
			    TIM_Delayms(TIM5, 500);
				SendData(RECEIVE_SIM800,"send already\r\n\t",0);
				state=WAIT_CONNECT;
				break;
	
			case WAIT_CONNECT:
				if(deal_data())
				{		
					CompareResult =compare_char(FeedBackData,"CONNECT OK");							
					if(CompareResult==1)
					{
						state=CONNECT_CLEAR;TIM_Delayms(TIM5, 100);//��ʱһ��ʱ��ȴ����ݽ��ܣ��ٽ������
						//������ʾ��Գɹ�����ʾ ��������UUU
						SendData(RECEIVE_SIM800,"UUU\r\n\t",0);						
					}
					else
					{
						CompareResult =compare_char(FeedBackData,"ALREADY CONNECT");
						if(CompareResult==1)
						{
							state=CONNECT_CLEAR;TIM_Delayms(TIM5, 100);//��ʱһ��ʱ��ȴ����ݽ��ܣ��ٽ������
							SendData(RECEIVE_SIM800,"WWW\r\n\t",0);
						}
						//�ж��Ƿ���ERROR CLOSED
						else if(JudgeErrorClosed())
						{
							state=SEND_CONNECT;
							SendData(RECEIVE_SIM800,"get error\r\n\t",0);
						}
						else
						{
							 SendData(RECEIVE_SIM800,"no use\r\n\t",0);
						}
					}
					i=0;
					while(FeedBackData[i])
					{
						USART_SendData(RECEIVE_SIM800, FeedBackData[i]);
						//�����ʾ����
						FeedBackData[i]=0;
						i++;
					}
				}
				else
				{
//					SendData(RECEIVE_SIM800,"no data\t",0);
				}
				break;
				
			case CONNECT_CLEAR:
				SendData(RECEIVE_SIM800,"clear\r\n\t",0);
				while(deal_data())
				{
					i=0;
					while(FeedBackData[i])
					{
						USART_SendData(RECEIVE_SIM800, FeedBackData[i]);
						//�����ʾ����
						FeedBackData[i]=0;
						i++;
					}				
				}	
				state=SEND_DATA;				
				SendData(RECEIVE_SIM800,"empty\r\n\t",0);
				
				break;
			case SEND_DATA:
				if(update_messege())
				{
					//���͵���������
					datasend++;
					//����ת��Ϊ�ַ������з���
					data_convert2send(datasend);
					
					SendData(SEND_TO_SIM800,cipsend,1);TIM_Delayms(TIM5, 100);	
					SendData(SEND_TO_SIM800,cipdataheader,0);
					SendData(SEND_TO_SIM800,cipdataheaderlength,0);
				
					SendData(SEND_TO_SIM800,cipdatatop1,0);
					SendData(SEND_TO_SIM800,data_temperature,0);
					SendData(SEND_TO_SIM800,cipdataend1,0);
				
					SendData(SEND_TO_SIM800,cipdatatop2,0);
					SendData(SEND_TO_SIM800,data_humidity,0);
					SendData(SEND_TO_SIM800,cipdataend2,0);
				
					SendData(SEND_TO_SIM800,cipdatatop3,0);
					SendData(SEND_TO_SIM800,cipdataval,0);
					SendData(SEND_TO_SIM800,cipdataend3,0);	
			

					USART_SendData(SEND_TO_SIM800, 0x1a);         //�����ֽ�
					BeefON();
					TIM_Delayms(TIM5, 100);
					BeefOFF();
					state=WAIT_SENDOK;	
					SendData(RECEIVE_SIM800,"already send data\r\n\t",0);TIM_Delayms(TIM5, 50);					
				}
				else
				{
					BeefON();TIM_Delayms(TIM5, 100);BeefOFF();TIM_Delayms(TIM5, 100);
					BeefON();TIM_Delayms(TIM5, 100);BeefOFF();TIM_Delayms(TIM5, 100);	
					SendData(RECEIVE_SIM800,"not update data\r\n\t",0);TIM_Delayms(TIM5, 50);
				}
				
					
				break;
			case WAIT_SENDOK:	
				if(deal_data())
				{								
					CompareResult =compare_char(FeedBackData,"SEND OK");
					if(CompareResult==1)
					{										
						state=CONNECT_CLEAR;
						SendData(RECEIVE_SIM800,"rece sendok\r\n\t",0);
					}
					else
					{
						//�ж��Ƿ���ERROR CLOSED
						if(JudgeErrorClosed())
						{
							SendData(RECEIVE_SIM800,"rece error\r\n\t",0);
							state=SEND_CONNECT;
						}
						else{
							SendData(RECEIVE_SIM800,"rece no use\r\n\t",0);
						}
					}					
					i=0;		
					while(FeedBackData[i])
					{
						USART_SendData(RECEIVE_SIM800, FeedBackData[i]);
						//�����ʾ����
						FeedBackData[i]=0;
						i++;
					}
				}			
				break;
			default :
				
				break;
		
		}
		if(statelast==state&&(state==WAIT_CONNECT||state==WAIT_SENDOK))
		{
			signal_count=1;
			if(time_state_st>910)
			{
				SendData(RECEIVE_SIM800,"\r\nreset\r\n\t",0);
				time_state_st=0;
				state=0;
			}
		}
		else
		{
			signal_count=0;
			time_state_st=0;
			statelast=state;
		}
//����ģ�ⷢ��ATָ��
//		if(KEY0==0)
//		{
//		SendData(cipstart,1);	TIM_Delayms(TIM5, 500);
//		}
//		if(KEY1==0)
//		{
//		SendData(cipsend,1);	TIM_Delayms(TIM5, 500);TIM_Delayms(TIM5, 500);TIM_Delayms(TIM5, 500);
//			
//					SendData(cipdataheader,0);	
//			SendData(cipdata,0);
//		USART_SendData(SEND_TO_SIM800, 0x1a);         //�򴮿�1��������
//			TIM_Delayms(TIM5, 500);
//		}



	
}


void SendData(USART_TypeDef* USARTx,uint8_t *p,uint8_t endsignal)
{
	//=1ʱ����  0x0d 0x0a
	//=0ʱ������
	int i=0;
	while(p[i]!='\t')
	{
		USART_SendData(USARTx, p[i]);         //�򴮿�1��������
	    i++;
	}
	if(endsignal)
	{
//		USART_SendData(USARTx, 0x0d);         //�򴮿�1��������
		USART_SendData(USARTx, 0x0d);         //�򴮿�1��������
		USART_SendData(USARTx, 0x0a);         //�򴮿�1��������
	}
}

uint8_t compare_char(uint8_t *p1,uint8_t *contrast)
{
	int i=0;
	while(*p1==0x01&&i<3)
	{
		p1++;
		i++;
	}

	while((*contrast<='~')&&(*contrast>=' '))
	{
		if((*p1<='~')&&(*p1>=' '))
		{
			if(*p1!=*contrast)
				return 0;
		}
		else
			return 0;
		p1++;
		contrast++;
	
	}
	return 1;
}


uint8_t deal_data(void)
{
	//uint16_t next_num=get_next_number(record_number,SIM800_BUFFER_SIZE);
	//record_number Ϊ��һ��0x0a��һ��λ�� ����һ�����ݵ�λ��
	uint16_t start_num=record_number;
	uint16_t end_num=get_next_number(record_number,SIM800_BUFFER_SIZE);
	uint16_t i=0;
	uint16_t data_length=0;
	if(Receive_Buffer[start_num]==NO_DATA)
		return 0;
	else if(Receive_Buffer[start_num]==0x0d||Receive_Buffer[start_num]==0x0a)
	{
		Receive_Buffer[start_num]=NO_DATA;
		record_number=get_next_number(record_number,SIM800_BUFFER_SIZE);
		return 0;			
	}
	else{			
			while(Receive_Buffer[end_num]!=NO_DATA)
			{
				if(Receive_Buffer[end_num]==0x0a)
				{
					//��Ҫ��������ȡ����+�����ݶ���������ɾ��
					data_length=solve_data_length(start_num,end_num);							
					if(data_length<3)
					{
						//������0x0d 0x0a����Ҫ������ʾ��ֻ��Ҫ��գ�ͬʱλ�����ơ�		
						for(i=start_num;i<start_num+data_length;i++)
						{
							Receive_Buffer[i%SIM800_BUFFER_SIZE]=NO_DATA;						
						}					
						record_number=get_next_number(end_num,SIM800_BUFFER_SIZE);
					}
					else{
						 //����ָ����ʼ�㣬��ֹ�����ݸ��ƣ�ͬʱ��������գ�ͬʱλ������
						copy_clear_data(start_num,end_num);
						record_number=get_next_number(end_num,SIM800_BUFFER_SIZE);
						return 1;
					}
				}
				end_num=get_next_number(end_num,SIM800_BUFFER_SIZE);			
			 }
	}
	return 0;

}
uint16_t get_next_number(uint16_t x,uint16_t max_val)
{
	if(x==max_val-1)
		return 0;
	else
		return (x+1);
}
uint16_t solve_data_length(uint16_t start,uint16_t end)
{
	uint16_t data_length=0;
	data_length=end>start?(end-start+1):(SIM800_BUFFER_SIZE-(start-end)+1);
	return data_length;
}
void copy_clear_data(uint16_t start,uint16_t end)
{
	int i=0;
    int data_num=0;
	if(end>start)
	{
		for(i=start;i<end+1;i++)
		{
			FeedBackData[data_num]=Receive_Buffer[i];//����
			Receive_Buffer[i]=NO_DATA;                     //ɾ��
			data_num++;	
		}
	}
	else
	{
		//�����ݿ�ʼ--500
		for(i=start;i<SIM800_BUFFER_SIZE;i++)
		{
			FeedBackData[data_num]=Receive_Buffer[i];//����
			Receive_Buffer[i]=NO_DATA;                     //ɾ��
			data_num++;	
		}
		//������0--���ݽ���
		for(i=0;i<end+1;i++)
		{
			FeedBackData[data_num]=Receive_Buffer[i];//����
			Receive_Buffer[i]=NO_DATA;                      //ɾ��
			data_num++;	
		}
	}

}



void data_convert2send(int datasend)
{
	char str[25]={0};
	int length_data=0;
	int length_temperature=0;
	int length_humidity=0;
	uint8_t i=0;
	uint8_t number=0;
	
	
    sprintf(str, " %d" , datasend);
	//��һ���ַ�����
	i=1;data_uint8_t_length=0;
	while(str[i])
	{
		cipdataval[data_uint8_t_length]=str[i];
		str[i]=0;
		data_uint8_t_length++;
		i++;
	}
	cipdataval[data_uint8_t_length]='\t';
	
	length_humidity=0;
	while(data_humidity[length_humidity]!='\t')
	{
		length_humidity++;
	}
	
	length_temperature=0;
	while(data_temperature[length_temperature]!='\t')
	{
		length_temperature++;
	}	
	

	
	
	
	
	length_data=strlen(cipdatatop1)-1+strlen(cipdataend1)-1+strlen(cipdatatop2)-1+strlen(cipdataend2)-1+strlen(cipdatatop3)-1+strlen(cipdataend3)-1;
	
	sprintf(str, " %d" , (data_uint8_t_length+length_temperature+length_humidity+length_data));
	i=1;
	number=0;
	while(str[i])
	{
		cipdataheaderlength[number]=str[i];
		str[i]=0;
		number++;
		i++;
	}
	cipdataheaderlength[number]='\r';number++;
	cipdataheaderlength[number]='\n';number++;
	cipdataheaderlength[number]='\r';number++;
	cipdataheaderlength[number]='\n';number++;
	cipdataheaderlength[number]='\t';number++;

}
uint8_t JudgeErrorClosed(void)
{
	int i=0;
	//�ж�ERROR
	CompareResult =compare_char(FeedBackData,"ERROR");	
	if(CompareResult==1)
	{
		return 1;
	}	
	//�ж�CLOSED
	CompareResult =compare_char(FeedBackData,"CLOSED");	
	if(CompareResult==1)
	{
		return 1;
	}
	return 0;
}










































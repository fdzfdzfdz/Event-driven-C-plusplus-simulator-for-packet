#include <deque>
#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <string.h>
#include <fstream>
#include "Node.h"
#include "Event.h"
#define MAXLEN 200    //MAXLEN=Bits_per_value+Type_bit+Numofnodes_bit+Packet_duration+3=118

using namespace std;

extern ofstream logs;

extern int Binary_to_decimal(int *Bit_array, int Start_bit, int End_bit);
extern void Decimal_to_binary(int *Bit_array, int Start_bit, int End_bit, int Decimal_number);
extern void Decimal_to_binary_string(char* string_name, int Maximum_bit, long Decimal_number);

extern int Preamble_bit;
extern int Type_bit;
extern int Numofnodes_bit;
extern int listen_period_bit;
extern int Bits_per_value;
extern int Packet_count;  

void Interleaving_packet(deque<TCPP_pair> &Queuing_system, int *arrayTotal, int arrayTotal_size, int type_decimal, 
	int Node_id, int Num_of_values_in_packet)
{
	Decimal_to_binary(arrayTotal, Preamble_bit, Preamble_bit+Type_bit-1, type_decimal);  //Type
	Decimal_to_binary(arrayTotal, Preamble_bit+Type_bit, Preamble_bit+Type_bit+Numofnodes_bit-1, Node_id);  //Tx_id
	//2015.7.25 Packet_duration
	//Decimal_to_binary(arrayTotal, Preamble_bit+Type_bit+Numofnodes_bit, Preamble_bit+Type_bit+Numofnodes_bit+Packet_duration-1, 
	//	Num_of_values_in_packet); //Packet_duration
	deque<TCPP_pair>::iterator TCPP_point;
	int ii=1;
	for(TCPP_point=Queuing_system.begin(), ii=1; ii<=Num_of_values_in_packet && TCPP_point!=Queuing_system.end(); ii++, TCPP_point++)
	{
		//2015.7.25 Packet_duration
		Decimal_to_binary(arrayTotal, Preamble_bit+Type_bit+Numofnodes_bit+(ii-1)*Bits_per_value, 
			Preamble_bit+Type_bit+Numofnodes_bit+ii*Bits_per_value-1, TCPP_point->TCPP);
	}

	//for(int i=0; i<arrayTotal_size; i++)   //just for debug
	//	cout<<arrayTotal[i]<<" ";
	//cout<<endl;

	//CRC
	////long, 32 bits
	//long i=static_cast<long>(Binary_to_decimal(arrayTotal, 0, Type_bit+Numofnodes_bit+Packet_duration+Num_of_values_in_packet*Bits_per_value+3-4)); 
    //long generate =13;   //1101
	//char* G_add=new char [MAXLEN];
	//char* S_end=new char [MAXLEN];
	//Decimal_to_binary_string(S_end, MAXLEN, i);
	//Decimal_to_binary_string(G_add, MAXLEN, generate);
	//char len_g = strlen(G_add);
	//i = i<<(len_g-1);
	//Decimal_to_binary_string(G_add, MAXLEN, i);
	//char len_s = strlen(G_add);  
	//long temp;  
	//while(len_s >= len_g)  
	//{  
	//   temp = generate<<(len_s-len_g);  
	//   i = i^temp;  
	//   Decimal_to_binary_string(G_add, MAXLEN, i);
	//   len_s = strlen(G_add);
	//}  
	////strcat_s(s_end, g_add);
	////cout<<"The CRC (binary): "<<G_add<<" at ";
	//for(unsigned int i=0; i<strlen(G_add); i++)
	//{
	//	arrayTotal[Type_bit+Numofnodes_bit+Packet_duration+Num_of_values_in_packet*Bits_per_value+3-(3-i)]=G_add[i]-48;
	//}
}

int Decoding_cluster(deque<TCPP_pair>& Queuing_system_temporary, int* TCPP_Data, int TCPP_Size, int Node_id, 
	long long Current_time)
{
	int N=0;
	if((TCPP_Size-Preamble_bit-Type_bit-Numofnodes_bit)%Bits_per_value!=0)   //just for test
		cout<<"Error code at Decoding_cluster Node-"<<Node_id<<" at "<<Current_time<<endl;

	//2015.7.25 Packet_duration
	int Num_of_values_in_packet=(TCPP_Size-Preamble_bit-Type_bit-Numofnodes_bit-Packet_count)/Bits_per_value;
	for(int i=1; i<=Num_of_values_in_packet; i++)
	{
		TCPP_pair element; N++;
		element.TCPP=Binary_to_decimal(TCPP_Data, Preamble_bit+Type_bit+Numofnodes_bit+(i-1)*Bits_per_value, 
			Preamble_bit+Type_bit+Numofnodes_bit+i*Bits_per_value-1);
		element.Time_in=Current_time;
		Queuing_system_temporary.push_back(element);
	}
	return N;
}

void Singal_construction(int* Signal, int Signal_size, int type_decimal, int transmitter_decimal, int receiver_decimal, 
	int listen_period_decimal, int Num_of_values_in_packet)
{
	if(type_decimal==0)  //SYNC
	{//type=0
		Decimal_to_binary(Signal, Preamble_bit+Type_bit, Preamble_bit+Type_bit+Numofnodes_bit-1, transmitter_decimal);
		Decimal_to_binary(Signal, Preamble_bit+Type_bit+Numofnodes_bit, Preamble_bit+Type_bit+Numofnodes_bit+listen_period_bit-1, listen_period_decimal);
	}
	if(type_decimal==1)  //RTS
	{
		Decimal_to_binary(Signal, Preamble_bit, Preamble_bit+Type_bit-1, type_decimal);
		Decimal_to_binary(Signal, Preamble_bit+Type_bit, Preamble_bit+Type_bit+Numofnodes_bit-1, transmitter_decimal);
		Decimal_to_binary(Signal, Preamble_bit+Type_bit+Numofnodes_bit, Preamble_bit+Type_bit+2*Numofnodes_bit-1, receiver_decimal);
		//Decimal_to_binary(Signal, Preamble_bit+Type_bit+2*Numofnodes_bit, Signal_size-1, Num_of_values_in_packet);   //2015.7.25 Packet_duration
	}
	if(type_decimal==2)  //CTS
	{
		Decimal_to_binary(Signal, Preamble_bit, Preamble_bit+Type_bit-1, type_decimal);
		Decimal_to_binary(Signal, Preamble_bit+Type_bit, Preamble_bit+Type_bit+Numofnodes_bit-1, transmitter_decimal);
		//Decimal_to_binary(Signal, Preamble_bit+Type_bit+Numofnodes_bit, Signal_size-1, Num_of_values_in_packet);     //2015.7.25 Packet_duration
	}
	if(type_decimal==4)  //ACK
	{
		Decimal_to_binary(Signal, Preamble_bit, Preamble_bit+Type_bit-1, type_decimal);
		Decimal_to_binary(Signal, Preamble_bit+Type_bit, Preamble_bit+Type_bit+Numofnodes_bit-1, transmitter_decimal);
	}

	//for(int i=0; i<Signal_size; i++)   //just for debug
	//	cout<<Signal[i]<<" ";
	//cout<<endl;
}
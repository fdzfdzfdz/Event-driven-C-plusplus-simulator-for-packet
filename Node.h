#pragma once
#include <deque>
#include "Event.h"
#include <list>
#include <ctime>
#include <cmath>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <fstream>

using namespace std;

class Node
{
private:
	int Node_id;
	int Xaxis;
	int Yaxis;
	long long Timer_last_bit;
	int Back_off_coeff;                //back-off coefficient
	int Backoff_time;
	int Interleaving_num;              //indicate the number of interleaving in current Cluster
	std::deque<TCPP_pair> Queuing_system;    
	int RTS_CTS_buffer[10];
	int Transmitter_id;                //indicate it is transmitter in this round
	int Receiver_id;                   //indicate it is receiver in this round
	Event* Timer_event;                //point to the timer event
	int Neighbor_table[10];            //indicate the all the neighbors
	long long Time_of_bo;              //the moment that the node starts back-off
	long long Temp_of_TCPP_time;        
	int       Temp_of_TCPP;

	bool response_SYNC_control;
	long long Time_radio_on;
	Event* Timer_event_radio_off;
	bool radio_on;                //use time to control the first time synchronization
	long long Timer_normal_periodic_SYNC;
	//long long Time_sleep;
	int Schedule_table_node_id[5];
	long long Schedule_table_sleep_time[5];
	int SYNC_buffer[10];
	int Num_of_values_in_packet;

	long long Total_num_tran_TCPP;
	long long Succe_num_tran_TCPP;
	long long Total_num_rece_TCPP;
	long long Total_time_radio_on;
	long long Total_time_interval;
	long long Data_tran_interval;
	long long Loop_waiting_delay;      
	long long Loop_access_delay;
	long long Waiting_delay;
	long long Access_delay;
	//long long Loop_delay;
	//long long Latency;
	long long Total_RTS;
	long long Total_CTS;
	long long Total_ACK;
	long long Pulse_count;
	long long Pulse_count_non_SYNC;
	long long SYNC_time_radio_on;
	long long Trans_finish_time;      //use to calculate Data transmission rate
	long long Total_times_tran;       
	long long Succe_timers_tran;
	long long Total_time_receiving;
	long long SYNC_time_receiving;
	long long Radio_off_time;
	long long Last_moment_receiving;

	long long Total_SYNC;
	/*long long Loop_tran_bits;
	long long Loop_all_signals_interval_bits;
	long long loop_data_interval_bits;*/

	//long long Total_transmission;     //use to calculate the times of sending data
	//long long TCPP_transmitted;
	//long long Num_of_retransmission;
	//long long Num_of_TCPP_retransmitted;
	//long long TCPP_received;           //total received TCPP, when retransmission, count them as twice
	
	//long long Loss_of_TCPP;
	//long long Loss_of_times;

	//long long Acknowledged_RTS;
	//long long Acknowledged_CTS;
	//bool Time_of_RTS_Sending_controller;
	//long long Num_of_Interleaving;    //use to calculate Average number of interleaved PPCPs
public:
	Node(void);
	~Node(void);
	void setInterleaving_num(int i){Interleaving_num=i; }
	int  getInterleaving_num(){return Interleaving_num; }

	void setNode_id(int i){Node_id=i; }
	int  getNode_id(){return Node_id; }

	int getXaxis(){return Xaxis; }
	int getYaxis(){return Yaxis; }
	void setXaxis_Yaxis(int i, int j){Xaxis=i; Yaxis=j;}

	size_t getQueuing_system_size(){return Queuing_system.size(); }

	long long  getTotal_num_tran_TCPP(){return Total_num_tran_TCPP; }
	long long  getSucce_num_tran_TCPP(){return Succe_num_tran_TCPP; }
	long long  getTotal_num_rece_TCPP(){return Total_num_rece_TCPP; }
	long long  getTotal_time_radio_on(){return Total_time_radio_on; }
	long long  getTotal_time_interval(){return Total_time_interval; }
	long long  getData_tran_interval(){return Data_tran_interval; }

	long long  getTotal_times_tran(){return Total_times_tran; }
	long long  getSucce_timers_tran(){return Succe_timers_tran; }
	long long  getTotal_time_receiving(){return Total_time_receiving; }
	long long  getSYNC_time_receiving(){return SYNC_time_receiving; }

	/*void setLatency(long long t){Latency=t; }
	long long getLatency(){return Latency; }

	void setLoop_delay(long long i){Loop_delay=i; }
	long long getLoop_delay(){return Loop_delay; }*/

	void setWaiting_delay(long long t){Waiting_delay=t; }
	long long getWaiting_delay(){return Waiting_delay; }

	void setAccess_delay(long long i){Access_delay=i; }
	long long getAccess_delay(){return Access_delay; }

	void setLoop_waiting_delay(long long i){Loop_waiting_delay=i; }
	long long getLoop_waiting_delay(){return Loop_waiting_delay; }

	void setLoop_access_delay(long long i){Loop_access_delay=i; }
	long long getLoop_access_delay(){return Loop_access_delay; }

	void setTime_of_bo(long long i){Time_of_bo=i; }
	long long getTime_of_bo(){return Time_of_bo; }

	long long getPulse_count(){return Pulse_count; }
	long long getPulse_count_non_SYNC(){return Pulse_count_non_SYNC; }
	
	void setSYNC_time_radio_on(long long i){SYNC_time_radio_on=i; }
	long long getSYNC_time_radio_on(){return SYNC_time_radio_on; }

	void setTrans_finish_time(long long t){Trans_finish_time=t; }
	long long getTrans_finish_time(){return Trans_finish_time; }

	long long getTotal_ACK(){return Total_ACK; }
	long long getTotal_RTS(){return Total_RTS; }
	long long getTotal_CTS(){return Total_CTS; }
	long long getTotal_SYNC(){return Total_SYNC; }
	
	/*void setTemp_of_TCPP_time(long long i){Temp_of_TCPP_time=i; }
	void setTemp_of_TCPP(int i){Temp_of_TCPP=i; }*/

	//Event* getTimer_event(){return Timer_event; }

	
	
	//long long getTotal_num_tran_TCPP(){return Total_num_tran_TCPP; }
	//long long getNum_of_retransmission(){return Num_of_retransmission; }
	//long long getNum_of_TCPP_retransmitted(){return Num_of_TCPP_retransmitted; }



	void Back_off_delay_SYNC(Event *handle, Event *New_coming_event);                    //Event_type=0      
	void Send_SYNC(Event *handle, Event *New_coming_event);                              //Event_type=1
	void Receive_SYNC_start(Event *handle, Event *New_coming_event);                     //Event_type=2
	void Receive_SYNC_end_back_off(Event *handle, Event *New_coming_event);              //Event_type=3
	void Rebroadcast_SYNC_bakc_off(Event *handle, Event *New_coming_event);              //Event_type=4
	void Rebroadcast_SYNC(Event *handle, Event *New_coming_event);                       //Event_type=5
	void Receive_rebro_SYNC_Start(Event *handle, Event *New_coming_event);               //Event_type=6
	void Receive_rebro_SYNC_end(Event *handle, Event *New_coming_event);                 //Event_type=7
	void Back_off_delay_RTS(Event *handle, Event *New_coming_event);                     //Event_type=8
	void Send_RTS(Event *handle, Event *New_coming_event);                               //Event_type=9
	void Receive_RTS_start(Event *handle, Event *New_coming_event);                      //Event_type=10
	void Receive_RTS_end_send_CTS(Event *handle, Event *New_coming_event);               //Event_type=11
	void Receive_CTS_start(Event *handle, Event *New_coming_event);                      //Event_type=12
	void Receive_CTS_end_Send_TCPP(Event *handle, Event *New_coming_event);              //Event_type=13
	void Receive_TCPP_start(Event *handle, Event *New_coming_event);                     //Event_type=14
	void Receive_TCPP_end_send_ACK(Event *handle, Event *New_coming_event);              //Event_type=15
	void Receive_ACK_Start(Event *handle, Event *New_coming_event);                      //Event_type=16
	void Receive_ACK_end(Event *handle, Event *New_coming_event);                        //Event_type=17
	//void Release_passive_node(Event *handle, Event *New_coming_event);                   //Event_type=18
	//void Data_col_send_Error_signal(Event *handle, Event *New_coming_event);             
	//void Receive_Error_signal_start(Event *handle, Event *New_coming_event);             
	//void Receive_Error_signal_end(Event *handle, Event *New_coming_event);               
	//void Release_node_received_Error_signal(Event *handle, Event *New_coming_event);     
	void TCPP_insert_queue(Event *handle, Event *New_coming_event);                      //Event_type=19
	void Sleep_mode(Event *handle, Event *New_coming_event);                             //Event_type=20

	void setNeighbor_table(int a[], int n)   //initialize the neighbor_table
	{for(int i=0; i<n; i++) this->Neighbor_table[i]=a[i]; }

	void printNeighbor_table(int n)          //print out neighbor_table
	{
		for(int i=0; i<n; i++)
		{
			if(this->Neighbor_table[i]>=0)
				cout<<this->Neighbor_table[i]<<" "; 
		}
		cout<<endl; 
	}
};


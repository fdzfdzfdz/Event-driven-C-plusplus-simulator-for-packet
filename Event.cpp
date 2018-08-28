#include "Event.h"
#include "Node.h"
#include <queue>
#include <deque>
#include <iostream>
#include <ctime>
#include <cmath>
#include <stdio.h>
#include <cstdlib>
#include <fstream>

extern list<Node> Node_network;
extern Node *getNode_from_id(int i);
extern long long Simulation_time;
extern double Channel_rate; 
extern int Value_TCPP_Max;
extern double Lambda;
extern ofstream logs;

extern int Numofnodes;
extern int Node_array[64];

extern ofstream logs2;
extern ofstream logs3;
//extern ofstream logs4;
//extern long long scale;
int Period_point_1=static_cast<int>(pow(10.0, 6));
int Period_point_2=static_cast<int>(pow(10.0, 7));
int Period_point_3=static_cast<int>(pow(10.0, 8));

using namespace std;

Event::Event(void):Event_node_id(-1), Event_type(0), Event_time(0),Flag_event_point(0)
{}
Event::~Event(void)
{}
Simulator::Simulator(void)
{}
Simulator::~Simulator(void)
{}

void Event::action()
{
	if(this->getEvent_time()<=Simulation_time || this->getEvent_type()!=19)   //new
	{   
		int forwarding_node_id=this->getEvent_node_id();

		//if(this->getEvent_time()>=scale)
		//	{cout<<"scale: "<<scale<<endl; scale=scale+2000000000;}

		/*if(scale==0 && this->getEvent_time()>=Period_point_1)
		{   scale++; logs4<<"The number of transmission completed in "<<Period_point_1<<" unit time"<<" "<<this->getEvent_time()<<endl;
			for(int i=1; i<=Numofnodes; i++) { logs4<<"Node-"<<i<<": "<<getNode_from_id(i)->getTCPP_transmitted()<<endl;}
		}
		if(scale==1 && this->getEvent_time()>=Period_point_2)
		{   scale++; logs4<<"The number of transmission completed in "<<Period_point_2<<" unit time"<<" "<<this->getEvent_time()<<endl;
			for(int i=1; i<=Numofnodes; i++) { logs4<<"Node-"<<i<<": "<<getNode_from_id(i)->getTCPP_transmitted()<<endl;}
		}
		if(scale==2 && this->getEvent_time()>=Period_point_3)
		{ 	scale++; logs4<<"The number of transmission completed in "<<Period_point_3<<" unit time"<<" "<<this->getEvent_time()<<endl;
			for(int i=1; i<=Numofnodes; i++) { logs4<<"Node-"<<i<<": "<<getNode_from_id(i)->getTCPP_transmitted()<<endl;}
		}*/

		int k=0;
		if(this->getEvent_time()>Simulation_time)   //if over Simulation_time, see if deque is empty
		{
			for(int i=0; i<Numofnodes; i++)
			{
				if(getNode_from_id(i)->getQueuing_system_size()==0)
				{Node_array[i]=-1; }
			}
			for(int j=0; j<Numofnodes; j++)
			{
				if(Node_array[j]>0)
				{k++; }
			}
		}

		if(this->getEvent_type()==19 && this->getTCPP()>=0 )   //source node creates a new PPCP
		{
			double temp=0;
			while(temp==0)
				temp=rand()/(RAND_MAX*1.0);                                 //rand() generates a value from 0 to 32767
			int YY=static_cast<int>(- Channel_rate*log(temp)/Lambda);
			if(this->getEvent_time() +YY < Simulation_time )
			{
				Event *future_event1=new Event;
				future_event1->setEvent_type(19);  
				future_event1->setEvent_node_id( forwarding_node_id );
				future_event1->setEvent_time(this->getEvent_time() + YY );
				future_event1->setEvent_pointer(NULL);
				future_event1->setEvent_pointer_size(0);
				int TCPP=1;                                    //rand()%(Value_TCPP_Max+1);
				future_event1->setTCPP(TCPP);
				handle->add(future_event1);
			}
		}

		if(k>0 || this->getEvent_time()<=Simulation_time || 
			(this->getEvent_time()>Simulation_time && this->getEvent_type()!=0) )
		{
			Node *Current_node=getNode_from_id( forwarding_node_id );
			Current_node->setTrans_finish_time(this->getEvent_time() );
			switch(this->getEvent_type() )
			{
				case 0: Current_node->Back_off_delay_SYNC(handle, this); break;
				case 1: Current_node->Send_SYNC(handle, this); break;
				case 2: Current_node->Receive_SYNC_start(handle, this); break;
				case 3: Current_node->Receive_SYNC_end_back_off(handle, this); break;
				case 4: Current_node->Rebroadcast_SYNC_bakc_off(handle, this); break;
				case 5: Current_node->Rebroadcast_SYNC(handle, this); break;
				case 6: Current_node->Receive_rebro_SYNC_Start(handle, this); break;
				case 7: Current_node->Receive_rebro_SYNC_end(handle, this); break;
				case 8: Current_node->Back_off_delay_RTS(handle, this); break;
				case 9: Current_node->Send_RTS(handle, this); break;
				case 10: Current_node->Receive_RTS_start(handle, this); break;
				case 11: Current_node->Receive_RTS_end_send_CTS(handle, this); break;
				case 12: Current_node->Receive_CTS_start(handle, this); break;
				case 13: Current_node->Receive_CTS_end_Send_TCPP(handle, this); break;
				case 14: Current_node->Receive_TCPP_start(handle, this); break;
				case 15: Current_node->Receive_TCPP_end_send_ACK(handle, this); break;
				case 16: Current_node->Receive_ACK_Start(handle, this); break;
				case 17: Current_node->Receive_ACK_end(handle, this); break;

				case 19: Current_node->TCPP_insert_queue(handle, this); break;
				case 20: Current_node->Sleep_mode(handle, this); break;
			}
		}
	}
}

void Simulator::action()
{
	while(pendingEvents.size()!=0)
	{
		Event *current_event=pendingEvents.top();
		//pendingEvents.pop();                      //pop is advanced implemented
		if(current_event->getFlag_event_point()==0)
		{current_event->action(); }
		pendingEvents.pop();  // 1/4/2015 for Invalid heap
	//	logs2<<current_event->getEvent_type()<<"	"<<current_event->getEvent_node_id()<<"	"<<current_event->getEvent_time()
	//		<<"	"<<current_event->getFlag_event_point()<<"	"<<current_event<<endl;
		delete current_event;
	}
}
void Simulator::add(Event *aEvent)
{
	aEvent->add(this);
	//logs3<<aEvent->getEvent_type()<<"	"<<aEvent->getEvent_node_id()<<"	"<<aEvent->getEvent_time()
	//		<<"	"<<aEvent->getFlag_event_point()<<"	"<<aEvent<<endl;
	pendingEvents.push(aEvent);
	//count=pendingEvents.size();
}





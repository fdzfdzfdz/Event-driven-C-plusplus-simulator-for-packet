#pragma once
//NOTE: #include "My_node.h"  will cause some problem, great!!!!!!!!!!!!!!!!!!!!
#include "Event.h"
#include <queue>
#include <ctime>
#include <cmath>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <fstream>

using namespace std;

struct TCPP_pair
{
	int TCPP;
	long long Time_in;
};

class Event
{
private:
	int* Event_pointer;
	int Event_pointer_size;
	int TCPP;  
	int Event_node_id;
	int Event_type;                      //indicate which function needs to be called
	long long Event_time;
	int Flag_event_point;                //0: active,   1: deactive
	//int Event_source_node_id;                 //add later
	//TCPP_cell *Event_TCPP_pointer;            //use in the future

protected:
	Event *handle;
public:
	Event(void);
	virtual ~Event(void);
	void setEvent_type(int type){Event_type=type;}
	int getEvent_type(){return Event_type;}

	void setEvent_node_id(int n_id){Event_node_id=n_id;}
	int getEvent_node_id(){return Event_node_id;}

	void setEvent_pointer(int* q_id){Event_pointer=q_id;}
	int* getEvent_pointer(){return Event_pointer;}

	void setEvent_pointer_size(int i){Event_pointer_size=i;}
	int getEvent_pointer_size(){return Event_pointer_size;}

	void setEvent_time(long long t){Event_time=t;}
	long long getEvent_time(){return Event_time;}

	void setTCPP(int i){TCPP=i;}
	int getTCPP(){return TCPP;}

	void setFlag_event_point(int i){Flag_event_point=i;}
	int getFlag_event_point(){return Flag_event_point;}

	virtual void add(Event *e){handle=e;}
	virtual void action();

	/*void setEvent_TCPP_pointer(TCPP_cell *q){Event_TCPP_pointer=q; }
	TCPP_cell *getEvent_TCPP_pointer(){return Event_TCPP_pointer; }*/
	/*void setEvent_source_node_id(int i){Event_source_node_id=i; }
	int getEvent_source_node_id(){return Event_source_node_id; }*/
	/*void setEvent_size(double size){Event_size=size;}
	double getEvent_size(){return Event_size;}*/
	//void setEventNo(long long eNumber){eventNo=eNumber;}
	//long long getEventNo(){return eventNo; }
};


struct OrderByTime
{bool operator() (Event* const a, Event* const b) 
{
	/*if(a->getEvent_time()==b->getEvent_time() && a->getEvent_node_id()==b->getEvent_node_id() 
		&& (a->getEvent_type()>=10 || b->getEvent_type()>=10) )
		return a->getEvent_type() < b->getEvent_type();
	else*/
		return a->getEvent_time() > b->getEvent_time();  // 1/4/2015 for Invalid heap
}  
};             //for the event heap ordering

typedef priority_queue< Event*,  vector<Event*>,  OrderByTime > event_queue;                           //for the event heap



class Simulator: public Event
{
private:
	event_queue pendingEvents;
public:
	Simulator(void);
	~Simulator(void);
	virtual void action(void);
	virtual void add(Event *aEvent);
	event_queue get_event_queue(){return pendingEvents; }
};



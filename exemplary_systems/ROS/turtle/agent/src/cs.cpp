/* FILE GENERATED AUTOMATICALY */
/*
	Copyright (c) 2019, Robot Control and Pattern Recognition Group, Warsaw University of Technology
	All rights reserved.
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:
				* Redistributions of source code must retain the above copyright
				  notice, this list of conditions and the following disclaimer.
				* Redistributions in binary form must reproduce the above copyright
				  notice, this list of conditions and the following disclaimer in the
				  documentation and/or other materials provided with the distribution.
				* Neither the name of the Warsaw University of Technology nor the
				  names of its contributors may be used to endorse or promote products
				  derived from this software without specific prior written permission.
	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

	Author: Maksym Figat

*/

#ifndef CS_H
#define CS_H

#include "auxiliary_functions.h"
#include "auxiliary_agent.h"
#include "auxiliary_cs.h"

namespace agent{
	class cs{
	private:
		/* ROS node */
		ros::NodeHandle * _n;

		/* Current subsystem state name */
		std::string _currentSubsystemState;

		double _subsystemFrequency;
		std::string _subsystemName;

		/* Subsystem iterations */
		int _subsystemIterations;

		/* Behaviour iterations */
		int _behaviourIterations;

		ros::Publisher _sender_desiredAngle;
		ros::Publisher _sender_beginMovement;
		ros::Publisher _sender_desiredDirection;
		std::vector<ros::Publisher > _vectorOfSenderDiagnostics;
		ros::Subscriber _subscriber_moveNorthFinished;
		ros::Subscriber _subscriber_moveEastFinished;
		ros::Subscriber _subscriber_moveSouthFinished;
		ros::Subscriber _subscriber_moveWestFinished;

		/* Buffers */
		/* Input Buffers */
		std_msgs::Int64 _moveNorthFinished;
		std_msgs::Int64 _moveEastFinished;
		std_msgs::Int64 _moveSouthFinished;
		std_msgs::Int64 _moveWestFinished;

		/* Output Buffers */
		std_msgs::Float64 _desiredAngle;
		std_msgs::Int64 _beginMovement;
		std_msgs::Int64 _desiredDirection;

		/* Internal Memory Buffers */
		std_msgs::Int64 _isFirst;
	public:
		cs();
		/* updates current state based on initial condition */
		void updateCurrentState();
		void startSubsystem();
		void update_moveNorthFinished(const std_msgs::Int64::ConstPtr& msg);
		void update_moveEastFinished(const std_msgs::Int64::ConstPtr& msg);
		void update_moveSouthFinished(const std_msgs::Int64::ConstPtr& msg);
		void update_moveWestFinished(const std_msgs::Int64::ConstPtr& msg);
		/* Return data from output buffers */
		std_msgs::Float64 get_desiredAngle();
		std_msgs::Int64 get_beginMovement();
		std_msgs::Int64 get_desiredDirection();

		/* Return data from input buffers */
		std_msgs::Int64 get_moveNorthFinished();
		std_msgs::Int64 get_moveEastFinished();
		std_msgs::Int64 get_moveSouthFinished();
		std_msgs::Int64 get_moveWestFinished();

		/* Return data from internal buffers */
		std_msgs::Int64 get_isFirst();

		/* Set Input Buffers */
		void set_moveNorthFinished(std_msgs::Int64 data);
		void set_moveEastFinished(std_msgs::Int64 data);
		void set_moveSouthFinished(std_msgs::Int64 data);
		void set_moveWestFinished(std_msgs::Int64 data);

		/* Set Internal Buffers */
		void set_isFirst(std_msgs::Int64 data);

		/* Set Output Buffers */
		void set_desiredAngle(std_msgs::Float64 data);
		void set_beginMovement(std_msgs::Int64 data);
		void set_desiredDirection(std_msgs::Int64 data);

		std::string getSubsystemName();
		void printInfo(std::string str);
		void printError(std::string str);
		void printLog(std::string str);
		double get_subsystemFrequency();
		/* Initialise communication model */
		void initialiseModel();
		void initialiseSendChannel();
		void initialiseSendChannelForDiagnostics();
		void initialiseReceiveChannel();
		void sendDataForDiagnostics();
		/* Behaviour declarations */

		/* Behaviour Idle */
		/* Terminal Condition */
		bool terminalCondition_Idle(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Int64_isFirst */);
		/* Error Condition */
		bool errorCondition_Idle(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Int64_isFirst */);
		/* Transition function */
		void transitionFunction_Idle(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,std_msgs::Int64_isFirst */);
		/* Send data to other subsystems */
		void sendData_Idle(/* std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection */);
		/* Receive data from other subsystems */
		void receiveData_Idle(/* */);
		/* Execute behaviour Idle*/
		void executeBehaviour_Idle(/* */);

		/* Behaviour moveNorth */
		/* Terminal Condition */
		bool terminalCondition_moveNorth(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Int64_isFirst */);
		/* Error Condition */
		bool errorCondition_moveNorth(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Int64_isFirst */);
		/* Transition function */
		void transitionFunction_moveNorth(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,std_msgs::Int64_isFirst */);
		/* Send data to other subsystems */
		void sendData_moveNorth(/* std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection */);
		/* Receive data from other subsystems */
		void receiveData_moveNorth(/* */);
		/* Execute behaviour moveNorth*/
		void executeBehaviour_moveNorth(/* */);

		/* Behaviour moveEast */
		/* Terminal Condition */
		bool terminalCondition_moveEast(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Int64_isFirst */);
		/* Error Condition */
		bool errorCondition_moveEast(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Int64_isFirst */);
		/* Transition function */
		void transitionFunction_moveEast(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,std_msgs::Int64_isFirst */);
		/* Send data to other subsystems */
		void sendData_moveEast(/* std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection */);
		/* Receive data from other subsystems */
		void receiveData_moveEast(/* */);
		/* Execute behaviour moveEast*/
		void executeBehaviour_moveEast(/* */);

		/* Behaviour moveSouth */
		/* Terminal Condition */
		bool terminalCondition_moveSouth(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Int64_isFirst */);
		/* Error Condition */
		bool errorCondition_moveSouth(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Int64_isFirst */);
		/* Transition function */
		void transitionFunction_moveSouth(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,std_msgs::Int64_isFirst */);
		/* Send data to other subsystems */
		void sendData_moveSouth(/* std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection */);
		/* Receive data from other subsystems */
		void receiveData_moveSouth(/* */);
		/* Execute behaviour moveSouth*/
		void executeBehaviour_moveSouth(/* */);

		/* Behaviour moveWest */
		/* Terminal Condition */
		bool terminalCondition_moveWest(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Int64_isFirst */);
		/* Error Condition */
		bool errorCondition_moveWest(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Int64_isFirst */);
		/* Transition function */
		void transitionFunction_moveWest(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,std_msgs::Int64_isFirst */);
		/* Send data to other subsystems */
		void sendData_moveWest(/* std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection */);
		/* Receive data from other subsystems */
		void receiveData_moveWest(/* */);
		/* Execute behaviour moveWest*/
		void executeBehaviour_moveWest(/* */);

		/* Declaration of functions responsible for switching subsystem cs between states : S_Idle S_moveNorth S_moveEast S_moveSouth S_moveWest */
		/* State S_Idle: */
		void subsystemState_S_Idle();
		/* State S_moveNorth: */
		void subsystemState_S_moveNorth();
		/* State S_moveEast: */
		void subsystemState_S_moveEast();
		/* State S_moveSouth: */
		void subsystemState_S_moveSouth();
		/* State S_moveWest: */
		void subsystemState_S_moveWest();
		/* Initial conditions */
		/* Initial condition for state S_Idle: switching to state S_moveNorth*/
		bool initialCondition_From_S_Idle_To_S_moveNorth();
		/* Initial condition for state S_moveNorth: switching to state S_moveEast*/
		bool initialCondition_From_S_moveNorth_To_S_moveEast();
		/* Initial condition for state S_moveEast: switching to state S_moveSouth*/
		bool initialCondition_From_S_moveEast_To_S_moveSouth();
		/* Initial condition for state S_moveSouth: switching to state S_moveWest*/
		bool initialCondition_From_S_moveSouth_To_S_moveWest();
		/* Initial condition for state S_moveWest: switching to state S_moveNorth*/
		bool initialCondition_From_S_moveWest_To_S_moveNorth();
	}; // END OF cs
} // END OF agent


agent::cs::cs(){
	_n = new ros::NodeHandle();
	_subsystemName="cs";
	_subsystemFrequency=10;
	_currentSubsystemState="S_Idle";
	initialiseModel();
}

/* Start subsystem */
void agent::cs::startSubsystem(){
	try{
		do{
			/* Execute behaviour associated with _currentSubsystemState -- choose appropriate state based on _currentSubsystemState */
			if(_currentSubsystemState=="S_Idle"){
				subsystemState_S_Idle();
				continue;
			}
			if(_currentSubsystemState=="S_moveNorth"){
				subsystemState_S_moveNorth();
				continue;
			}
			if(_currentSubsystemState=="S_moveEast"){
				subsystemState_S_moveEast();
				continue;
			}
			if(_currentSubsystemState=="S_moveSouth"){
				subsystemState_S_moveSouth();
				continue;
			}
			if(_currentSubsystemState=="S_moveWest"){
				subsystemState_S_moveWest();
				continue;
			}
		}
		while(AuxiliaryFunctions::isSubsystemOK());
	}
	catch(...){
		printError("Current state is out of the bound -- method startSubsystem -- file subsystem.h!");
		AuxiliaryFunctions::shutdownSubsystem();
	}
}

void agent::cs::update_moveNorthFinished(const std_msgs::Int64::ConstPtr& msg){
	_moveNorthFinished=*msg;
}

void agent::cs::update_moveEastFinished(const std_msgs::Int64::ConstPtr& msg){
	_moveEastFinished=*msg;
}

void agent::cs::update_moveSouthFinished(const std_msgs::Int64::ConstPtr& msg){
	_moveSouthFinished=*msg;
}

void agent::cs::update_moveWestFinished(const std_msgs::Int64::ConstPtr& msg){
	_moveWestFinished=*msg;
}

void agent::cs::set_moveNorthFinished(std_msgs::Int64 data){
	_moveNorthFinished=data;
}

void agent::cs::set_moveEastFinished(std_msgs::Int64 data){
	_moveEastFinished=data;
}

void agent::cs::set_moveSouthFinished(std_msgs::Int64 data){
	_moveSouthFinished=data;
}

void agent::cs::set_moveWestFinished(std_msgs::Int64 data){
	_moveWestFinished=data;
}

void agent::cs::set_isFirst(std_msgs::Int64 data){
	_isFirst=data;
}

void agent::cs::set_desiredAngle(std_msgs::Float64 data){
	_desiredAngle=data;
}

void agent::cs::set_beginMovement(std_msgs::Int64 data){
	_beginMovement=data;
}

void agent::cs::set_desiredDirection(std_msgs::Int64 data){
	_desiredDirection=data;
}

std::string agent::cs::getSubsystemName(){
	return _subsystemName;
}

void agent::cs::printInfo(std::string str){
	std::cout<<"["<<_subsystemName<<"] -- info: "<<str<<std::endl;
}

void agent::cs::printError(std::string str){
	std::cerr<<"["<<_subsystemName<<"] -- error : "<<str<<std::endl;
}

void agent::cs::printLog(std::string str){
	#if PRINT_LOG
		std::clog<<"["<<_subsystemName<<"] -- log : "<<str<<std::endl;
	#endif
}

double agent::cs::get_subsystemFrequency(){
	return _subsystemFrequency;
}

void agent::cs::initialiseModel(){
	initialiseSendChannel();
	initialiseSendChannelForDiagnostics();
	initialiseReceiveChannel();
}

void agent::cs::initialiseSendChannel(){
		_sender_desiredAngle=_n->advertise<std_msgs::Float64>("desiredAngle", CHANNEL_SIZE);
		_sender_beginMovement=_n->advertise<std_msgs::Int64>("beginMovement", CHANNEL_SIZE);
		_sender_desiredDirection=_n->advertise<std_msgs::Int64>("desiredDirection", CHANNEL_SIZE);
}

void agent::cs::initialiseSendChannelForDiagnostics(){
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::Int64>("cs/_isFirst", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::String>("cs/_currentSubsystemState", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::Float64>("cs/_subsystemFrequency", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::String>("cs/_subsystemName", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::Int64>("cs/_subsystemIterations", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::Int64>("cs/_behaviourIterations", CHANNEL_SIZE));
}

void agent::cs::initialiseReceiveChannel(){
	_subscriber_moveNorthFinished=_n->subscribe("moveNorthFinished", CHANNEL_SIZE, &agent::cs::update_moveNorthFinished, this);
	_subscriber_moveEastFinished=_n->subscribe("moveEastFinished", CHANNEL_SIZE, &agent::cs::update_moveEastFinished, this);
	_subscriber_moveSouthFinished=_n->subscribe("moveSouthFinished", CHANNEL_SIZE, &agent::cs::update_moveSouthFinished, this);
	_subscriber_moveWestFinished=_n->subscribe("moveWestFinished", CHANNEL_SIZE, &agent::cs::update_moveWestFinished, this);
}

/* Return data from output buffers */
std_msgs::Float64 agent::cs::get_desiredAngle(){	return _desiredAngle;	}

std_msgs::Int64 agent::cs::get_beginMovement(){	return _beginMovement;	}

std_msgs::Int64 agent::cs::get_desiredDirection(){	return _desiredDirection;	}

/* Return data from input buffers */
std_msgs::Int64 agent::cs::get_moveNorthFinished(){	return _moveNorthFinished;	}

std_msgs::Int64 agent::cs::get_moveEastFinished(){	return _moveEastFinished;	}

std_msgs::Int64 agent::cs::get_moveSouthFinished(){	return _moveSouthFinished;	}

std_msgs::Int64 agent::cs::get_moveWestFinished(){	return _moveWestFinished;	}

/* Return data from internal buffers */
std_msgs::Int64 agent::cs::get_isFirst(){	return _isFirst;	}

/* Send (publish) diagnostic data */
void agent::cs::sendDataForDiagnostics(/* subsystem diagnostic state */){
	_vectorOfSenderDiagnostics[0].publish(_isFirst);	std_msgs::String temporarString;
	std_msgs::Int64 temporarInt;
	std_msgs::Float64 temporarFloat;
	temporarString.data=_currentSubsystemState;
	_vectorOfSenderDiagnostics[1].publish(temporarString);
	temporarFloat.data=_subsystemFrequency;
	_vectorOfSenderDiagnostics[2].publish(temporarFloat);
	temporarString.data=_subsystemName;
	_vectorOfSenderDiagnostics[3].publish(temporarString);
	temporarInt.data=_subsystemIterations;
	_vectorOfSenderDiagnostics[4].publish(temporarInt);
	temporarInt.data=_behaviourIterations;
	_vectorOfSenderDiagnostics[5].publish(temporarInt);
}
/* Behaviour definitions */
/* Behaviour Idle */
/* Terminal Condition */
bool agent::cs::terminalCondition_Idle(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Int64_isFirst */){
	return  true ;
}
/* Error Condition */
bool agent::cs::errorCondition_Idle(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Int64_isFirst */){
	return false;
}
/* Transition function */
void agent::cs::transitionFunction_Idle(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,std_msgs::Int64_isFirst */){
	/* Partial transition function - name - tf1_2*/
	 
	_desiredAngle.data=GO_WEST_DIRECTION;
	_beginMovement.data=1;
	_moveNorthFinished.data=0;
	_moveEastFinished.data=0;
	_moveSouthFinished.data=0;
	_moveWestFinished.data=0;
	_desiredDirection.data=0;
	_isFirst.data=1;
}
/* Send data to other subsystems */
void agent::cs::sendData_Idle(/* std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection */){
	/* DIAGNOSTICS SEND */
	sendDataForDiagnostics();
	/* END OF DIAGNOSTICS SEND */

	/* TYPICAL SEND CALL */
	_sender_desiredAngle.publish(_desiredAngle);
	_sender_beginMovement.publish(_beginMovement);
	_sender_desiredDirection.publish(_desiredDirection);
	/* END OF TYPICAL SEND CALL */

	std::cout<<"[Behaviour Idle] -- Sending Data"<<std::endl;
}
/* Receive data from other subsystems */
void agent::cs::receiveData_Idle(/* */){
	/* TYPICAL RECEIVE CALL */
	ros::spinOnce();
	/* END OF TYPICAL RECEIVE CALL */

	std::cout<<"[Behaviour Idle] -- Receiving Data"<<std::endl;
}
/* Execute behaviour Idle*/
void agent::cs::executeBehaviour_Idle(/* */){
	bool stopBehaviourIteration=false;
	/* Execution of a single iteration of a behaviour Idle */
	_behaviourIterations=0;
	AuxiliaryFunctions rate = AuxiliaryFunctions(_subsystemFrequency);
	/* Starts execution! */
	do{
		/* Sleep is a method from class AuxiliaryFunctions which executes sleep from ros::Rate */
		rate.sleep();
		/* Calculates transition function -- output and internal buffers can only be modified by this function! */
		transitionFunction_Idle();
		/* Sends data! */
		sendData_Idle();
		/* Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations */
		_behaviourIterations++;
		/* Receives data! */
		receiveData_Idle();
		/* Check both conditions, i.e. terminal condition and error condition */
		stopBehaviourIteration = terminalCondition_Idle() || errorCondition_Idle();
	}
	while(!stopBehaviourIteration && AuxiliaryFunctions::isSubsystemOK()); /* Iterate within the while loop until stopBehaviourIteration is set true, i.e. one of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise subsystem must have been switched to another state or SIGINT was sent */
	/* Stops execution! */
}


/* Behaviour moveNorth */
/* Terminal Condition */
bool agent::cs::terminalCondition_moveNorth(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Int64_isFirst */){
	return   _moveNorthFinished.data==1 ;
}
/* Error Condition */
bool agent::cs::errorCondition_moveNorth(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Int64_isFirst */){
	return false;
}
/* Transition function */
void agent::cs::transitionFunction_moveNorth(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,std_msgs::Int64_isFirst */){
	/* Partial transition function - name - tf1_1*/
	_desiredAngle.data=GO_NORTH_DIRECTION;
	_desiredDirection.data=GO_NORTH;
	if(_isFirst.data==1){
		_isFirst.data=0;	
		_beginMovement.data=1;
	}
	else{
		_beginMovement.data=1;
	}
	_moveEastFinished.data=0;
	_moveSouthFinished.data=0;
	_moveWestFinished.data=0;
	if(_moveNorthFinished.data==1){
		_beginMovement.data=1;
		_desiredDirection.data=0;
		_isFirst.data=1;
	}
	std::cout<<"[CS - MOVE NORTH]"<<std::endl;
										
}
/* Send data to other subsystems */
void agent::cs::sendData_moveNorth(/* std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection */){
	/* DIAGNOSTICS SEND */
	sendDataForDiagnostics();
	/* END OF DIAGNOSTICS SEND */

	/* TYPICAL SEND CALL */
	_sender_desiredAngle.publish(_desiredAngle);
	_sender_beginMovement.publish(_beginMovement);
	_sender_desiredDirection.publish(_desiredDirection);
	/* END OF TYPICAL SEND CALL */

	std::cout<<"[Behaviour moveNorth] -- Sending Data"<<std::endl;
}
/* Receive data from other subsystems */
void agent::cs::receiveData_moveNorth(/* */){
	/* TYPICAL RECEIVE CALL */
	ros::spinOnce();
	/* END OF TYPICAL RECEIVE CALL */

	std::cout<<"[Behaviour moveNorth] -- Receiving Data"<<std::endl;
}
/* Execute behaviour moveNorth*/
void agent::cs::executeBehaviour_moveNorth(/* */){
	bool stopBehaviourIteration=false;
	/* Execution of a single iteration of a behaviour moveNorth */
	_behaviourIterations=0;
	AuxiliaryFunctions rate = AuxiliaryFunctions(_subsystemFrequency);
	/* Starts execution! */
	do{
		/* Sleep is a method from class AuxiliaryFunctions which executes sleep from ros::Rate */
		rate.sleep();
		/* Calculates transition function -- output and internal buffers can only be modified by this function! */
		transitionFunction_moveNorth();
		/* Sends data! */
		sendData_moveNorth();
		/* Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations */
		_behaviourIterations++;
		/* Receives data! */
		receiveData_moveNorth();
		/* Check both conditions, i.e. terminal condition and error condition */
		stopBehaviourIteration = terminalCondition_moveNorth() || errorCondition_moveNorth();
	}
	while(!stopBehaviourIteration && AuxiliaryFunctions::isSubsystemOK()); /* Iterate within the while loop until stopBehaviourIteration is set true, i.e. one of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise subsystem must have been switched to another state or SIGINT was sent */
	/* Stops execution! */
}


/* Behaviour moveEast */
/* Terminal Condition */
bool agent::cs::terminalCondition_moveEast(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Int64_isFirst */){
	return   _moveEastFinished.data==1 ;
}
/* Error Condition */
bool agent::cs::errorCondition_moveEast(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Int64_isFirst */){
	return false;
}
/* Transition function */
void agent::cs::transitionFunction_moveEast(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,std_msgs::Int64_isFirst */){
	/* Partial transition function - name - tf1_1*/
	_moveNorthFinished.data=0;
	_moveSouthFinished.data=0;
	_moveWestFinished.data=0;
	if(_isFirst.data==1){
		_isFirst.data=0;	
		_beginMovement.data=1;
	}
	else{
		_beginMovement.data=1;
	}
	_desiredAngle.data=GO_EAST_DIRECTION;
	_desiredDirection.data=GO_EAST;
	if(_moveEastFinished.data==1){
		_beginMovement.data=1;
		_desiredDirection.data=0;
		_isFirst.data=1;
	}
	std::cout<<"[CS - MOVE EAST]"<<std::endl;
								
}
/* Send data to other subsystems */
void agent::cs::sendData_moveEast(/* std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection */){
	/* DIAGNOSTICS SEND */
	sendDataForDiagnostics();
	/* END OF DIAGNOSTICS SEND */

	/* TYPICAL SEND CALL */
	_sender_desiredAngle.publish(_desiredAngle);
	_sender_beginMovement.publish(_beginMovement);
	_sender_desiredDirection.publish(_desiredDirection);
	/* END OF TYPICAL SEND CALL */

	std::cout<<"[Behaviour moveEast] -- Sending Data"<<std::endl;
}
/* Receive data from other subsystems */
void agent::cs::receiveData_moveEast(/* */){
	/* TYPICAL RECEIVE CALL */
	ros::spinOnce();
	/* END OF TYPICAL RECEIVE CALL */

	std::cout<<"[Behaviour moveEast] -- Receiving Data"<<std::endl;
}
/* Execute behaviour moveEast*/
void agent::cs::executeBehaviour_moveEast(/* */){
	bool stopBehaviourIteration=false;
	/* Execution of a single iteration of a behaviour moveEast */
	_behaviourIterations=0;
	AuxiliaryFunctions rate = AuxiliaryFunctions(_subsystemFrequency);
	/* Starts execution! */
	do{
		/* Sleep is a method from class AuxiliaryFunctions which executes sleep from ros::Rate */
		rate.sleep();
		/* Calculates transition function -- output and internal buffers can only be modified by this function! */
		transitionFunction_moveEast();
		/* Sends data! */
		sendData_moveEast();
		/* Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations */
		_behaviourIterations++;
		/* Receives data! */
		receiveData_moveEast();
		/* Check both conditions, i.e. terminal condition and error condition */
		stopBehaviourIteration = terminalCondition_moveEast() || errorCondition_moveEast();
	}
	while(!stopBehaviourIteration && AuxiliaryFunctions::isSubsystemOK()); /* Iterate within the while loop until stopBehaviourIteration is set true, i.e. one of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise subsystem must have been switched to another state or SIGINT was sent */
	/* Stops execution! */
}


/* Behaviour moveSouth */
/* Terminal Condition */
bool agent::cs::terminalCondition_moveSouth(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Int64_isFirst */){
	return  _moveSouthFinished.data==1 ;
}
/* Error Condition */
bool agent::cs::errorCondition_moveSouth(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Int64_isFirst */){
	return false;
}
/* Transition function */
void agent::cs::transitionFunction_moveSouth(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,std_msgs::Int64_isFirst */){
	/* Partial transition function - name - tf1_1*/
	_moveNorthFinished.data=0;
	_moveEastFinished.data=0;
	_moveWestFinished.data=0;
	if(_isFirst.data==1){
		_isFirst.data=0;	
		_beginMovement.data=1;
	}
	else{
		_beginMovement.data=1;
	}
	_desiredAngle.data=GO_SOUTH_DIRECTION;
	_desiredDirection.data=GO_SOUTH;
	if(_moveSouthFinished.data==1){
		_beginMovement.data=1;
		_desiredDirection.data=0;
		_isFirst.data=1;
	}
	std::cout<<"[CS - MOVE SOUTH]"<<std::endl;
									
}
/* Send data to other subsystems */
void agent::cs::sendData_moveSouth(/* std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection */){
	/* DIAGNOSTICS SEND */
	sendDataForDiagnostics();
	/* END OF DIAGNOSTICS SEND */

	/* TYPICAL SEND CALL */
	_sender_desiredAngle.publish(_desiredAngle);
	_sender_beginMovement.publish(_beginMovement);
	_sender_desiredDirection.publish(_desiredDirection);
	/* END OF TYPICAL SEND CALL */

	std::cout<<"[Behaviour moveSouth] -- Sending Data"<<std::endl;
}
/* Receive data from other subsystems */
void agent::cs::receiveData_moveSouth(/* */){
	/* TYPICAL RECEIVE CALL */
	ros::spinOnce();
	/* END OF TYPICAL RECEIVE CALL */

	std::cout<<"[Behaviour moveSouth] -- Receiving Data"<<std::endl;
}
/* Execute behaviour moveSouth*/
void agent::cs::executeBehaviour_moveSouth(/* */){
	bool stopBehaviourIteration=false;
	/* Execution of a single iteration of a behaviour moveSouth */
	_behaviourIterations=0;
	AuxiliaryFunctions rate = AuxiliaryFunctions(_subsystemFrequency);
	/* Starts execution! */
	do{
		/* Sleep is a method from class AuxiliaryFunctions which executes sleep from ros::Rate */
		rate.sleep();
		/* Calculates transition function -- output and internal buffers can only be modified by this function! */
		transitionFunction_moveSouth();
		/* Sends data! */
		sendData_moveSouth();
		/* Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations */
		_behaviourIterations++;
		/* Receives data! */
		receiveData_moveSouth();
		/* Check both conditions, i.e. terminal condition and error condition */
		stopBehaviourIteration = terminalCondition_moveSouth() || errorCondition_moveSouth();
	}
	while(!stopBehaviourIteration && AuxiliaryFunctions::isSubsystemOK()); /* Iterate within the while loop until stopBehaviourIteration is set true, i.e. one of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise subsystem must have been switched to another state or SIGINT was sent */
	/* Stops execution! */
}


/* Behaviour moveWest */
/* Terminal Condition */
bool agent::cs::terminalCondition_moveWest(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Int64_isFirst */){
	return  _moveWestFinished.data==1 ;
}
/* Error Condition */
bool agent::cs::errorCondition_moveWest(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Int64_isFirst */){
	return false;
}
/* Transition function */
void agent::cs::transitionFunction_moveWest(/* std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,std_msgs::Int64_isFirst */){
	/* Partial transition function - name - tf1_1*/
	_moveNorthFinished.data=0;
	_moveEastFinished.data=0;
	_moveSouthFinished.data=0;
	if(_isFirst.data==1){
		_isFirst.data=0;	
		_beginMovement.data=1;
	}
	else{
		_beginMovement.data=1;
	}
	_desiredAngle.data=GO_WEST_DIRECTION;
	_desiredDirection.data=GO_WEST;
	if(_moveWestFinished.data==1){
		_beginMovement.data=1;
		_desiredDirection.data=0;
		_isFirst.data=1;
	}
	std::cout<<"[CS - MOVE WEST]"<<std::endl;
									
}
/* Send data to other subsystems */
void agent::cs::sendData_moveWest(/* std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection */){
	/* DIAGNOSTICS SEND */
	sendDataForDiagnostics();
	/* END OF DIAGNOSTICS SEND */

	/* TYPICAL SEND CALL */
	_sender_desiredAngle.publish(_desiredAngle);
	_sender_beginMovement.publish(_beginMovement);
	_sender_desiredDirection.publish(_desiredDirection);
	/* END OF TYPICAL SEND CALL */

	std::cout<<"[Behaviour moveWest] -- Sending Data"<<std::endl;
}
/* Receive data from other subsystems */
void agent::cs::receiveData_moveWest(/* */){
	/* TYPICAL RECEIVE CALL */
	ros::spinOnce();
	/* END OF TYPICAL RECEIVE CALL */

	std::cout<<"[Behaviour moveWest] -- Receiving Data"<<std::endl;
}
/* Execute behaviour moveWest*/
void agent::cs::executeBehaviour_moveWest(/* */){
	bool stopBehaviourIteration=false;
	/* Execution of a single iteration of a behaviour moveWest */
	_behaviourIterations=0;
	AuxiliaryFunctions rate = AuxiliaryFunctions(_subsystemFrequency);
	/* Starts execution! */
	do{
		/* Sleep is a method from class AuxiliaryFunctions which executes sleep from ros::Rate */
		rate.sleep();
		/* Calculates transition function -- output and internal buffers can only be modified by this function! */
		transitionFunction_moveWest();
		/* Sends data! */
		sendData_moveWest();
		/* Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations */
		_behaviourIterations++;
		/* Receives data! */
		receiveData_moveWest();
		/* Check both conditions, i.e. terminal condition and error condition */
		stopBehaviourIteration = terminalCondition_moveWest() || errorCondition_moveWest();
	}
	while(!stopBehaviourIteration && AuxiliaryFunctions::isSubsystemOK()); /* Iterate within the while loop until stopBehaviourIteration is set true, i.e. one of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise subsystem must have been switched to another state or SIGINT was sent */
	/* Stops execution! */
}

/* Definition of functions responsible for switching subsystem cs between states : S_Idle S_moveNorth S_moveEast S_moveSouth S_moveWest */
/* State S_Idle: */
void agent::cs::subsystemState_S_Idle(){
	/* Executing behaviour Idle */
	executeBehaviour_Idle(/* */);
	/* Behaviour has been terminated */
	/* Checking initial condition for state S_Idle: switching to state S_moveNorth*/
	if(initialCondition_From_S_Idle_To_S_moveNorth()){
		/* incrementing the number determining how many times subsystem has switched between states */
		_subsystemIterations++;
		_currentSubsystemState="S_moveNorth";
	}
}

/* State S_moveNorth: */
void agent::cs::subsystemState_S_moveNorth(){
	/* Executing behaviour moveNorth */
	executeBehaviour_moveNorth(/* */);
	/* Behaviour has been terminated */
	/* Checking initial condition for state S_moveNorth: switching to state S_moveEast*/
	if(initialCondition_From_S_moveNorth_To_S_moveEast()){
		/* incrementing the number determining how many times subsystem has switched between states */
		_subsystemIterations++;
		_currentSubsystemState="S_moveEast";
	}
}

/* State S_moveEast: */
void agent::cs::subsystemState_S_moveEast(){
	/* Executing behaviour moveEast */
	executeBehaviour_moveEast(/* */);
	/* Behaviour has been terminated */
	/* Checking initial condition for state S_moveEast: switching to state S_moveSouth*/
	if(initialCondition_From_S_moveEast_To_S_moveSouth()){
		/* incrementing the number determining how many times subsystem has switched between states */
		_subsystemIterations++;
		_currentSubsystemState="S_moveSouth";
	}
}

/* State S_moveSouth: */
void agent::cs::subsystemState_S_moveSouth(){
	/* Executing behaviour moveSouth */
	executeBehaviour_moveSouth(/* */);
	/* Behaviour has been terminated */
	/* Checking initial condition for state S_moveSouth: switching to state S_moveWest*/
	if(initialCondition_From_S_moveSouth_To_S_moveWest()){
		/* incrementing the number determining how many times subsystem has switched between states */
		_subsystemIterations++;
		_currentSubsystemState="S_moveWest";
	}
}

/* State S_moveWest: */
void agent::cs::subsystemState_S_moveWest(){
	/* Executing behaviour moveWest */
	executeBehaviour_moveWest(/* */);
	/* Behaviour has been terminated */
	/* Checking initial condition for state S_moveWest: switching to state S_moveNorth*/
	if(initialCondition_From_S_moveWest_To_S_moveNorth()){
		/* incrementing the number determining how many times subsystem has switched between states */
		_subsystemIterations++;
		_currentSubsystemState="S_moveNorth";
	}
}

/* Initial condition for state S_Idle: switching to state S_moveNorth*/
bool agent::cs::initialCondition_From_S_Idle_To_S_moveNorth(){
	/* Initial condition specified by user */
	return  true ;
}

/* Initial condition for state S_moveNorth: switching to state S_moveEast*/
bool agent::cs::initialCondition_From_S_moveNorth_To_S_moveEast(){
	/* Initial condition specified by user */
	return  true ;
}

/* Initial condition for state S_moveEast: switching to state S_moveSouth*/
bool agent::cs::initialCondition_From_S_moveEast_To_S_moveSouth(){
	/* Initial condition specified by user */
	return  true ;
}

/* Initial condition for state S_moveSouth: switching to state S_moveWest*/
bool agent::cs::initialCondition_From_S_moveSouth_To_S_moveWest(){
	/* Initial condition specified by user */
	return  true ;
}

/* Initial condition for state S_moveWest: switching to state S_moveNorth*/
bool agent::cs::initialCondition_From_S_moveWest_To_S_moveNorth(){
	/* Initial condition specified by user */
	return  true ;
}

/* ############################################### */
/* ############### MAIN FUNCTION ################# */
/* ############################################### */
int main(int argc, char ** argv){
	ros::init(argc, argv, "cs");
	/* Initialize subsystem */
	agent::cs *s=new agent::cs();

	/* Start subsystem: cs */
	s->startSubsystem();

	return 0;

} /* END OF MAIN FUNCTION */
#endif // CS

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

#ifndef VE_H
#define VE_H

#include "auxiliary_functions.h"
#include "auxiliary_agent.h"
#include "auxiliary_ve.h"

namespace agent{
	class ve{
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

		ros::Publisher _sender_outputBufferTwist;
		ros::Publisher _sender_moveNorthFinished;
		ros::Publisher _sender_moveEastFinished;
		ros::Publisher _sender_moveSouthFinished;
		ros::Publisher _sender_moveWestFinished;
		std::vector<ros::Publisher > _vectorOfSenderDiagnostics;
		ros::Subscriber _subscriber_turtlePose;
		ros::Subscriber _subscriber_desiredAngle;
		ros::Subscriber _subscriber_beginMovement;
		ros::Subscriber _subscriber_desiredDirection;

		/* Buffers */
		/* Input Buffers */
		turtlesim::Pose _turtlePose;
		std_msgs::Float64 _desiredAngle;
		std_msgs::Int64 _beginMovement;
		std_msgs::Int64 _desiredDirection;

		/* Output Buffers */
		geometry_msgs::Twist _outputBufferTwist;
		std_msgs::Int64 _moveNorthFinished;
		std_msgs::Int64 _moveEastFinished;
		std_msgs::Int64 _moveSouthFinished;
		std_msgs::Int64 _moveWestFinished;

		/* Internal Memory Buffers */
		turtlesim::Pose _lastTurtlePosition;
		std_msgs::Float64 _rotationSpeed;
		std_msgs::Float64 _linearSpeed;
		std_msgs::Int64 _rotationLeft;
		std_msgs::Int64 _rotationRight;
		std_msgs::Float64 _veTerminalCondition;
	public:
		ve();
		/* updates current state based on initial condition */
		void updateCurrentState();
		void startSubsystem();
		void update_turtlePose(const turtlesim::Pose::ConstPtr& msg);
		void update_desiredAngle(const std_msgs::Float64::ConstPtr& msg);
		void update_beginMovement(const std_msgs::Int64::ConstPtr& msg);
		void update_desiredDirection(const std_msgs::Int64::ConstPtr& msg);
		/* Return data from output buffers */
		geometry_msgs::Twist get_outputBufferTwist();
		std_msgs::Int64 get_moveNorthFinished();
		std_msgs::Int64 get_moveEastFinished();
		std_msgs::Int64 get_moveSouthFinished();
		std_msgs::Int64 get_moveWestFinished();

		/* Return data from input buffers */
		turtlesim::Pose get_turtlePose();
		std_msgs::Float64 get_desiredAngle();
		std_msgs::Int64 get_beginMovement();
		std_msgs::Int64 get_desiredDirection();

		/* Return data from internal buffers */
		turtlesim::Pose get_lastTurtlePosition();
		std_msgs::Float64 get_rotationSpeed();
		std_msgs::Float64 get_linearSpeed();
		std_msgs::Int64 get_rotationLeft();
		std_msgs::Int64 get_rotationRight();
		std_msgs::Float64 get_veTerminalCondition();

		/* Set Input Buffers */
		void set_turtlePose(turtlesim::Pose data);
		void set_desiredAngle(std_msgs::Float64 data);
		void set_beginMovement(std_msgs::Int64 data);
		void set_desiredDirection(std_msgs::Int64 data);

		/* Set Internal Buffers */
		void set_lastTurtlePosition(turtlesim::Pose data);
		void set_rotationSpeed(std_msgs::Float64 data);
		void set_linearSpeed(std_msgs::Float64 data);
		void set_rotationLeft(std_msgs::Int64 data);
		void set_rotationRight(std_msgs::Int64 data);
		void set_veTerminalCondition(std_msgs::Float64 data);

		/* Set Output Buffers */
		void set_outputBufferTwist(geometry_msgs::Twist data);
		void set_moveNorthFinished(std_msgs::Int64 data);
		void set_moveEastFinished(std_msgs::Int64 data);
		void set_moveSouthFinished(std_msgs::Int64 data);
		void set_moveWestFinished(std_msgs::Int64 data);

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

		/* Behaviour Init */
		/* Terminal Condition */
		bool terminalCondition_Init(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */);
		/* Error Condition */
		bool errorCondition_Init(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */);
		/* Transition function */
		void transitionFunction_Init(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,geometry_msgs::Twist_outputBufferTwist,std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */);
		/* Send data to other subsystems */
		void sendData_Init(/* geometry_msgs::Twist_outputBufferTwist,std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished */);
		/* Receive data from other subsystems */
		void receiveData_Init(/* */);
		/* Execute behaviour Init*/
		void executeBehaviour_Init(/* */);

		/* Behaviour Idle */
		/* Terminal Condition */
		bool terminalCondition_Idle(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */);
		/* Error Condition */
		bool errorCondition_Idle(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */);
		/* Transition function */
		void transitionFunction_Idle(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,geometry_msgs::Twist_outputBufferTwist,std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */);
		/* Send data to other subsystems */
		void sendData_Idle(/* geometry_msgs::Twist_outputBufferTwist,std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished */);
		/* Receive data from other subsystems */
		void receiveData_Idle(/* */);
		/* Execute behaviour Idle*/
		void executeBehaviour_Idle(/* */);

		/* Behaviour Rotate */
		/* Terminal Condition */
		bool terminalCondition_Rotate(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */);
		/* Error Condition */
		bool errorCondition_Rotate(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */);
		/* Transition function */
		void transitionFunction_Rotate(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,geometry_msgs::Twist_outputBufferTwist,std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */);
		/* Send data to other subsystems */
		void sendData_Rotate(/* geometry_msgs::Twist_outputBufferTwist,std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished */);
		/* Receive data from other subsystems */
		void receiveData_Rotate(/* */);
		/* Execute behaviour Rotate*/
		void executeBehaviour_Rotate(/* */);

		/* Behaviour MoveAlong */
		/* Terminal Condition */
		bool terminalCondition_MoveAlong(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */);
		/* Error Condition */
		bool errorCondition_MoveAlong(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */);
		/* Transition function */
		void transitionFunction_MoveAlong(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,geometry_msgs::Twist_outputBufferTwist,std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */);
		/* Send data to other subsystems */
		void sendData_MoveAlong(/* geometry_msgs::Twist_outputBufferTwist,std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished */);
		/* Receive data from other subsystems */
		void receiveData_MoveAlong(/* */);
		/* Execute behaviour MoveAlong*/
		void executeBehaviour_MoveAlong(/* */);

		/* Declaration of functions responsible for switching subsystem ve between states : S_Init S_Idle S_Rotate S_MoveAlong */
		/* State S_Init: */
		void subsystemState_S_Init();
		/* State S_Idle: */
		void subsystemState_S_Idle();
		/* State S_Rotate: */
		void subsystemState_S_Rotate();
		/* State S_MoveAlong: */
		void subsystemState_S_MoveAlong();
		/* Initial conditions */
		/* Initial condition for state S_Init: switching to state S_Idle*/
		bool initialCondition_From_S_Init_To_S_Idle();
		/* Initial condition for state S_Idle: switching to state S_Rotate*/
		bool initialCondition_From_S_Idle_To_S_Rotate();
		/* Initial condition for state S_Rotate: switching to state S_MoveAlong*/
		bool initialCondition_From_S_Rotate_To_S_MoveAlong();
		/* Initial condition for state S_MoveAlong: switching to state S_Idle*/
		bool initialCondition_From_S_MoveAlong_To_S_Idle();
	}; // END OF ve
} // END OF agent


agent::ve::ve(){
	_n = new ros::NodeHandle();
	_subsystemName="ve";
	_subsystemFrequency=5;
	_currentSubsystemState="S_Init";
	initialiseModel();
}

/* Start subsystem */
void agent::ve::startSubsystem(){
	try{
		do{
			/* Execute behaviour associated with _currentSubsystemState -- choose appropriate state based on _currentSubsystemState */
			if(_currentSubsystemState=="S_Init"){
				subsystemState_S_Init();
				continue;
			}
			if(_currentSubsystemState=="S_Idle"){
				subsystemState_S_Idle();
				continue;
			}
			if(_currentSubsystemState=="S_Rotate"){
				subsystemState_S_Rotate();
				continue;
			}
			if(_currentSubsystemState=="S_MoveAlong"){
				subsystemState_S_MoveAlong();
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

void agent::ve::update_turtlePose(const turtlesim::Pose::ConstPtr& msg){
	_turtlePose=*msg;
}

void agent::ve::update_desiredAngle(const std_msgs::Float64::ConstPtr& msg){
	_desiredAngle=*msg;
}

void agent::ve::update_beginMovement(const std_msgs::Int64::ConstPtr& msg){
	_beginMovement=*msg;
}

void agent::ve::update_desiredDirection(const std_msgs::Int64::ConstPtr& msg){
	_desiredDirection=*msg;
}

void agent::ve::set_turtlePose(turtlesim::Pose data){
	_turtlePose=data;
}

void agent::ve::set_desiredAngle(std_msgs::Float64 data){
	_desiredAngle=data;
}

void agent::ve::set_beginMovement(std_msgs::Int64 data){
	_beginMovement=data;
}

void agent::ve::set_desiredDirection(std_msgs::Int64 data){
	_desiredDirection=data;
}

void agent::ve::set_lastTurtlePosition(turtlesim::Pose data){
	_lastTurtlePosition=data;
}

void agent::ve::set_rotationSpeed(std_msgs::Float64 data){
	_rotationSpeed=data;
}

void agent::ve::set_linearSpeed(std_msgs::Float64 data){
	_linearSpeed=data;
}

void agent::ve::set_rotationLeft(std_msgs::Int64 data){
	_rotationLeft=data;
}

void agent::ve::set_rotationRight(std_msgs::Int64 data){
	_rotationRight=data;
}

void agent::ve::set_veTerminalCondition(std_msgs::Float64 data){
	_veTerminalCondition=data;
}

void agent::ve::set_outputBufferTwist(geometry_msgs::Twist data){
	_outputBufferTwist=data;
}

void agent::ve::set_moveNorthFinished(std_msgs::Int64 data){
	_moveNorthFinished=data;
}

void agent::ve::set_moveEastFinished(std_msgs::Int64 data){
	_moveEastFinished=data;
}

void agent::ve::set_moveSouthFinished(std_msgs::Int64 data){
	_moveSouthFinished=data;
}

void agent::ve::set_moveWestFinished(std_msgs::Int64 data){
	_moveWestFinished=data;
}

std::string agent::ve::getSubsystemName(){
	return _subsystemName;
}

void agent::ve::printInfo(std::string str){
	std::cout<<"["<<_subsystemName<<"] -- info: "<<str<<std::endl;
}

void agent::ve::printError(std::string str){
	std::cerr<<"["<<_subsystemName<<"] -- error : "<<str<<std::endl;
}

void agent::ve::printLog(std::string str){
	#if PRINT_LOG
		std::clog<<"["<<_subsystemName<<"] -- log : "<<str<<std::endl;
	#endif
}

double agent::ve::get_subsystemFrequency(){
	return _subsystemFrequency;
}

void agent::ve::initialiseModel(){
	initialiseSendChannel();
	initialiseSendChannelForDiagnostics();
	initialiseReceiveChannel();
}

void agent::ve::initialiseSendChannel(){
		_sender_outputBufferTwist=_n->advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", CHANNEL_SIZE);
		_sender_moveNorthFinished=_n->advertise<std_msgs::Int64>("moveNorthFinished", CHANNEL_SIZE);
		_sender_moveEastFinished=_n->advertise<std_msgs::Int64>("moveEastFinished", CHANNEL_SIZE);
		_sender_moveSouthFinished=_n->advertise<std_msgs::Int64>("moveSouthFinished", CHANNEL_SIZE);
		_sender_moveWestFinished=_n->advertise<std_msgs::Int64>("moveWestFinished", CHANNEL_SIZE);
}

void agent::ve::initialiseSendChannelForDiagnostics(){
	_vectorOfSenderDiagnostics.push_back(_n->advertise<turtlesim::Pose>("ve/_lastTurtlePosition", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::Float64>("ve/_rotationSpeed", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::Float64>("ve/_linearSpeed", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::Int64>("ve/_rotationLeft", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::Int64>("ve/_rotationRight", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::Float64>("ve/_veTerminalCondition", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::String>("ve/_currentSubsystemState", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::Float64>("ve/_subsystemFrequency", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::String>("ve/_subsystemName", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::Int64>("ve/_subsystemIterations", CHANNEL_SIZE));
	_vectorOfSenderDiagnostics.push_back(_n->advertise<std_msgs::Int64>("ve/_behaviourIterations", CHANNEL_SIZE));
}

void agent::ve::initialiseReceiveChannel(){
	_subscriber_turtlePose=_n->subscribe("/turtle1/pose", CHANNEL_SIZE, &agent::ve::update_turtlePose, this);
	_subscriber_desiredAngle=_n->subscribe("desiredAngle", CHANNEL_SIZE, &agent::ve::update_desiredAngle, this);
	_subscriber_beginMovement=_n->subscribe("beginMovement", CHANNEL_SIZE, &agent::ve::update_beginMovement, this);
	_subscriber_desiredDirection=_n->subscribe("desiredDirection", CHANNEL_SIZE, &agent::ve::update_desiredDirection, this);
}

/* Return data from output buffers */
geometry_msgs::Twist agent::ve::get_outputBufferTwist(){	return _outputBufferTwist;	}

std_msgs::Int64 agent::ve::get_moveNorthFinished(){	return _moveNorthFinished;	}

std_msgs::Int64 agent::ve::get_moveEastFinished(){	return _moveEastFinished;	}

std_msgs::Int64 agent::ve::get_moveSouthFinished(){	return _moveSouthFinished;	}

std_msgs::Int64 agent::ve::get_moveWestFinished(){	return _moveWestFinished;	}

/* Return data from input buffers */
turtlesim::Pose agent::ve::get_turtlePose(){	return _turtlePose;	}

std_msgs::Float64 agent::ve::get_desiredAngle(){	return _desiredAngle;	}

std_msgs::Int64 agent::ve::get_beginMovement(){	return _beginMovement;	}

std_msgs::Int64 agent::ve::get_desiredDirection(){	return _desiredDirection;	}

/* Return data from internal buffers */
turtlesim::Pose agent::ve::get_lastTurtlePosition(){	return _lastTurtlePosition;	}

std_msgs::Float64 agent::ve::get_rotationSpeed(){	return _rotationSpeed;	}

std_msgs::Float64 agent::ve::get_linearSpeed(){	return _linearSpeed;	}

std_msgs::Int64 agent::ve::get_rotationLeft(){	return _rotationLeft;	}

std_msgs::Int64 agent::ve::get_rotationRight(){	return _rotationRight;	}

std_msgs::Float64 agent::ve::get_veTerminalCondition(){	return _veTerminalCondition;	}

/* Send (publish) diagnostic data */
void agent::ve::sendDataForDiagnostics(/* subsystem diagnostic state */){
	_vectorOfSenderDiagnostics[0].publish(_lastTurtlePosition);	_vectorOfSenderDiagnostics[1].publish(_rotationSpeed);	_vectorOfSenderDiagnostics[2].publish(_linearSpeed);	_vectorOfSenderDiagnostics[3].publish(_rotationLeft);	_vectorOfSenderDiagnostics[4].publish(_rotationRight);	_vectorOfSenderDiagnostics[5].publish(_veTerminalCondition);	std_msgs::String temporarString;
	std_msgs::Int64 temporarInt;
	std_msgs::Float64 temporarFloat;
	temporarString.data=_currentSubsystemState;
	_vectorOfSenderDiagnostics[6].publish(temporarString);
	temporarFloat.data=_subsystemFrequency;
	_vectorOfSenderDiagnostics[7].publish(temporarFloat);
	temporarString.data=_subsystemName;
	_vectorOfSenderDiagnostics[8].publish(temporarString);
	temporarInt.data=_subsystemIterations;
	_vectorOfSenderDiagnostics[9].publish(temporarInt);
	temporarInt.data=_behaviourIterations;
	_vectorOfSenderDiagnostics[10].publish(temporarInt);
}
/* Behaviour definitions */
/* Behaviour Init */
/* Terminal Condition */
bool agent::ve::terminalCondition_Init(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */){
	return  true ;
}
/* Error Condition */
bool agent::ve::errorCondition_Init(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */){
	return false;
}
/* Transition function */
void agent::ve::transitionFunction_Init(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,geometry_msgs::Twist_outputBufferTwist,std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */){
	/* Partial transition function - name - tf1_1*/
	_moveNorthFinished.data=0;
	_moveEastFinished.data=0;
	_moveSouthFinished.data=0;
	_moveWestFinished.data=0;
	_outputBufferTwist.angular.z=0;
	_outputBufferTwist.linear.x=0;
	_desiredAngle.data=0;
	_beginMovement.data=0;
	_veTerminalCondition.data=0;
	_desiredDirection.data=0;
	std::cout<<"[VE - INIT]"<<std::endl;
								
}
/* Send data to other subsystems */
void agent::ve::sendData_Init(/* geometry_msgs::Twist_outputBufferTwist,std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished */){
	/* DIAGNOSTICS SEND */
	sendDataForDiagnostics();
	/* END OF DIAGNOSTICS SEND */

	/* TYPICAL SEND CALL */
	_sender_outputBufferTwist.publish(_outputBufferTwist);
	_sender_moveNorthFinished.publish(_moveNorthFinished);
	_sender_moveEastFinished.publish(_moveEastFinished);
	_sender_moveSouthFinished.publish(_moveSouthFinished);
	_sender_moveWestFinished.publish(_moveWestFinished);
	/* END OF TYPICAL SEND CALL */

	std::cout<<"[Behaviour Init] -- Sending Data"<<std::endl;
}
/* Receive data from other subsystems */
void agent::ve::receiveData_Init(/* */){
	/* TYPICAL RECEIVE CALL */
	ros::spinOnce();
	/* END OF TYPICAL RECEIVE CALL */

	std::cout<<"[Behaviour Init] -- Receiving Data"<<std::endl;
}
/* Execute behaviour Init*/
void agent::ve::executeBehaviour_Init(/* */){
	bool stopBehaviourIteration=false;
	/* Execution of a single iteration of a behaviour Init */
	_behaviourIterations=0;
	AuxiliaryFunctions rate = AuxiliaryFunctions(_subsystemFrequency);
	/* Starts execution! */
	do{
		/* Sleep is a method from class AuxiliaryFunctions which executes sleep from ros::Rate */
		rate.sleep();
		/* Calculates transition function -- output and internal buffers can only be modified by this function! */
		transitionFunction_Init();
		/* Sends data! */
		sendData_Init();
		/* Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations */
		_behaviourIterations++;
		/* Receives data! */
		receiveData_Init();
		/* Check both conditions, i.e. terminal condition and error condition */
		stopBehaviourIteration = terminalCondition_Init() || errorCondition_Init();
	}
	while(!stopBehaviourIteration && AuxiliaryFunctions::isSubsystemOK()); /* Iterate within the while loop until stopBehaviourIteration is set true, i.e. one of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise subsystem must have been switched to another state or SIGINT was sent */
	/* Stops execution! */
}


/* Behaviour Idle */
/* Terminal Condition */
bool agent::ve::terminalCondition_Idle(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */){
	return  _beginMovement.data==1 && _desiredDirection.data!=0 ;
}
/* Error Condition */
bool agent::ve::errorCondition_Idle(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */){
	return false;
}
/* Transition function */
void agent::ve::transitionFunction_Idle(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,geometry_msgs::Twist_outputBufferTwist,std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */){
	/* Partial transition function - name - tf1_1*/
	_rotationSpeed.data=ROTATE_ANGLE;
	_linearSpeed.data=MOVE_ALONG_SPEED;
	_outputBufferTwist.linear.x=0; 							// do not move in x-y plane
	_outputBufferTwist.angular.z=0;							// do not rotate around z-axis
	_lastTurtlePosition=_turtlePose; // update turtle pose - for move along behaviour
	std::cout<<"[VE - IDLE] - direction="<<_desiredDirection.data<<" current turtle position x="<<_turtlePose.x<<" y="<<_turtlePose.y<<std::endl;
								
}
/* Send data to other subsystems */
void agent::ve::sendData_Idle(/* geometry_msgs::Twist_outputBufferTwist,std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished */){
	/* DIAGNOSTICS SEND */
	sendDataForDiagnostics();
	/* END OF DIAGNOSTICS SEND */

	/* TYPICAL SEND CALL */
	_sender_outputBufferTwist.publish(_outputBufferTwist);
	_sender_moveNorthFinished.publish(_moveNorthFinished);
	_sender_moveEastFinished.publish(_moveEastFinished);
	_sender_moveSouthFinished.publish(_moveSouthFinished);
	_sender_moveWestFinished.publish(_moveWestFinished);
	/* END OF TYPICAL SEND CALL */

	std::cout<<"[Behaviour Idle] -- Sending Data"<<std::endl;
}
/* Receive data from other subsystems */
void agent::ve::receiveData_Idle(/* */){
	/* TYPICAL RECEIVE CALL */
	ros::spinOnce();
	/* END OF TYPICAL RECEIVE CALL */

	std::cout<<"[Behaviour Idle] -- Receiving Data"<<std::endl;
}
/* Execute behaviour Idle*/
void agent::ve::executeBehaviour_Idle(/* */){
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


/* Behaviour Rotate */
/* Terminal Condition */
bool agent::ve::terminalCondition_Rotate(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */){
	return  _veTerminalCondition.data==1 ;
}
/* Error Condition */
bool agent::ve::errorCondition_Rotate(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */){
	return false;
}
/* Transition function */
void agent::ve::transitionFunction_Rotate(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,geometry_msgs::Twist_outputBufferTwist,std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */){
	/* Partial transition function - name - tf1_1*/
	// code
	_veTerminalCondition.data=0;
	_beginMovement.data=0;
	_lastTurtlePosition.x=_turtlePose.x; // update turtle pose - for move along behaviour
	_lastTurtlePosition.y=_turtlePose.y; // update turtle pose - for move along behaviour
	// #########################
	// do something
	// ###################################################################################################################
	// get quarter
	std_msgs::Int64 quarter;
	if(_turtlePose.theta>=GO_EAST_DIRECTION && _turtlePose.theta<GO_NORTH_DIRECTION){
		quarter.data=1;
	}
	else if(_turtlePose.theta>=GO_NORTH_DIRECTION && _turtlePose.theta<=GO_WEST_DIRECTION){
		quarter.data=2;
	}
	else if(_turtlePose.theta>=-GO_WEST_DIRECTION && _turtlePose.theta<GO_SOUTH_DIRECTION){
		quarter.data=3;
	}
	else if(_turtlePose.theta>=GO_SOUTH_DIRECTION && _turtlePose.theta<GO_EAST_DIRECTION){
		quarter.data=4;
	}
	std::cout<<"[VE - ROTATE] - CURRENT QUARTER="<<quarter.data<<std::endl;
	// ##################################################################################################################
	if(_desiredDirection.data == GO_NORTH){
		if(quarter.data==1) {_outputBufferTwist.angular.z=_rotationSpeed.data; _rotationLeft.data=1; }
		if(quarter.data==2) {_outputBufferTwist.angular.z=-_rotationSpeed.data; _rotationRight.data=1; }
		if(quarter.data==3) {_outputBufferTwist.angular.z=-_rotationSpeed.data; _rotationRight.data=1; }
		if(quarter.data==4) {_outputBufferTwist.angular.z=_rotationSpeed.data; _rotationLeft.data=1; }
	}
	else if(_desiredDirection.data == GO_EAST){
		if(quarter.data==1) {_outputBufferTwist.angular.z=-_rotationSpeed.data; _rotationRight.data=1; }
		if(quarter.data==2) {_outputBufferTwist.angular.z=-_rotationSpeed.data; _rotationRight.data=1; }
		if(quarter.data==3) {_outputBufferTwist.angular.z=_rotationSpeed.data; _rotationLeft.data=1; }
		if(quarter.data==4) {_outputBufferTwist.angular.z=_rotationSpeed.data; _rotationLeft.data=1; }
	}
	else if(_desiredDirection.data == GO_SOUTH){
		if(quarter.data==1) {_outputBufferTwist.angular.z=-_rotationSpeed.data; _rotationRight.data=1; }
		if(quarter.data==2) {_outputBufferTwist.angular.z=_rotationSpeed.data; _rotationLeft.data=1; }
		if(quarter.data==3) {_outputBufferTwist.angular.z=_rotationSpeed.data; _rotationLeft.data=1; }
		if(quarter.data==4) {_outputBufferTwist.angular.z=-_rotationSpeed.data; _rotationRight.data=1; }
	}
	else if(_desiredDirection.data == GO_WEST){
		if(quarter.data==1) {_outputBufferTwist.angular.z=_rotationSpeed.data; _rotationLeft.data=1; }
		if(quarter.data==2) {_outputBufferTwist.angular.z=_rotationSpeed.data; _rotationLeft.data=1; }
		if(quarter.data==3) {_outputBufferTwist.angular.z=-_rotationSpeed.data; _rotationRight.data=1; }
		if(quarter.data==4) {_outputBufferTwist.angular.z=-_rotationSpeed.data; _rotationRight.data=1; }
	}
	std_msgs::Float64 tmp;
	tmp.data=abs(_desiredAngle.data -_turtlePose.theta);
	if(	_rotationLeft.data==1 && _rotationRight.data==1){
		_rotationSpeed.data/=2;
		std::cout<<"[VE - ROTATE] - _rotationSpeed - divided by 2 - _rotationSpeed.data="<<_rotationSpeed.data<<std::endl;
		_rotationLeft.data=0;
		_rotationRight.data=0;
	}
	_veTerminalCondition.data=tmp.data < EPSILON_ROTATE;
	// #########################
	std::cout<<"[VE - ROTATE] - direction="<<_desiredDirection.data<<std::endl;
									
}
/* Send data to other subsystems */
void agent::ve::sendData_Rotate(/* geometry_msgs::Twist_outputBufferTwist,std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished */){
	/* DIAGNOSTICS SEND */
	sendDataForDiagnostics();
	/* END OF DIAGNOSTICS SEND */

	/* TYPICAL SEND CALL */
	_sender_outputBufferTwist.publish(_outputBufferTwist);
	_sender_moveNorthFinished.publish(_moveNorthFinished);
	_sender_moveEastFinished.publish(_moveEastFinished);
	_sender_moveSouthFinished.publish(_moveSouthFinished);
	_sender_moveWestFinished.publish(_moveWestFinished);
	/* END OF TYPICAL SEND CALL */

	std::cout<<"[Behaviour Rotate] -- Sending Data"<<std::endl;
}
/* Receive data from other subsystems */
void agent::ve::receiveData_Rotate(/* */){
	/* TYPICAL RECEIVE CALL */
	ros::spinOnce();
	/* END OF TYPICAL RECEIVE CALL */

	std::cout<<"[Behaviour Rotate] -- Receiving Data"<<std::endl;
}
/* Execute behaviour Rotate*/
void agent::ve::executeBehaviour_Rotate(/* */){
	bool stopBehaviourIteration=false;
	/* Execution of a single iteration of a behaviour Rotate */
	_behaviourIterations=0;
	AuxiliaryFunctions rate = AuxiliaryFunctions(_subsystemFrequency);
	/* Starts execution! */
	do{
		/* Sleep is a method from class AuxiliaryFunctions which executes sleep from ros::Rate */
		rate.sleep();
		/* Calculates transition function -- output and internal buffers can only be modified by this function! */
		transitionFunction_Rotate();
		/* Sends data! */
		sendData_Rotate();
		/* Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations */
		_behaviourIterations++;
		/* Receives data! */
		receiveData_Rotate();
		/* Check both conditions, i.e. terminal condition and error condition */
		stopBehaviourIteration = terminalCondition_Rotate() || errorCondition_Rotate();
	}
	while(!stopBehaviourIteration && AuxiliaryFunctions::isSubsystemOK()); /* Iterate within the while loop until stopBehaviourIteration is set true, i.e. one of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise subsystem must have been switched to another state or SIGINT was sent */
	/* Stops execution! */
}


/* Behaviour MoveAlong */
/* Terminal Condition */
bool agent::ve::terminalCondition_MoveAlong(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */){
	return  _veTerminalCondition.data==1 ;
}
/* Error Condition */
bool agent::ve::errorCondition_MoveAlong(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */){
	return false;
}
/* Transition function */
void agent::ve::transitionFunction_MoveAlong(/* turtlesim::Pose_turtlePose,std_msgs::Float64_desiredAngle,std_msgs::Int64_beginMovement,std_msgs::Int64_desiredDirection,geometry_msgs::Twist_outputBufferTwist,std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished,turtlesim::Pose_lastTurtlePosition,std_msgs::Float64_rotationSpeed,std_msgs::Float64_linearSpeed,std_msgs::Int64_rotationLeft,std_msgs::Int64_rotationRight,std_msgs::Float64_veTerminalCondition */){
	/* Partial transition function - name - tf1_1*/
	// code
	_veTerminalCondition.data=0;
	_beginMovement.data=0;
	_moveNorthFinished.data=0;
	_moveEastFinished.data=0;
	_moveSouthFinished.data=0;
	_moveWestFinished.data=0;
	_outputBufferTwist.angular.z=0;
	// #########################
	// do something
	std_msgs::Float64 distance;
	distance.data= (_lastTurtlePosition.x-_turtlePose.x)*(_lastTurtlePosition.x-_turtlePose.x)+(_lastTurtlePosition.y-_turtlePose.y)*(_lastTurtlePosition.y-_turtlePose.y);
	distance.data=sqrt(distance.data);
	_linearSpeed.data=((MAX_DISTANCE-distance.data)/MAX_DISTANCE)*MOVE_ALONG_SPEED;
	// move along without rotation
	_outputBufferTwist.linear.x=_linearSpeed.data;
	_veTerminalCondition.data=abs(MAX_DISTANCE-distance.data)<EPSILON_MOVE;
	// #########################
	if(_veTerminalCondition.data==1)
	{
		if(_desiredDirection.data==GO_NORTH){
			_moveNorthFinished.data=1;
		}
		else if(_desiredDirection.data==GO_EAST){
			_moveEastFinished.data=1;
		}
		else if(_desiredDirection.data==GO_SOUTH){
			_moveSouthFinished.data=1;
		}
		else if(_desiredDirection.data==GO_WEST){
			_moveWestFinished.data=1;
		}
	}
	std::cout<<"[VE - MOVE ALONG] - direction="<<_desiredDirection.data<<" distance="<<distance.data<<" terminalCondition="<<_veTerminalCondition.data<<" last position x="<<_lastTurtlePosition.x<<" y="<<_lastTurtlePosition.y<<"current turtle position x="<<_turtlePose.x<<" y="<<_turtlePose.y<<std::endl;
									
}
/* Send data to other subsystems */
void agent::ve::sendData_MoveAlong(/* geometry_msgs::Twist_outputBufferTwist,std_msgs::Int64_moveNorthFinished,std_msgs::Int64_moveEastFinished,std_msgs::Int64_moveSouthFinished,std_msgs::Int64_moveWestFinished */){
	/* DIAGNOSTICS SEND */
	sendDataForDiagnostics();
	/* END OF DIAGNOSTICS SEND */

	/* TYPICAL SEND CALL */
	_sender_outputBufferTwist.publish(_outputBufferTwist);
	_sender_moveNorthFinished.publish(_moveNorthFinished);
	_sender_moveEastFinished.publish(_moveEastFinished);
	_sender_moveSouthFinished.publish(_moveSouthFinished);
	_sender_moveWestFinished.publish(_moveWestFinished);
	/* END OF TYPICAL SEND CALL */

	std::cout<<"[Behaviour MoveAlong] -- Sending Data"<<std::endl;
}
/* Receive data from other subsystems */
void agent::ve::receiveData_MoveAlong(/* */){
	/* TYPICAL RECEIVE CALL */
	ros::spinOnce();
	/* END OF TYPICAL RECEIVE CALL */

	std::cout<<"[Behaviour MoveAlong] -- Receiving Data"<<std::endl;
}
/* Execute behaviour MoveAlong*/
void agent::ve::executeBehaviour_MoveAlong(/* */){
	bool stopBehaviourIteration=false;
	/* Execution of a single iteration of a behaviour MoveAlong */
	_behaviourIterations=0;
	AuxiliaryFunctions rate = AuxiliaryFunctions(_subsystemFrequency);
	/* Starts execution! */
	do{
		/* Sleep is a method from class AuxiliaryFunctions which executes sleep from ros::Rate */
		rate.sleep();
		/* Calculates transition function -- output and internal buffers can only be modified by this function! */
		transitionFunction_MoveAlong();
		/* Sends data! */
		sendData_MoveAlong();
		/* Updates index! -- i.e. i:i+1 -- increment number of behaviour iterations and number of subsystem iterations */
		_behaviourIterations++;
		/* Receives data! */
		receiveData_MoveAlong();
		/* Check both conditions, i.e. terminal condition and error condition */
		stopBehaviourIteration = terminalCondition_MoveAlong() || errorCondition_MoveAlong();
	}
	while(!stopBehaviourIteration && AuxiliaryFunctions::isSubsystemOK()); /* Iterate within the while loop until stopBehaviourIteration is set true, i.e. one of error and terminal condition is fulfilled and isSubsystemOK is true. Otherwise subsystem must have been switched to another state or SIGINT was sent */
	/* Stops execution! */
}

/* Definition of functions responsible for switching subsystem ve between states : S_Init S_Idle S_Rotate S_MoveAlong */
/* State S_Init: */
void agent::ve::subsystemState_S_Init(){
	/* Executing behaviour Init */
	executeBehaviour_Init(/* */);
	/* Behaviour has been terminated */
	/* Checking initial condition for state S_Init: switching to state S_Idle*/
	if(initialCondition_From_S_Init_To_S_Idle()){
		/* incrementing the number determining how many times subsystem has switched between states */
		_subsystemIterations++;
		_currentSubsystemState="S_Idle";
	}
}

/* State S_Idle: */
void agent::ve::subsystemState_S_Idle(){
	/* Executing behaviour Idle */
	executeBehaviour_Idle(/* */);
	/* Behaviour has been terminated */
	/* Checking initial condition for state S_Idle: switching to state S_Rotate*/
	if(initialCondition_From_S_Idle_To_S_Rotate()){
		/* incrementing the number determining how many times subsystem has switched between states */
		_subsystemIterations++;
		_currentSubsystemState="S_Rotate";
	}
}

/* State S_Rotate: */
void agent::ve::subsystemState_S_Rotate(){
	/* Executing behaviour Rotate */
	executeBehaviour_Rotate(/* */);
	/* Behaviour has been terminated */
	/* Checking initial condition for state S_Rotate: switching to state S_MoveAlong*/
	if(initialCondition_From_S_Rotate_To_S_MoveAlong()){
		/* incrementing the number determining how many times subsystem has switched between states */
		_subsystemIterations++;
		_currentSubsystemState="S_MoveAlong";
	}
}

/* State S_MoveAlong: */
void agent::ve::subsystemState_S_MoveAlong(){
	/* Executing behaviour MoveAlong */
	executeBehaviour_MoveAlong(/* */);
	/* Behaviour has been terminated */
	/* Checking initial condition for state S_MoveAlong: switching to state S_Idle*/
	if(initialCondition_From_S_MoveAlong_To_S_Idle()){
		/* incrementing the number determining how many times subsystem has switched between states */
		_subsystemIterations++;
		_currentSubsystemState="S_Idle";
	}
}

/* Initial condition for state S_Init: switching to state S_Idle*/
bool agent::ve::initialCondition_From_S_Init_To_S_Idle(){
	/* Initial condition specified by user */
	return  true ;
}

/* Initial condition for state S_Idle: switching to state S_Rotate*/
bool agent::ve::initialCondition_From_S_Idle_To_S_Rotate(){
	/* Initial condition specified by user */
	return  true ;
}

/* Initial condition for state S_Rotate: switching to state S_MoveAlong*/
bool agent::ve::initialCondition_From_S_Rotate_To_S_MoveAlong(){
	/* Initial condition specified by user */
	return  true ;
}

/* Initial condition for state S_MoveAlong: switching to state S_Idle*/
bool agent::ve::initialCondition_From_S_MoveAlong_To_S_Idle(){
	/* Initial condition specified by user */
	return  true ;
}

/* ############################################### */
/* ############### MAIN FUNCTION ################# */
/* ############################################### */
int main(int argc, char ** argv){
	ros::init(argc, argv, "ve");
	/* Initialize subsystem */
	agent::ve *s=new agent::ve();

	/* Start subsystem: ve */
	s->startSubsystem();

	return 0;

} /* END OF MAIN FUNCTION */
#endif // VE

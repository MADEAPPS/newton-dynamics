/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


#ifndef _D_AI_AGENT_VEHICLE_CONTROLLER_H_
#define _D_AI_AGENT_VEHICLE_CONTROLLER_H_

#include "dAIAgent.h"
#include "dAIAgentState.h"

/*
class dAIAgentVehicleController: public dAIAgent
{
	public: 
	enum
	{
		m_engineOn,
		m_engineOff,
	};

	class dEngineOnState: public dAIAgentState
	{
		public:
		dEngineOnState (dAIAgentVehicleController* const agent)
			:dAIAgentState (agent)
		{
		}
		virtual void Enter(int threadID);
		virtual void Exit(int threadID);
	};

	// this state do nothing
	class dEngineOffState: public dAIAgentState
	{
		public:
		dEngineOffState (dAIAgentVehicleController* const agent)
			:dAIAgentState (agent)
		{
		}
	};

	dAIAgentVehicleController (dAI* const manager, CustomWheelVehicle* const vehicle);
	virtual ~dAIAgentVehicleController ();
	virtual void Update (dFloat timestep, int threadID);

	CustomWheelVehicle* GetJoint() const ;

	private:
	CustomWheelVehicle* m_vehicle;
};


inline CustomWheelVehicle* dAIAgentVehicleController::GetJoint() const 
{
	return m_vehicle;
}
*/

#endif 


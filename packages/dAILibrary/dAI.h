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


#ifndef _D_NEWTON_AI_H_
#define _D_NEWTON_AI_H_

#include <Newton.h>
#include <NewtonAI.h>
#include <CustomJoint.h>


#ifdef _MSC_VER 
	#pragma warning (disable: 4100) //unreferenced formal parameter
#endif


class dAIAgent;

class dAI
{
	public:
	dAI ();
	virtual ~dAI ();

	NewtonAIWorld* GetAI () const;
	dAIAgent* GetGameLogicAgent() const;
	NewtonAIAgent* GetGameLogicNewtonAgent() const;

	dAIAgent* GetFirstAgent() const;
	dAIAgent* GetNextAgent(dAIAgent* const agent) const;

	void Update (dFloat timestep);

	protected:
	NewtonAIWorld* m_ai;
};

inline NewtonAIWorld* dAI::GetAI () const
{
	return m_ai;
}



#endif 


/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


#ifndef D_PLAYER_CONTROLLER_SOLVER_H_
#define D_PLAYER_CONTROLLER_SOLVER_H_


#include "dPlayerControllerContactSolver.h"


class dPlayerController;

class dPlayerControllerImpulseSolver
{
	public:
	dPlayerControllerImpulseSolver(dPlayerController* const controller);
	void Reset(dPlayerController* const controller);
	void AddAngularRows();
	int AddLinearRow(const dVector& dir, const dVector& r, dFloat speed, dFloat low, dFloat high, int normalIndex = -1);
	int AddContactRow(const NewtonWorldConvexCastReturnInfo* const contact, const dVector& dir, const dVector& r, dFloat speed, dFloat low, dFloat high, int normalIndex = -1);
	dVector CalculateImpulse();
	void ApplyReaction(dFloat timestep);

	dMatrix m_invInertia;
	dVector m_veloc;
	dVector m_zero;
	dComplementaritySolver::dJacobianPair m_jacobianPairs[D_PLAYER_MAX_ROWS];
	const NewtonWorldConvexCastReturnInfo* m_contactPoint[D_PLAYER_MAX_ROWS];
	dFloat m_rhs[D_PLAYER_MAX_ROWS];
	dFloat m_low[D_PLAYER_MAX_ROWS];
	dFloat m_high[D_PLAYER_MAX_ROWS];
	dFloat m_impulseMag[D_PLAYER_MAX_ROWS];
	int m_normalIndex[D_PLAYER_MAX_ROWS];
	dFloat m_mass;
	dFloat m_invMass;
	int m_rowCount;
};

#endif 


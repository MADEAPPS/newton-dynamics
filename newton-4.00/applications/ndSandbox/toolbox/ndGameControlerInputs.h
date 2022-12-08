/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __D_GAME_CONTROLLER_INPUTS__
#define __D_GAME_CONTROLLER_INPUTS__

#include "ndSandboxStdafx.h"
class ndDemoEntityManager;

class ndGameControllerInputs
{
	public:
	enum ndAxis
	{
		m_steeringWheel,
		m_gasPedal,
		m_brakePedal,
		m_axisCount,
	};

	enum ndButtons
	{
		m_handBreakButton = 0,
		m_upGearButton,
		m_downGearButton,
		m_neutralGearButton,
		m_ignitionButton,
		m_reverseGearButton,
		m_automaticGearBoxButton,
		m_parkingButton,
		m_buttonCount
	};

	ndFixSizeArray<char, 32> m_buttons;
	ndFixSizeArray<ndFloat32, 8> m_axis;

	ndGameControllerInputs();
	~ndGameControllerInputs();
	void Update(ndDemoEntityManager* const scene);

	private:
	void GetKeyboardInputs(ndDemoEntityManager* const scene);
	void GetWheelJoystickInputs(ndDemoEntityManager* const scene);

	ndFloat32 m_keyBoardSteerAngle;
};


#endif
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

#ifndef __ND_GAME_CONTROLLER_INPUTS__
#define __ND_GAME_CONTROLLER_INPUTS__

#include "ndSandboxStdafx.h"
class ndDemoEntityManager;

class ndGameControllerInputs
{
	public:
	enum ndAxis
	{
		m_azis_00,
		m_azis_01,
		m_azis_02,
		m_azis_03,
		m_axisCount,
	};

	enum ndButtons
	{
		m_button_00 = 0,
		m_button_01,
		m_button_02,
		m_button_03,
		m_button_04,
		m_button_05,
		m_button_06,
		m_button_07,
		m_button_08,
		m_buttonCount
	};

	ndFixSizeArray<char, 32> m_buttons;
	ndFixSizeArray<ndFloat32, 8> m_axis;

	ndGameControllerInputs();
	~ndGameControllerInputs();
	void Update(ndDemoEntityManager* const scene);

	private:
	bool GetKeyboardInputs(ndDemoEntityManager* const scene);
	void GetJoystickInputs(ndDemoEntityManager* const scene);
	void GetXboxJoystickInputs(ndDemoEntityManager* const scene);
	void GetWheelJoystickInputs(ndDemoEntityManager* const scene);

	ndFloat32 m_keyBoardSteerAngle;
};


#endif
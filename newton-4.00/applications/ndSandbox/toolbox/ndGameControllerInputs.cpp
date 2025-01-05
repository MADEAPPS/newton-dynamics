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

#include "ndSandboxStdafx.h"
#include "ndDemoEntityManager.h"
#include "ndGameControlerInputs.h"

ndGameControllerInputs::ndGameControllerInputs()
	:m_keyBoardSteerAngle(0.0f)
{
}

ndGameControllerInputs::~ndGameControllerInputs()
{
}

bool ndGameControllerInputs::GetKeyboardInputs(ndDemoEntityManager* const scene)
{
	m_buttons.SetCount(m_buttonCount);
	m_buttons[m_button_00] = scene->GetKeyState(' ');
	m_buttons[m_button_01] = scene->GetKeyState('>') || scene->GetKeyState('.');
	m_buttons[m_button_02] = scene->GetKeyState('<') || scene->GetKeyState(',');
	m_buttons[m_button_03] = scene->GetKeyState('N');
	m_buttons[m_button_04] = scene->GetKeyState('I');
	m_buttons[m_button_05] = scene->GetKeyState('R');
	m_buttons[m_button_06] = scene->GetKeyState('?') || scene->GetKeyState('/');
	m_buttons[m_button_07] = scene->GetKeyState('P');
	m_buttons[m_button_08] = scene->GetKeyState('C');
	
	m_axis.SetCount(m_axisCount);
	ndFloat32 steerAngle = ndFloat32(scene->GetKeyState('A')) - ndFloat32(scene->GetKeyState('D'));
	m_keyBoardSteerAngle += (steerAngle - m_keyBoardSteerAngle) * 0.10f;
	m_keyBoardSteerAngle = (m_keyBoardSteerAngle < (1.0e-4f)) ? (m_keyBoardSteerAngle > (-1.0e-4f) ? 0.0f : m_keyBoardSteerAngle) : m_keyBoardSteerAngle;
	m_axis[m_azis_00] = m_keyBoardSteerAngle;
	m_axis[m_azis_01] = ndFloat32(scene->GetKeyState('W')) ? 1.0f : 0.0f;
	m_axis[m_azis_02] = ndFloat32(scene->GetKeyState('S') ? 1.0f : 0.0f);
	m_axis[m_azis_03] = ndFloat32(0.0f);

//static int frame;
//frame++;
//bool check = false;
//for (ndInt32 i = 0; i < m_buttons.GetCount(); ++i)
//{
//	if (m_buttons[i])
//	{
//		check = true;
//	}
//}
//if (check)
//{
//	ndTrace(("\n%d: ", frame));
//	for (ndInt32 i = 0; i < m_buttons.GetCount(); ++i)
//	{
//		if (m_buttons[i])
//		{
//			ndTrace(("(%d %d) ", i, m_buttons[i]));
//		}
//	}
//}
	
	char ret = false;
	for (ndInt32 i = 0; i < m_buttons.GetCount(); ++i)
	{
		ret = ret | m_buttons[i];
	}
	for (ndInt32 i = 0; i < m_axis.GetCount(); ++i)
	{
		ret = ret | ((m_axis[i] != 0.0f) ? 1 : 0);
	}

	return ret ? true : false;
}

void ndGameControllerInputs::GetJoystickInputs(ndDemoEntityManager* const scene)
{
	static ndFixSizeArray<int, 8> axisMapping;
	static ndFixSizeArray<int, 32> buttonMapping;

	ndFixSizeArray<char, 32> unmappedButtons;
	ndFixSizeArray<ndFloat32, 8> unmappedAxis;

	scene->GetJoystickAxis(unmappedAxis);
	scene->GetJoystickButtons(unmappedButtons);

	if (!buttonMapping.GetCount())
	{
		if (!buttonMapping.GetCount())
		{
			for (ndInt32 i = 0; i < buttonMapping.GetCapacity(); ++i)
			{
				buttonMapping.PushBack(m_buttonCount);
			}

			buttonMapping[0] = m_button_00;		//m_handBreakButton
			buttonMapping[3] = m_button_01;		//m_upGearButton
			buttonMapping[2] = m_button_02;		//m_downGearButton
			buttonMapping[5] = m_button_03;		//m_neutralGearButton
			buttonMapping[10] = m_button_04;	//m_ignitionButton
			buttonMapping[4] = m_button_05;		//m_reverseGearButton
			buttonMapping[11] = m_button_06;	//m_automaticGearBoxButton
			buttonMapping[1] = m_button_07;		//m_parkingButton
			buttonMapping[8] = m_button_08;		//m_playerButton
		}
	}

	m_buttons.SetCount(m_buttonCount);
	for (ndInt32 i = 0; i < unmappedButtons.GetCount(); ++i)
	{
		ndInt32 j = buttonMapping[i];
		m_buttons[j] = unmappedButtons[i];
	}

	if (!axisMapping.GetCount())
	{
		for (ndInt32 i = 0; i < axisMapping.GetCapacity(); ++i)
		{
			axisMapping.PushBack(m_axisCount);
		}
		axisMapping[0] = m_azis_00;
		axisMapping[1] = m_azis_01;
	}

	m_axis.SetCount(m_axisCount);
	for (ndInt32 i = 0; i < axisMapping.GetCount(); i++)
	{
		m_axis[axisMapping[i]] = unmappedAxis[i];
	}

	m_axis[m_azis_00] = -m_axis[m_azis_00] * m_axis[m_azis_00] * m_axis[m_azis_00];

	ndFloat32 gas = ndMin (m_axis[m_azis_01], ndFloat32 (0.0f));
	ndFloat32 brake = ndMax(m_axis[m_azis_01], ndFloat32(0.0f));

	m_axis[m_azis_01] = gas * gas;
	m_axis[m_azis_02] = brake * brake;
}

void ndGameControllerInputs::GetXboxJoystickInputs(ndDemoEntityManager* const scene)
{
	static ndFixSizeArray<int, 8> axisMapping;
	static ndFixSizeArray<int, 32> buttonMapping;

	ndFixSizeArray<char, 32> unmappedButtons;
	ndFixSizeArray<ndFloat32, 8> unmappedAxis;

	scene->GetJoystickAxis(unmappedAxis);
	scene->GetJoystickButtons(unmappedButtons);

	if (!buttonMapping.GetCount())
	{
		if (!buttonMapping.GetCount())
		{
			for (ndInt32 i = 0; i < buttonMapping.GetCapacity(); ++i)
			{
				buttonMapping.PushBack(m_buttonCount);
			}

			buttonMapping[11] = m_button_00; //m_handBreakButton
			buttonMapping[5] = m_button_01;  //m_downGearButton
			buttonMapping[4] = m_button_02;  //m_upGearButton
			buttonMapping[12] = m_button_03; //m_neutralGearButton
			buttonMapping[7] = m_button_04;  //m_ignitionButton
			buttonMapping[13] = m_button_05; //m_reverseGearButton
			buttonMapping[8] = m_button_06;  //m_automaticGearBoxButton
			buttonMapping[10] = m_button_07; //m_parkingButton
			buttonMapping[6] = m_button_08;  //m_playerButton
		}
	}

	m_buttons.SetCount(m_buttonCount);
	for (ndInt32 i = 0; i < unmappedButtons.GetCount(); ++i)
	{
		m_buttons[buttonMapping[i]] = unmappedButtons[i];
	}

	if (!axisMapping.GetCount())
	{
		for (ndInt32 i = 0; i < axisMapping.GetCapacity(); ++i)
		{
			axisMapping.PushBack(m_axisCount);
		}
		axisMapping[2] = m_azis_00;
		axisMapping[5] = m_azis_01;
		axisMapping[4] = m_azis_02;
	}

	m_axis.SetCount(m_axisCount);
	for (ndInt32 i = 0; i < axisMapping.GetCount(); i++)
	{
		m_axis[axisMapping[i]] = unmappedAxis[i];
	}
	m_axis[m_azis_00] = -m_axis[m_azis_00] * m_axis[m_azis_00] * m_axis[m_azis_00];

	ndFloat32 steer = m_axis[m_azis_00] * m_axis[m_azis_00] * m_axis[m_azis_00];
	ndFloat32 gas = (m_axis[m_azis_01] + ndFloat32(1.0f)) * ndFloat32(0.5f);
	ndFloat32 brake = (m_axis[m_azis_02] + ndFloat32(1.0f)) * ndFloat32(0.5f);

	m_axis[m_azis_00] = steer;
	m_axis[m_azis_01] = gas * gas;
	m_axis[m_azis_02] = brake * brake;
}

void ndGameControllerInputs::GetWheelJoystickInputs(ndDemoEntityManager* const scene)
{
	// logitech g920 mapping
	static ndFixSizeArray<int, 8> axisMapping;
	static ndFixSizeArray<int, 32> buttonMapping;

	ndFixSizeArray<char, 32> unmappedButtons;
	ndFixSizeArray<ndFloat32, 8> unmappedAxis;

	scene->GetJoystickAxis(unmappedAxis);
	scene->GetJoystickButtons(unmappedButtons);

	if (!buttonMapping.GetCount())
	{
		if (!buttonMapping.GetCount())
		{
			for (ndInt32 i = 0; i < buttonMapping.GetCapacity(); ++i)
			{
				buttonMapping.PushBack(m_buttonCount);
			}
			buttonMapping[1] = m_button_00;     //m_handBreakButton
			buttonMapping[4] = m_button_01;		//m_downGearButton
			buttonMapping[5] = m_button_02;		//m_upGearButton
			buttonMapping[6] = m_button_03;		//m_neutralGearButton
			buttonMapping[10] = m_button_04;	//m_ignitionButton
			buttonMapping[8] = m_button_05;		//m_reverseGearButton
			buttonMapping[9] = m_button_06;		//m_automaticGearBoxButton
			buttonMapping[20] = m_button_07;	//m_parkingButton
			buttonMapping[7] = m_button_08;		//m_playerButton
		}										
	}

	m_buttons.SetCount(m_buttonCount);
	for (ndInt32 i = 0; i < unmappedButtons.GetCount(); ++i)
	{
		m_buttons[buttonMapping[i]] = unmappedButtons[i];
	}

//static int frame;
//frame++;
//bool check = false;
//for (ndInt32 i = 0; i < m_buttons.GetCount(); ++i)
//{
//	if (m_buttons[i])
//		check = true;
//}
//if (check)
//{
//	ndTrace(("\n%d: ", frame));
//	for (ndInt32 i = 0; i < m_buttons.GetCount(); ++i)
//	{
//		if (m_buttons[i])
//			ndTrace(("(%d %d) ", i, m_buttons[i]));
//	}
//}

	if (!axisMapping.GetCount())
	{
		for (ndInt32 i = 0; i < axisMapping.GetCapacity(); ++i)
		{
			axisMapping.PushBack(m_axisCount);
		}
		axisMapping[0] = m_azis_00;
		axisMapping[1] = m_azis_01;
		axisMapping[2] = m_azis_02;
	}

	m_axis.SetCount(m_axisCount);
	for (ndInt32 i = 0; i < axisMapping.GetCount(); i++)
	{
		m_axis[axisMapping[i]] = unmappedAxis[i];
	}
	m_axis[m_azis_00] = -m_axis[m_azis_00] * 2.0f;
	m_axis[m_azis_01] = (1.0f - m_axis[m_azis_01]) * 0.5f;
	m_axis[m_azis_02] = (1.0f - m_axis[m_azis_02]) * 0.5f;
}

void ndGameControllerInputs::Update(ndDemoEntityManager* const scene)
{
	if (scene->JoystickDetected())
	{
		if (!GetKeyboardInputs(scene))
		{
			char joystickName[256];
			strcpy(&joystickName[0], glfwGetJoystickName(0));
			strtolwr(joystickName);
			if (strstr(joystickName, "wheel"))
			{
				GetWheelJoystickInputs(scene);
			}
			else if (strstr(joystickName, "xbox"))
			{
				GetXboxJoystickInputs(scene);
			}
			else
			{
				GetJoystickInputs(scene);
			}
		}
	}
	else
	{
		GetKeyboardInputs(scene);
	}
}
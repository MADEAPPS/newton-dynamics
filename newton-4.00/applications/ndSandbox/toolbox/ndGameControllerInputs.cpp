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

void ndGameControllerInputs::Update(ndDemoEntityManager* const scene)
{
	if (scene->JoystickDetected())
	{
		char joystickName[256];
		strcpy(&joystickName[0], glfwGetJoystickName(0));
		_strlwr(joystickName);
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
	else
	{
		GetKeyboardInputs(scene);
	}
}

void ndGameControllerInputs::GetKeyboardInputs(ndDemoEntityManager* const scene)
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
	
	m_axis.SetCount(m_axisCount);
	ndFloat32 steerAngle = ndFloat32(scene->GetKeyState('A')) - ndFloat32(scene->GetKeyState('D'));
	m_keyBoardSteerAngle += (steerAngle - m_keyBoardSteerAngle) * 0.10f;
	m_axis[m_azix_00] = m_keyBoardSteerAngle;
	m_axis[m_azix_01] = ndFloat32(scene->GetKeyState('W')) ? 1.0f : 0.0f;
	m_axis[m_azix_02] = ndFloat32(scene->GetKeyState('S') ? 1.0f : 0.0f);
	m_axis[m_azix_03] = ndFloat32(0.0f);
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
			buttonMapping[1] = m_button_00;
			buttonMapping[4] = m_button_01;
			buttonMapping[5] = m_button_02;
			buttonMapping[6] = m_button_03;
			buttonMapping[7] = m_button_04;
			buttonMapping[8] = m_button_05;
			buttonMapping[9] = m_button_06;
			buttonMapping[10] = m_button_07;
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
		axisMapping[0] = m_azix_00;
		axisMapping[1] = m_azix_01;
		axisMapping[2] = m_azix_02;
	}

	m_axis.SetCount(m_axisCount);
	for (ndInt32 i = 0; i < axisMapping.GetCount(); i++)
	{
		m_axis[axisMapping[i]] = unmappedAxis[i];
	}
	m_axis[m_azix_00] = -m_axis[m_azix_00];
	m_axis[m_azix_01] = (1.0f - m_axis[m_azix_01]) * 0.5f;
	m_axis[m_azix_02] = (1.0f - m_axis[m_azix_02]) * 0.5f;
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

			buttonMapping[4] = m_button_02;  //m_upGearButton
			buttonMapping[5] = m_button_01;  //m_downGearButton
			buttonMapping[6] = m_button_06;  //m_automaticGearBoxButton
			buttonMapping[7] = m_button_04;  //m_ignitionButton
			buttonMapping[11] = m_button_00; //m_handBreakButton
			buttonMapping[13] = m_button_05; //m_reverseGearButton
			buttonMapping[10] = m_button_07; //m_parkingButton
			buttonMapping[12] = m_button_03; //m_neutralGearButton
		}
	}

	m_buttons.SetCount(m_buttonCount);
	for (ndInt32 i = 0; i < unmappedButtons.GetCount(); ++i)
	{
		m_buttons[buttonMapping[i]] = unmappedButtons[i];
		if (unmappedButtons[i])
			ndTrace(("button %d\n", i));
	}

	if (!axisMapping.GetCount())
	{
		for (ndInt32 i = 0; i < axisMapping.GetCapacity(); ++i)
		{
			axisMapping.PushBack(m_axisCount);
		}
		axisMapping[0] = m_azix_00;
		axisMapping[5] = m_azix_01;
		axisMapping[4] = m_azix_02;
	}

	m_axis.SetCount(m_axisCount);
	for (ndInt32 i = 0; i < axisMapping.GetCount(); i++)
	{
		m_axis[axisMapping[i]] = unmappedAxis[i];
	}
	m_axis[m_azix_00] = -m_axis[m_azix_00] * m_axis[m_azix_00] * m_axis[m_azix_00];

	ndFloat32 gas = (m_axis[m_azix_01] + ndFloat32(1.0f)) * ndFloat32(0.5f);
	m_axis[m_azix_01] = gas * gas;

	ndFloat32 brake = (m_axis[m_azix_02] + ndFloat32(1.0f)) * ndFloat32(0.5f);
	m_axis[m_azix_02] = brake * brake;
}

void ndGameControllerInputs::GetJoystickInputs(ndDemoEntityManager* const scene)
{
	ndAssert(0);
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

			buttonMapping[4] = m_button_02;  //m_upGearButton
			buttonMapping[5] = m_button_01;  //m_downGearButton
			buttonMapping[6] = m_button_06;  //m_automaticGearBoxButton
			buttonMapping[7] = m_button_04;  //m_ignitionButton
			buttonMapping[11] = m_button_00; //m_handBreakButton
			buttonMapping[13] = m_button_05; //m_reverseGearButton
			buttonMapping[10] = m_button_07; //m_parkingButton
			buttonMapping[12] = m_button_03; //m_neutralGearButton
		}
	}

	m_buttons.SetCount(m_buttonCount);
	for (ndInt32 i = 0; i < unmappedButtons.GetCount(); ++i)
	{
		m_buttons[buttonMapping[i]] = unmappedButtons[i];
		if (unmappedButtons[i])
			ndTrace(("button %d\n", i));
	}

	if (!axisMapping.GetCount())
	{
		for (ndInt32 i = 0; i < axisMapping.GetCapacity(); ++i)
		{
			axisMapping.PushBack(m_axisCount);
		}
		axisMapping[0] = m_azix_00;
		axisMapping[5] = m_azix_01;
		axisMapping[4] = m_azix_02;
	}

	m_axis.SetCount(m_axisCount);
	for (ndInt32 i = 0; i < axisMapping.GetCount(); i++)
	{
		m_axis[axisMapping[i]] = unmappedAxis[i];
		if ((ndAbs(unmappedAxis[i]) > 0.1) && (ndAbs(unmappedAxis[i]) < 0.99))
		{
			ndTrace(("%d %f\n", i, unmappedAxis[i]));
		}
	}
	m_axis[m_azix_00] = -m_axis[m_azix_00] * m_axis[m_azix_00] * m_axis[m_azix_00];

	ndFloat32 gas = (m_axis[m_azix_01] + ndFloat32(1.0f)) * ndFloat32(0.5f);
	m_axis[m_azix_01] = gas * gas;

	ndFloat32 brake = (m_axis[m_azix_02] + ndFloat32(1.0f)) * ndFloat32(0.5f);
	m_axis[m_azix_02] = brake * brake;
}


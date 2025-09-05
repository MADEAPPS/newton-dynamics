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
	if (scene->JoystickDetected() && !scene->AnyKeyDown())
	{
		return false;
	}
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

void ndGameControllerInputs::Update(ndDemoEntityManager* const scene)
{
	if (scene->JoystickDetected())
	{
#if !defined (__APPLE__)
		ndAssert(0);
		//char joystickName[256];
		//strcpy(&joystickName[0], glfwGetJoystickName(0));
		//strtolwr(joystickName);
		//if (strstr(joystickName, "wheel"))
		//{
		//	GetWheelJoystickInputs(scene);
		//}
		//else if (strstr(joystickName, "xbox"))
		//{
		//	GetXboxJoystickInputs(scene);
		//}
		//else
		//{
		//	GetJoystickInputs(scene);
		//}
#endif
	}
	GetKeyboardInputs(scene);
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

void ndGameControllerInputs::GetWheelJoystickInputs(ndDemoEntityManager* const scene)
{
	// logitech g920 mapping
	ndFixSizeArray<char, 32> unmappedButtons;
	ndFixSizeArray<ndFloat32, 8> unmappedAxis;
	static ndFixSizeArray<int, 8> axisMapping;
	static ndFixSizeArray<int, 32> buttonMapping;
	
	scene->GetJoystickAxis(unmappedAxis);
	scene->GetJoystickButtons(unmappedButtons);
	
	if (!buttonMapping.GetCount())
	{
		for (ndInt32 i = 0; i < unmappedButtons.GetCount(); ++i)
		{
			buttonMapping.PushBack(m_buttonCount);
		}

		buttonMapping[2] = m_button_00;		//m_ignitionButton
		buttonMapping[4] = m_button_01;		//m_upGearButton
		buttonMapping[5] = m_button_02;		//m_downGearButton
		buttonMapping[0] = m_button_03;		//m_handBreakButton
		buttonMapping[1] = m_button_03;		//m_handBreakButton
		buttonMapping[6] = m_button_04;		//m_neutralGearButton
		buttonMapping[7] = m_button_08;		//m_playerButton
		buttonMapping[8] = m_button_05;		//m_reverseGearButton
		buttonMapping[18] = m_button_01;	//m_upGearButton paddle on streeng wheel
		buttonMapping[20] = m_button_02;	//m_downGearButton paddle on streeng wheel
	}

	m_buttons.SetCount(m_buttonCount+1);
	ndMemSet(&m_buttons[0], char(0), m_buttons.GetCount());
	for (ndInt32 i = 0; i < unmappedButtons.GetCount(); ++i)
	{
		if (unmappedButtons[i])
		{
			ndInt32 buttonIndex = buttonMapping[i];
			m_buttons[buttonIndex] = unmappedButtons[i];
			//ndTrace(("(%d %d)\n", buttonIndex, unmappedButtons[i]));
		}
	}

	if (!axisMapping.GetCount())
	{
		for (ndInt32 i = 0; i < unmappedAxis.GetCount(); ++i)
		{
			axisMapping.PushBack(m_axisCount);
		}
		axisMapping[0] = m_azis_00;
		axisMapping[1] = m_azis_01;
		axisMapping[2] = m_azis_02;
		axisMapping[3] = m_azis_03;
	}
	
	m_axis.SetCount(m_axisCount + 1);
	ndMemSet(&m_axis[0], ndFloat32(0.0f), m_axis.GetCount());
	for (ndInt32 i = 0; i < unmappedAxis.GetCount(); i++)
	{
		ndInt32 axisIndex = axisMapping[i];
		m_axis[axisIndex] = unmappedAxis[i];

		//if (((i == 0) && (ndAbs(unmappedAxis[0]) > 0.1f)) || ((i > 0) && ndAbs(unmappedAxis[i]) < 0.9f))
		//ndTrace(("(%d %f)\n", axisIndex, unmappedAxis[i]));
	}
	m_axis[m_azis_00] = -m_axis[m_azis_00] * 2.0f; 
	m_axis[m_azis_01] = (1.0f - m_axis[m_azis_01]) * 0.5f;
	m_axis[m_azis_02] = ndFloat32 (1.0f) - ndClamp(m_axis[m_azis_02], ndFloat32(0.0f), ndFloat32(1.0f));
	m_axis[m_azis_03] = ndFloat32(0.0f);
}

void ndGameControllerInputs::GetXboxJoystickInputs(ndDemoEntityManager* const scene)
{
	ndFixSizeArray<char, 32> unmappedButtons;
	ndFixSizeArray<ndFloat32, 8> unmappedAxis;
	static ndFixSizeArray<int, 8> axisMapping;
	static ndFixSizeArray<int, 32> buttonMapping;

	scene->GetJoystickAxis(unmappedAxis);
	scene->GetJoystickButtons(unmappedButtons);

	if (!buttonMapping.GetCount())
	{
		for (ndInt32 i = 0; i < unmappedButtons.GetCount(); ++i)
		{
			buttonMapping.PushBack(m_buttonCount);
		}

		buttonMapping[2] = m_button_00;		//m_ignitionButton
		//buttonMapping[0] = m_button_03;		//m_handBreakButton
		//buttonMapping[1] = m_button_03;		//m_handBreakButton
		buttonMapping[4] = m_button_03;		//m_handBreakButton
		buttonMapping[5] = m_button_03;		//m_handBreakButton
		buttonMapping[7] = m_button_04;		//m_neutralGearButton
		buttonMapping[6] = m_button_08;		//m_playerButton
		buttonMapping[0] = m_button_05;		//m_reverseGearButton
		buttonMapping[1] = m_button_05;		//m_reverseGearButton
		buttonMapping[10] = m_button_01;	//m_upGearButton
		buttonMapping[12] = m_button_02;	//m_downGearButton
	}

	m_buttons.SetCount(m_buttonCount + 1);
	ndMemSet(&m_buttons[0], char(0), m_buttons.GetCount());
	for (ndInt32 i = 0; i < unmappedButtons.GetCount(); ++i)
	{
		if (unmappedButtons[i])
		{
			ndInt32 buttonIndex = buttonMapping[i];
			m_buttons[buttonIndex] = unmappedButtons[i];
			//ndTrace(("(%d %d)\n", buttonIndex, unmappedButtons[i]));
		}
	}

	if (!axisMapping.GetCount())
	{
		for (ndInt32 i = 0; i < unmappedAxis.GetCount(); ++i)
		{
			axisMapping.PushBack(m_axisCount);
		}
		axisMapping[2] = m_azis_00;
		axisMapping[4] = m_azis_02;
		axisMapping[5] = m_azis_01;
	}

	m_axis.SetCount(m_axisCount + 1);
	ndMemSet(&m_axis[0], ndFloat32(0.0f), m_axis.GetCount());
	for (ndInt32 i = 0; i < unmappedAxis.GetCount(); i++)
	{
		ndInt32 axisIndex = axisMapping[i];
		m_axis[axisIndex] = unmappedAxis[i];
	}
	
	//ndTrace(("%f %f %f %f %f %f\n", unmappedAxis[0], unmappedAxis[1], unmappedAxis[2], unmappedAxis[3], unmappedAxis[4], unmappedAxis[5]));

	ndFloat32 steer = -m_axis[m_azis_00] * m_axis[m_azis_00] * m_axis[m_azis_00];
	ndFloat32 gas = (m_axis[m_azis_01] + ndFloat32(1.0f)) * ndFloat32(0.5f);
	ndFloat32 brake = (ndFloat32(1.0f) + m_axis[m_azis_02]) * ndFloat32(0.5f);

	//ndTrace(("%f %f\n", brake, unmappedAxis[5]));
	//ndTrace(("gass:%f brake:%f)\n", gas, brake));
	m_axis[m_azis_00] = steer;
	m_axis[m_azis_01] = gas * gas;
	m_axis[m_azis_02] = brake * brake;
	m_axis[m_azis_03] = ndFloat32(0.0f);
}
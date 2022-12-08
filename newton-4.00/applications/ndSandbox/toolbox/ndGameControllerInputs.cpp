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
			ndAssert(0);
			//GetXboxJoystickInputs(scene, buttons, axis);
		}
		else
		{
			ndAssert(0);
			//GetJoystickInputs(scene, buttons, axis);
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
	m_buttons[m_handBreakButton] = scene->GetKeyState(' ');
	m_buttons[m_upGearButton] = scene->GetKeyState('>') || scene->GetKeyState('.');
	m_buttons[m_downGearButton] = scene->GetKeyState('<') || scene->GetKeyState(',');
	m_buttons[m_neutralGearButton] = scene->GetKeyState('N');
	m_buttons[m_ignitionButton] = scene->GetKeyState('I');
	m_buttons[m_reverseGearButton] = scene->GetKeyState('R');
	m_buttons[m_automaticGearBoxButton] = scene->GetKeyState('?') || scene->GetKeyState('/');
	m_buttons[m_parkingButton] = scene->GetKeyState('P');
	
	m_axis.SetCount(m_axisCount);
	ndFloat32 steerAngle = ndFloat32(scene->GetKeyState('A')) - ndFloat32(scene->GetKeyState('D'));
	m_keyBoardSteerAngle += (steerAngle - m_keyBoardSteerAngle) * 0.10f;
	m_axis[m_steeringWheel] = m_keyBoardSteerAngle;
	m_axis[m_gasPedal] = ndFloat32(scene->GetKeyState('W')) ? 1.0f : 0.0f;
	m_axis[m_brakePedal] = ndFloat32(scene->GetKeyState('S') ? 1.0f : 0.0f);
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
			//buttons[0] = 0;
			//buttons[1] = scene->GetKeyState(' ');
			//buttons[2] = 0;
			//buttons[3] = 0;
			//buttons[4] = scene->GetKeyState('>') || scene->GetKeyState('.');
			//buttons[5] = scene->GetKeyState('<') || scene->GetKeyState(',');
			//buttons[6] = scene->GetKeyState('N');
			//buttons[7] = scene->GetKeyState('I');
			//buttons[8] = scene->GetKeyState('R');
			//buttons[9] = scene->GetKeyState('?') || scene->GetKeyState('/');
			//buttons[10] = scene->GetKeyState('P');
			buttonMapping[1] = m_handBreakButton;
			buttonMapping[4] = m_upGearButton;
			buttonMapping[5] = m_downGearButton;
			buttonMapping[6] = m_neutralGearButton;
			buttonMapping[7] = m_ignitionButton;
			buttonMapping[8] = m_reverseGearButton;
			buttonMapping[9] = m_automaticGearBoxButton;
			buttonMapping[10] = m_parkingButton;
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
		axisMapping[0] = m_steeringWheel;
		axisMapping[1] = m_gasPedal;
		axisMapping[2] = m_brakePedal;
	}

	m_axis.SetCount(m_axisCount);
	for (ndInt32 i = 0; i < axisMapping.GetCount(); i++)
	{
		m_axis[axisMapping[i]] = unmappedAxis[i];
	}
	m_axis[m_steeringWheel] = -m_axis[m_steeringWheel];
	m_axis[m_gasPedal] = (1.0f - m_axis[m_gasPedal]) * 0.5f;
	m_axis[m_brakePedal] = (1.0f - m_axis[m_brakePedal]) * 0.5f;
}

/*
void ndVehicleCommon::GetXboxJoystickInputs(ndDemoEntityManager* const scene, ndFixSizeArray<char, 32>& buttons, ndFixSizeArray<ndFloat32, 8>& axis) const
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
			//buttons[1] = scene->GetKeyState(' ');
			//buttons[4] = scene->GetKeyState('>') || scene->GetKeyState('.');
			//buttons[5] = scene->GetKeyState('<') || scene->GetKeyState(',');
			//buttons[6] = scene->GetKeyState('N');
			//buttons[7] = scene->GetKeyState('I');
			//buttons[8] = scene->GetKeyState('R');
			//buttons[9] = scene->GetKeyState('?') || scene->GetKeyState('/');
			//buttons[10] = scene->GetKeyState('P');

			buttonMapping[4] = m_automaticGearBoxButton;
			buttonMapping[5] = m_reverseGearButton;
			buttonMapping[6] = m_ignitionButton;
			buttonMapping[7] = m_neutralGearButton;
			buttonMapping[10] = m_parkingButton;
			buttonMapping[12] = m_handBreakButton;
			buttonMapping[11] = m_upGearButton;
			buttonMapping[13] = m_downGearButton;
		}
	}

	buttons.SetCount(m_buttonCount);
	for (ndInt32 i = 0; i < unmappedButtons.GetCount(); ++i)
	{
		buttons[buttonMapping[i]] = unmappedButtons[i];
	}

	if (!axisMapping.GetCount())
	{
		for (ndInt32 i = 0; i < axisMapping.GetCapacity(); ++i)
		{
			axisMapping.PushBack(m_axisCount);
		}
		axisMapping[2] = m_steeringWheel;
		//axisMapping[1] = m_gasPedal;
		//axisMapping[2] = m_brakePedal;
	}

	axis.SetCount(m_axisCount);
	for (ndInt32 i = 0; i < axisMapping.GetCount(); i++)
	{
		axis[axisMapping[i]] = unmappedAxis[i];

		if ((ndAbs(unmappedAxis[i]) > 0.1) && (ndAbs(unmappedAxis[i]) < 0.99))
		{
			ndAssert(0);
			ndTrace(("%d %f\n", i, unmappedAxis[i]));
		}
		//axis[axisMapping[i]] = 0;
	}
	axis[m_steeringWheel] = -axis[m_steeringWheel] * axis[m_steeringWheel] * axis[m_steeringWheel];
	//axis[m_gasPedal] = (1.0f - axis[m_gasPedal]) * 0.5f;
	//axis[m_brakePedal] = (1.0f - axis[m_brakePedal]) * 0.5f;
}

void ndVehicleCommon::GetJoystickInputs(ndDemoEntityManager* const scene, ndFixSizeArray<char, 32>& buttons, ndFixSizeArray<ndFloat32, 8>& axis) const
{
	ndFixSizeArray<char, 32> unmappedButtons;
	ndFixSizeArray<ndFloat32, 8> unmappedAxis;
	
	scene->GetJoystickAxis(unmappedAxis);
	scene->GetJoystickButtons(unmappedButtons);

	char joystickName[256];
	strcpy(&joystickName[0], glfwGetJoystickName(0));
	_strlwr(joystickName);
	if (strstr(joystickName, "wheel"))
	{ 
		GetWheelJoystickInputs(scene, buttons, axis);
		// logitech g920 mapping
		static ndFixSizeArray<int, 8> axisMapping;
		static ndFixSizeArray<int, 32> buttonMapping;

		if (!buttonMapping.GetCount())
		{
			if (!buttonMapping.GetCount())
			{
				for (ndInt32 i = 0; i < buttonMapping.GetCapacity(); ++i)
				{
					buttonMapping.PushBack(m_buttonCount);
				}
				//buttons[0] = 0;
				//buttons[1] = scene->GetKeyState(' ');
				//buttons[2] = 0;
				//buttons[3] = 0;
				//buttons[4] = scene->GetKeyState('>') || scene->GetKeyState('.');
				//buttons[5] = scene->GetKeyState('<') || scene->GetKeyState(',');
				//buttons[6] = scene->GetKeyState('N');
				//buttons[7] = scene->GetKeyState('I');
				//buttons[8] = scene->GetKeyState('R');
				//buttons[9] = scene->GetKeyState('?') || scene->GetKeyState('/');
				//buttons[10] = scene->GetKeyState('P');
				buttonMapping[1] = m_handBreakButton;
				buttonMapping[4] = m_upGearButton;
				buttonMapping[5] = m_downGearButton;
				buttonMapping[6] = m_neutralGearButton;
				buttonMapping[7] = m_ignitionButton;
				buttonMapping[8] = m_reverseGearButton;
				buttonMapping[9] = m_automaticGearBoxButton;
				buttonMapping[10] = m_parkingButton;
			}
		}
		
		buttons.SetCount(m_buttonCount);
		for (ndInt32 i = 0; i < unmappedButtons.GetCount(); ++i)
		{
			buttons[buttonMapping[i]] = unmappedButtons[i];
		}

		if (!axisMapping.GetCount())
		{
			for (ndInt32 i = 0; i < axisMapping.GetCapacity(); ++i)
			{
				axisMapping.PushBack(m_axisCount);
			}
			axisMapping[0] = m_steeringWheel;
			axisMapping[1] = m_gasPedal;
			axisMapping[2] = m_brakePedal;
		}

		axis.SetCount(m_axisCount);
		for (ndInt32 i = 0; i < axisMapping.GetCount(); i++)
		{
			axis[axisMapping[i]] = unmappedAxis[i];
		}
		axis[m_steeringWheel] = -axis[m_steeringWheel];
		axis[m_gasPedal] = (1.0f - axis[m_gasPedal]) * 0.5f;
		axis[m_brakePedal] = (1.0f - axis[m_brakePedal]) * 0.5f;
	}
	else if (strstr(joystickName, "xbox"))
	{
		static ndFixSizeArray<int, 8> axisMapping;
		static ndFixSizeArray<int, 32> buttonMapping;

		if (!buttonMapping.GetCount())
		{
			if (!buttonMapping.GetCount())
			{
				for (ndInt32 i = 0; i < buttonMapping.GetCapacity(); ++i)
				{
					buttonMapping.PushBack(m_buttonCount);
				}
				//buttons[0] = 0;
				//buttons[1] = scene->GetKeyState(' ');
				//buttons[2] = 0;
				//buttons[3] = 0;
				//buttons[4] = scene->GetKeyState('>') || scene->GetKeyState('.');
				//buttons[5] = scene->GetKeyState('<') || scene->GetKeyState(',');
				//buttons[6] = scene->GetKeyState('N');
				//buttons[7] = scene->GetKeyState('I');
				//buttons[8] = scene->GetKeyState('R');
				//buttons[9] = scene->GetKeyState('?') || scene->GetKeyState('/');
				//buttons[10] = scene->GetKeyState('P');
				buttonMapping[1] = m_handBreakButton;
				buttonMapping[4] = m_upGearButton;
				buttonMapping[5] = m_downGearButton;
				buttonMapping[6] = m_neutralGearButton;
				buttonMapping[7] = m_ignitionButton;
				buttonMapping[8] = m_reverseGearButton;
				buttonMapping[9] = m_automaticGearBoxButton;
				buttonMapping[10] = m_parkingButton;
			}
		}

		buttons.SetCount(m_buttonCount);
		for (ndInt32 i = 0; i < unmappedButtons.GetCount(); ++i)
		{
			buttons[buttonMapping[i]] = unmappedButtons[i];
		}

		if (!axisMapping.GetCount())
		{
			for (ndInt32 i = 0; i < axisMapping.GetCapacity(); ++i)
			{
				axisMapping.PushBack(m_axisCount);
			}
			axisMapping[0] = m_steeringWheel;
			axisMapping[1] = m_gasPedal;
			axisMapping[2] = m_brakePedal;
		}

		axis.SetCount(m_axisCount);
		for (ndInt32 i = 0; i < axisMapping.GetCount(); i++)
		{
			axis[axisMapping[i]] = unmappedAxis[i];
		}

		axis[m_steeringWheel] = axis[m_steeringWheel] * axis[m_steeringWheel] * axis[m_steeringWheel];
		////axis[1] = (1.0f - axis[1]) * 0.5f;
		////axis[2] = (1.0f - axis[2]) * 0.5f;
	}
	else
	{
		ndAssert(0);

		static ndFixSizeArray<int, 8> axisMapping;
		static ndFixSizeArray<int, 32> buttonMapping;

		if (!buttonMapping.GetCount())
		{
			if (!buttonMapping.GetCount())
			{
				for (ndInt32 i = 0; i < buttonMapping.GetCapacity(); ++i)
				{
					buttonMapping.PushBack(m_buttonCount);
				}
				//buttons[0] = 0;
				//buttons[1] = scene->GetKeyState(' ');
				//buttons[2] = 0;
				//buttons[3] = 0;
				//buttons[4] = scene->GetKeyState('>') || scene->GetKeyState('.');
				//buttons[5] = scene->GetKeyState('<') || scene->GetKeyState(',');
				//buttons[6] = scene->GetKeyState('N');
				//buttons[7] = scene->GetKeyState('I');
				//buttons[8] = scene->GetKeyState('R');
				//buttons[9] = scene->GetKeyState('?') || scene->GetKeyState('/');
				//buttons[10] = scene->GetKeyState('P');
				buttonMapping[1] = m_handBreakButton;
				buttonMapping[4] = m_upGearButton;
				buttonMapping[5] = m_downGearButton;
				buttonMapping[6] = m_neutralGearButton;
				buttonMapping[7] = m_ignitionButton;
				buttonMapping[8] = m_reverseGearButton;
				buttonMapping[9] = m_automaticGearBoxButton;
				buttonMapping[10] = m_parkingButton;
			}
		}

		buttons.SetCount(m_buttonCount);
		for (ndInt32 i = 0; i < unmappedButtons.GetCount(); ++i)
		{
			buttons[buttonMapping[i]] = unmappedButtons[i];
		}

		if (!axisMapping.GetCount())
		{
			for (ndInt32 i = 0; i < axisMapping.GetCapacity(); ++i)
			{
				axisMapping.PushBack(m_axisCount);
			}
			axisMapping[0] = m_steeringWheel;
			axisMapping[1] = m_gasPedal;
			axisMapping[2] = m_brakePedal;
		}

		axis.SetCount(m_axisCount);
		for (ndInt32 i = 0; i < axisMapping.GetCount(); i++)
		{
			axis[axisMapping[i]] = unmappedAxis[i];
		}

		axis[m_steeringWheel] = axis[m_steeringWheel] * axis[m_steeringWheel] * axis[m_steeringWheel];
		////axis[1] = (1.0f - axis[1]) * 0.5f;
		////axis[2] = (1.0f - axis[2]) * 0.5f;
	}

	//for (ndInt32 i = 0; i < buttons.GetCount(); i++)
	//{
	//	if (buttons[i])
	//	{
	//		ndTrace(("button_%d: %d\n", i, buttons[i]));
	//	}
	//}
	
	//for (ndInt32 i = 0; i < axis.GetCount(); i++)
	//{
	//	if ((ndAbs (axis[i]) > 0.01f) && (ndAbs(axis[i]) < 0.99f))
	//	{
	//		ndTrace(("axis_%d: %f\n", i, axis[i]));
	//	}
	//}
}
*/

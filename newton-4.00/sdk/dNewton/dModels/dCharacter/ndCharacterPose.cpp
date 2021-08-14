/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndCharacterPose.h"

ndCharaterKeyFramePose::ndCharaterKeyFramePose()
	:m_x(dFloat32 (0.0f))
	,m_y(dFloat32(0.0f))
	,m_z(dFloat32(0.0f))
	,m_pitch(dFloat32(0.0f))
	,m_yaw(dFloat32(0.0f))
	,m_roll(dFloat32(0.0f))
	,m_node(nullptr)
{
}

ndCharaterKeyFramePose::ndCharaterKeyFramePose(ndCharacterLimbNode* const node, const dMatrix& matrix)
	:m_x(matrix.m_posit.m_x)
	,m_y(matrix.m_posit.m_y)
	,m_z(matrix.m_posit.m_z)
	,m_node(node)
{
	dVector euler0;
	dVector euler1;
	matrix.CalcPitchYawRoll(euler0, euler1);
	m_pitch = euler0.m_x;
	m_yaw = euler0.m_y;
	m_roll = euler0.m_z;
}

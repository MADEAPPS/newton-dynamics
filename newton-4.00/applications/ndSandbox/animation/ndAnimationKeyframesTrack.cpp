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

#include "ndSandboxStdafx.h"
#include "ndAnimationKeyframesTrack.h"

const void ndAnimationKeyFramesTrack::InterpolatePosition(dFloat32 t, dVector& posit) const
{
	if (m_position.GetCount()) 
	{
		int base = m_position.GetIndex(t);
		const dFloat32 t0 = m_position.m_time[base];
		const dFloat32 t1 = m_position.m_time[base + 1];
		const dFloat32 param = (t - t0) / (t1 - t0 + dFloat32(1.0e-6f));
		const dVector& p0 = m_position[base];
		const dVector& p1 = m_position[base + 1];
		posit = p0 + (p1 - p0).Scale(param);
	}
}

const void ndAnimationKeyFramesTrack::InterpolateRotation(dFloat32 t, dQuaternion& rotation) const
{
	if (m_rotation.GetCount()) 
	{
		int base = m_rotation.GetIndex(t);
		const dFloat32 t0 = m_rotation.m_time[base];
		const dFloat32 t1 = m_rotation.m_time[base + 1];
		const dFloat32 param = (t - t0) / (t1 - t0 + dFloat32 (1.0e-6f));
		const dQuaternion& rot0 = m_rotation[base];
		const dQuaternion& rot1 = m_rotation[base + 1];
		rotation = rot0.Slerp(rot1, param);
	}
}

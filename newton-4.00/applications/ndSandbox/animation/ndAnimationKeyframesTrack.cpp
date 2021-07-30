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

template<class OBJECT>
dInt32 ndAnimationKeyFramesTrack::dKeyFramesArray<OBJECT>::GetIndex(dFloat32 time) const
{
	dAssert(time >= 0.0f);
	dInt32 index = dArray<OBJECT>::GetCount() - 1;
	if (time < m_time[index])
	{
		dInt32 i0 = 0;
		dInt32 i1 = index;
		while ((i1 - i0) > 4)
		{
			const dInt32 mid = (i1 + i0) / 2;
			if (time < m_time[mid])
			{
				i1 = mid;
			}
			else
			{
				i0 = mid;
			}
		}
		dAssert(m_time[i0] <= time);
		for (i0 = i0 + 1; i0 <= index; i0++)
		{
			if (m_time[i0] >= time)
			{
				return i0 - 1;
			}
		}
		index = dMin (index, i0);
	}
	return index;
}

void ndAnimationKeyFramesTrack::InterpolatePosition(dFloat32 time, dFloat32 length, dVector& posit) const
{
	//if (m_name != "mixamorig:Hips")
	//{
	//	posit = m_position[0];
	//	return;
	//}
	//posit = m_position[0];
	//return;

	if (m_position.GetCount()) 
	{
		dInt32 base = m_position.GetIndex(time);
		if (base < (m_position.GetCount() - 1))
		{
			const dFloat32 t0 = m_position.m_time[base + 0];
			const dFloat32 t1 = m_position.m_time[base + 1];
			const dFloat32 param = (time - t0) / (t1 - t0 + dFloat32(1.0e-6f));
			const dVector& p0 = m_position[base + 0];
			const dVector& p1 = m_position[base + 1];
			posit = p0 + (p1 - p0).Scale(param);
		}
		else
		{
			const dFloat32 t1 = length;
			const dFloat32 t0 = m_position.m_time[base];
			const dFloat32 param = (time - t0) / (t1 - t0 + dFloat32(1.0e-6f));
			const dVector& p0 = m_position[base];
			const dVector& p1 = m_position[0];
			posit = p0 + (p1 - p0).Scale(param);
		}

		//static dVector xxxx(posit);
		//dVector err(posit - xxxx);
		//xxxx = posit;
		//dTrace(("%f (%f %f %f)\n", time, posit.m_x, posit.m_y, posit.m_z));
		//dTrace(("%f (%f %f %f)\n", time, err.m_x, err.m_y, err.m_z));
	}
}

void ndAnimationKeyFramesTrack::InterpolateRotation(dFloat32 time, dFloat32 length, dQuaternion& rotation) const
{
	//if (m_name != "mixamorig:LeftLeg")
	//{
	//	rotation = m_rotation[0];
	//	return;
	//}
	
	if (m_rotation.GetCount()) 
	{
		//time = 0.0f;
		dInt32 base = m_rotation.GetIndex(time);
		if (base < m_rotation.GetCount() - 1)
		{
			const dFloat32 t0 = m_rotation.m_time[base + 0];
			const dFloat32 t1 = m_rotation.m_time[base + 1];
			const dFloat32 param = (time - t0) / (t1 - t0 + dFloat32(1.0e-6f));
			const dQuaternion& rot0 = m_rotation[base + 0];
			const dQuaternion& rot1 = m_rotation[base + 1];
			dAssert(rot0.DotProduct(rot1).GetScalar() > 0.0f);
			rotation = rot0.Slerp(rot1, param);
		}
		else
		{
			const dFloat32 t1 = length;
			const dFloat32 t0 = m_rotation.m_time[base];
			const dFloat32 param = (time - t0) / (t1 - t0 + dFloat32(1.0e-6f));
			const dQuaternion& rot0 = m_rotation[base];
			const dQuaternion& rot1 = m_rotation[0];
			dAssert(rot0.DotProduct(rot1).GetScalar() > 0.0f);
			rotation = rot0.Slerp(rot1, param);
		}
		//dTrace(("%d %f (%f %f %f %f)\n", base, time, rotation.m_x, rotation.m_y, rotation.m_z, rotation.m_w));
	}
}

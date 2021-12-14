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
ndInt32 ndAnimationKeyFramesTrack::dKeyFramesArray<OBJECT>::GetIndexDebug(ndFloat32 time) const
{
	ndInt32 index = 0;
	for (ndInt32 i = ndArray<OBJECT>::GetCount()-1; i >= 0; i --)
	{
		if (m_time[i] < time)
		{
			index = i;
			break;
		}
	}
	return index;
}

template<class OBJECT>
ndInt32 ndAnimationKeyFramesTrack::dKeyFramesArray<OBJECT>::GetIndex(ndFloat32 time) const
{
	dAssert(time >= 0.0f);
	ndInt32 index = ndArray<OBJECT>::GetCount() - 1;
	if (time < m_time[index])
	{
		ndInt32 i0 = 0;
		ndInt32 i1 = index;
		while ((i1 - i0) > 4)
		{
			const ndInt32 mid = (i1 + i0) / 2;
			if (m_time[mid] < time)
			{
				i0 = mid;
			}
			else
			{
				i1 = mid;
			}
		}
		dAssert(m_time[i1] >= time);
		index = 0;
		for (ndInt32 i = i1; i >= 0; i--)
		{
			if (m_time[i] < time)
			{
				index = i;
				break;
			}
		}
	}

	dAssert(index == GetIndexDebug(time));
	return index;
}

void ndAnimationKeyFramesTrack::InterpolatePosition(ndFloat32 time, ndFloat32 length, ndVector& posit) const
{
	if (m_position.GetCount()) 
	{
		const ndInt32 base = m_position.GetIndex(time);
		if (base < (m_position.GetCount() - 1))
		{
			const ndFloat32 t0 = m_position.m_time[base + 0];
			const ndFloat32 t1 = m_position.m_time[base + 1];
			const ndVector& p0 = m_position[base + 0];
			const ndVector& p1 = m_position[base + 1];
			const ndFloat32 param = (time - t0) / (t1 - t0 + ndFloat32(1.0e-6f));
			posit = p0 + (p1 - p0).Scale(param);
		}
		else
		{
			const ndFloat32 t1 = length;
			const ndFloat32 t0 = m_position.m_time[base];
			const ndVector& p1 = m_position[0];
			const ndVector& p0 = m_position[base];
			const ndFloat32 param = (time - t0) / (t1 - t0 + ndFloat32(1.0e-6f));
			posit = p0 + (p1 - p0).Scale(param);
		}
	}
}

void ndAnimationKeyFramesTrack::InterpolateRotation(ndFloat32 time, ndFloat32 length, ndQuaternion& rotation) const
{
	if (m_rotation.GetCount()) 
	{
		const ndInt32 base = m_rotation.GetIndex(time);
		if (base < m_rotation.GetCount() - 1)
		{
			const ndFloat32 t0 = m_rotation.m_time[base + 0];
			const ndFloat32 t1 = m_rotation.m_time[base + 1];
			const ndQuaternion& rot0 = m_rotation[base + 0];
			const ndQuaternion& rot1 = m_rotation[base + 1];
			const ndFloat32 param = (time - t0) / (t1 - t0 + ndFloat32(1.0e-6f));
			dAssert(rot0.DotProduct(rot1).GetScalar() > 0.0f);
			rotation = rot0.Slerp(rot1, param);
		}
		else
		{
			const ndFloat32 t1 = length;
			const ndFloat32 t0 = m_rotation.m_time[base];
			const ndQuaternion& rot1 = m_rotation[0];
			const ndQuaternion& rot0 = m_rotation[base];
			const ndFloat32 param = (time - t0) / (t1 - t0 + ndFloat32(1.0e-6f));
			dAssert(rot0.DotProduct(rot1).GetScalar() > 0.0f);
			rotation = rot0.Slerp(rot1, param);
		}
	}
}

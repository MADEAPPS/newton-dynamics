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
ndInt32 ndAnimationKeyFramesTrack::dKeyFramesArray<OBJECT>::GetIndexDebug(ndFloat32 param) const
{
	ndInt32 index = 0;
	for (ndInt32 i = ndArray<OBJECT>::GetCount()-1; i >= 0; --i)
	{
		if (m_param[i] < param)
		{
			index = i;
			break;
		}
	}
	return index;
}

template<class OBJECT>
ndInt32 ndAnimationKeyFramesTrack::dKeyFramesArray<OBJECT>::GetIndex(ndFloat32 param) const
{
	dAssert(param >= 0.0f);
	ndInt32 index = ndArray<OBJECT>::GetCount() - 1;
	if (param < m_param[index])
	{
		ndInt32 i0 = 0;
		ndInt32 i1 = index;
		while ((i1 - i0) > 4)
		{
			const ndInt32 mid = (i1 + i0) / 2;
			if (m_param[mid] < param)
			{
				i0 = mid;
			}
			else
			{
				i1 = mid;
			}
		}
		dAssert(m_param[i1] >= param);
		index = 0;
		for (ndInt32 i = i1; i >= 0; --i)
		{
			if (m_param[i] < param)
			{
				index = i;
				break;
			}
		}
	}

	dAssert(index == GetIndexDebug(param));
	return index;
}

void ndAnimationKeyFramesTrack::InterpolatePosition(ndFloat32 param, ndVector& posit) const
{
	if (m_position.GetCount()) 
	{
		const ndInt32 base = m_position.GetIndex(param);
		if (base < (m_position.GetCount() - 1))
		{
			const ndFloat32 t0 = m_position.m_param[base + 0];
			const ndFloat32 t1 = m_position.m_param[base + 1];
			const ndVector& p0 = m_position[base + 0];
			const ndVector& p1 = m_position[base + 1];
			const ndFloat32 t = (param - t0) / (t1 - t0 + ndFloat32(1.0e-6f));
			posit = p0 + (p1 - p0).Scale(t);
		}
		else
		{
			dAssert(0);
			//const ndFloat32 t1 = length;
			//const ndFloat32 t0 = m_position.m_param[base];
			//const ndVector& p1 = m_position[0];
			//const ndVector& p0 = m_position[base];
			//const ndFloat32 t = (param - t0) / (t1 - t0 + ndFloat32(1.0e-6f));
			//posit = p0 + (p1 - p0).Scale(t);
		}
	}
}

void ndAnimationKeyFramesTrack::InterpolateRotation(ndFloat32 param, ndQuaternion& rotation) const
{
	if (m_rotation.GetCount()) 
	{
		const ndInt32 base = m_rotation.GetIndex(param);
		if (base < m_rotation.GetCount() - 1)
		{
			const ndFloat32 t0 = m_rotation.m_param[base + 0];
			const ndFloat32 t1 = m_rotation.m_param[base + 1];
			const ndQuaternion& rot0 = m_rotation[base + 0];
			const ndQuaternion& rot1 = m_rotation[base + 1];
			const ndFloat32 t = (param - t0) / (t1 - t0 + ndFloat32(1.0e-6f));
			dAssert(rot0.DotProduct(rot1).GetScalar() > 0.0f);
			rotation = rot0.Slerp(rot1, t);
		}
		else
		{
			dAssert(0);
			//const ndFloat32 t1 = length;
			//const ndFloat32 t0 = m_rotation.m_param[base];
			//const ndQuaternion& rot1 = m_rotation[0];
			//const ndQuaternion& rot0 = m_rotation[base];
			//const ndFloat32 t = (param - t0) / (t1 - t0 + ndFloat32(1.0e-6f));
			//dAssert(rot0.DotProduct(rot1).GetScalar() > 0.0f);
			//rotation = rot0.Slerp(rot1, t);
		}
	}
}

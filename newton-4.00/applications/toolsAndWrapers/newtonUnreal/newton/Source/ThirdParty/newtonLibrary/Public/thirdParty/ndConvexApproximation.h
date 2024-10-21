/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#pragma once
#include "ndNewton.h"

class ndHullPoint
{
	public:
	ndReal m_x;
	ndReal m_y;
	ndReal m_z;
};

class ndHullInputMesh
{
	public:
	class ndFace
	{
		public:
		ndInt32 m_i0;
		ndInt32 m_i1;
		ndInt32 m_i2;
	};
	ndArray<ndFace> m_faces;
	ndArray<ndHullPoint> m_points;
};

class ndHullOutput: public ndArray<ndHullPoint>
{
};

class ndConvexApproximation: public ndClassAlloc
{
	class ProgressBar;

	public:
	D_CORE_API ndConvexApproximation(ndInt32 maxConvexes, bool quality);
	D_CORE_API virtual ~ndConvexApproximation();

	D_CORE_API void Execute();
	D_CORE_API virtual void ShowProgress();

	ndHullInputMesh m_inputMesh;
	ndArray<ndHullOutput*> m_ouputHulls;
	ndFloat32 m_tolerance;
	ndInt32 m_maxConvexes;
	ndInt32 m_maxPointPerHull;
	
	bool m_quality;
};

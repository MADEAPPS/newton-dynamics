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

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndVector.h"
#include "ndPolyhedraMassProperties.h"

ndPolyhedraMassProperties::ndPolyhedraMassProperties()
	:ndClassAlloc()
{
	memset (this, 0, sizeof (ndPolyhedraMassProperties));
	mult[0] = ndFloat32 (1.0f/6.0f); 
	mult[1] = ndFloat32 (1.0f/24.0f);
	mult[2] = ndFloat32 (1.0f/24.0f);
	mult[3] = ndFloat32 (1.0f/24.0f);
	mult[4] = ndFloat32 (1.0f/60.0f);
	mult[5] = ndFloat32 (1.0f/60.0f);
	mult[6] = ndFloat32 (1.0f/60.0f);
	mult[7] = ndFloat32 (1.0f/120.0f);
	mult[8] = ndFloat32 (1.0f/120.0f);
	mult[9] = ndFloat32 (1.0f/120.0f);
}

void ndPolyhedraMassProperties::AddCGFace (ndInt32 indexCount, const ndVector* const faceVertex)
{
	#define CDSubexpressions(w0,w1,w2,f1,f2) \
	{					\
		ndFloat32 temp0 = w0 + w1; \
		f1 = temp0 + w2; \
		f2 = w0 * w0 + w1 * temp0 + w2 * f1; \
	}					

	ndVector p0 (faceVertex[0]);
	ndVector p1 (faceVertex[1]);

	for (ndInt32 i = 2; i < indexCount; i++) 
	{
		ndVector p2 (faceVertex[i]);

		ndVector e01 (p1 - p0);
		ndVector e02 (p2 - p0);
		ndVector d (e01.CrossProduct(e02));

		ndVector f1;
		ndVector f2;
		CDSubexpressions (p0.m_x, p1.m_x, p2.m_x, f1.m_x, f2.m_x);
		CDSubexpressions (p0.m_y, p1.m_y, p2.m_y, f1.m_y, f2.m_y);
		CDSubexpressions (p0.m_z, p1.m_z, p2.m_z, f1.m_z, f2.m_z);

		// update integrals
		intg[0] += d[0] * f1.m_x;

		intg[1] += d[0] * f2.m_x; 
		intg[2] += d[1] * f2.m_y; 
		intg[3] += d[2] * f2.m_z;

		p1 = p2;
	}
}

void ndPolyhedraMassProperties::AddInertiaFace (ndInt32 indexCount, const ndVector* const faceVertex)
{
	#define InertiaSubexpression(w0,w1,w2,f1,f2,f3) \
	{					 \
		ndFloat32 temp0 = w0 + w1; \
		ndFloat32 temp1 = w0 * w0; \
		ndFloat32 temp2 = temp1 + w1 * temp0; \
		f1 = temp0 + w2; \
		f2 = temp2 + w2 * f1;  \
		f3 = w0 * temp1 + w1 * temp2 + w2 * f2; \
	}

	ndVector p0 (faceVertex[0]);
	ndVector p1 (faceVertex[1]);

	for (ndInt32 i = 2; i < indexCount; i++) 
	{
		ndVector p2 (faceVertex[i]);

		ndVector e01 (p1 - p0);
		ndVector e02 (p2 - p0);
		ndVector d (e01.CrossProduct(e02));

		ndVector f1;
		ndVector f2;
		ndVector f3;
		InertiaSubexpression (p0.m_x, p1.m_x, p2.m_x, f1.m_x, f2.m_x, f3.m_x);
		InertiaSubexpression (p0.m_y, p1.m_y, p2.m_y, f1.m_y, f2.m_y, f3.m_y);
		InertiaSubexpression (p0.m_z, p1.m_z, p2.m_z, f1.m_z, f2.m_z, f3.m_z);

		// update integrals
		intg[0] += d[0] * f1.m_x;

		intg[1] += d[0] * f2.m_x; 
		intg[2] += d[1] * f2.m_y; 
		intg[3] += d[2] * f2.m_z;

		intg[4] += d[0] * f3.m_x; 
		intg[5] += d[1] * f3.m_y; 
		intg[6] += d[2] * f3.m_z;

		p1 = p2;
	}
}

void ndPolyhedraMassProperties::AddInertiaAndCrossFace (ndInt32 indexCount, const ndVector* const faceVertex)
{
	#define Subexpressions(w0,w1,w2,f1,f2,f3,g0,g1,g2) \
	{												   \
		ndFloat32 temp0 = w0 + w1; \
		ndFloat32 temp1 = w0 * w0; \
		ndFloat32 temp2 = temp1 + w1 * temp0; \
		f1 = temp0 + w2; \
		f2 = temp2 + w2 * f1;  \
		f3 = w0 * temp1 + w1 * temp2 + w2 * f2; \
		g0 = f2 + w0 * (f1 + w0); \
		g1 = f2 + w1 * (f1 + w1); \
		g2 = f2 + w2 * (f1 + w2); \
	}

	ndVector p0 (faceVertex[0]);
	ndVector p1 (faceVertex[1]);
	for (ndInt32 i = 2; i < indexCount; i++) 
	{
		ndVector p2 (faceVertex[i]);

		ndVector e01 (p1 - p0);
		ndVector e02 (p2 - p0);
		ndVector d (e01.CrossProduct(e02));

		ndVector f1;
		ndVector f2;
		ndVector f3;
		ndVector g0;
		ndVector g1;
		ndVector g2;
		Subexpressions (p0.m_x, p1.m_x, p2.m_x, f1.m_x, f2.m_x, f3.m_x, g0.m_x, g1.m_x, g2.m_x);
		Subexpressions (p0.m_y, p1.m_y, p2.m_y, f1.m_y, f2.m_y, f3.m_y, g0.m_y, g1.m_y, g2.m_y);
		Subexpressions (p0.m_z, p1.m_z, p2.m_z, f1.m_z, f2.m_z, f3.m_z, g0.m_z, g1.m_z, g2.m_z);

		// update integrals
		intg[0] += d[0] * f1.m_x;

		intg[1] += d[0] * f2.m_x; 
		intg[2] += d[1] * f2.m_y; 
		intg[3] += d[2] * f2.m_z;

		intg[4] += d[0] * f3.m_x; 
		intg[5] += d[1] * f3.m_y; 
		intg[6] += d[2] * f3.m_z;

		intg[7] += d[0] * (p0.m_y * g0.m_x + p1.m_y * g1.m_x + p2.m_y * g2.m_x);
		intg[8] += d[1] * (p0.m_z * g0.m_y + p1.m_z * g1.m_y + p2.m_z * g2.m_y);
		intg[9] += d[2] * (p0.m_x * g0.m_z + p1.m_x * g1.m_z + p2.m_x * g2.m_z);

		p1 = p2;
	}
}

ndFloat32 ndPolyhedraMassProperties::MassProperties (ndVector& cg, ndVector& inertia, ndVector& crossInertia)
{
	for (ndInt32 i = 0; i < 10; i++) 
	{
		intg[i] *= mult[i];
	}

	cg.m_x = intg[1];
	cg.m_y = intg[2];
	cg.m_z = intg[3];
	cg.m_w = ndFloat32 (0.0f);
	inertia.m_x = intg[5] + intg[6];
	inertia.m_y = intg[4] + intg[6];
	inertia.m_z = intg[4] + intg[5];
	inertia.m_w = ndFloat32 (0.0f);
	crossInertia.m_x = -intg[8];
	crossInertia.m_y = -intg[9];
	crossInertia.m_z = -intg[7];
	crossInertia.m_w = ndFloat32 (0.0f);
	return intg[0];
}




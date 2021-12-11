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

#ifndef __NDG_DEALUNAY_TETRAHEDRALIZAION_4D__
#define __NDG_DEALUNAY_TETRAHEDRALIZAION_4D__

#include "ndCoreStdafx.h"
#include "ndConvexHull4d.h"

class ndDelaunayTetrahedralization: public ndConvexHull4d
{
	public:
	D_CORE_API ndDelaunayTetrahedralization(const dFloat64* const vertexCloud, dInt32 count, dInt32 strideInByte, dFloat64 distTol);
	D_CORE_API virtual ~ndDelaunayTetrahedralization();
	D_CORE_API void RemoveUpperHull ();

	D_CORE_API dInt32 AddVertex (const ndBigVector& vertex);

	protected:
	D_CORE_API void SortVertexArray();
	D_CORE_API virtual void DeleteFace (ndNode* const node) ;
};

#endif

/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

/****************************************************************************
*
*  Visual C++ 6.0 created by: Julio Jerez
*
****************************************************************************/
#ifndef __D_POLYGONSOUP_BUILDER_H__
#define __D_POLYGONSOUP_BUILDER_H__

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dPlane.h"
#include "dArray.h"
#include "dVector.h"
#include "dMatrix.h"

class AdjacentdFace
{
	public:
	dPlane m_normal;
	dInt32 m_count;
	dInt32 *m_index;
	dInt64 m_edgeMap[256];
};

class dPolygonSoupBuilder: public dClassAlloc 
{
	class dgFaceMap;
	class dgFaceInfo;
	class dgFaceBucket;
	class dgPolySoupFilterAllocator;
	public:

	D_CORE_API dPolygonSoupBuilder ();
	D_CORE_API dPolygonSoupBuilder (const dPolygonSoupBuilder& sopurce);
	D_CORE_API ~dPolygonSoupBuilder ();

	D_CORE_API void Begin();
	D_CORE_API void End(bool optimize);
	//D_CORE_API void AddMesh (const dFloat32* const vertex, dInt32 vertexCount, 
	//						 dInt32 strideInBytes, dInt32 faceCount,
	//						 const dInt32* const faceArray, const dInt32* const indexArray, 
	//						 const dInt32* const faceTagsData, const dMatrix& worldMatrix); 

	D_CORE_API void AddFace(const dFloat32* const vertex, dInt32 strideInBytes, dInt32 vertexCount, const dInt32 faceId);
	D_CORE_API void AddFaceIndirect(const dFloat32* const vertex, dInt32 strideInBytes, dInt32 faceId, const dInt32* const indexArray, dInt32 indexCount);

	D_CORE_API void SavePLY(const char* const fileName) const;

	private:
	void Optimize(dInt32 faceId, const dgFaceBucket& faceBucket, const dPolygonSoupBuilder& source);

	void Finalize();
	void FinalizeAndOptimize();
	void OptimizeByIndividualFaces();
	dInt32 FilterFace (dInt32 count, dInt32* const indexArray);
	dInt32 AddConvexFace (dInt32 count, dInt32* const indexArray, dInt32* const  facesArray);
	void PackArray();

	public:
	class dgVertexArray: public dArray<dBigVector>
	{	
		public:
		dgVertexArray()
			:dArray<dBigVector>()
		{
		}
	};

	class dgIndexArray: public dArray<dInt32>
	{
		public:
		dgIndexArray()
			:dArray<dInt32>()
		{
		}
	};

	dgIndexArray m_faceVertexCount;
	dgIndexArray m_vertexIndex;
	dgIndexArray m_normalIndex;
	dgVertexArray m_vertexPoints;
	dgVertexArray m_normalPoints;
	dInt32 m_run;
};

#endif


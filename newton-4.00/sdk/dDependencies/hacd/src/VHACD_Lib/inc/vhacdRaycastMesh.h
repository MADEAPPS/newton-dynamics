/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
 All rights reserved.


 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

 3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ND_RAYCAST_MESH_H
#define ND_RAYCAST_MESH_H

namespace nd
{
	namespace VHACD
	{
		// Very simple brute force raycast against a triangle mesh.  Tests every triangle; no hierachy.
		// Does a deep copy, always does calculations with full double float precision
		class RaycastMesh
		{
			public:
			static RaycastMesh * createRaycastMesh(uint32_t vcount,		// The number of vertices in the source triangle mesh
				const double *vertices,		// The array of vertex positions in the format x1,y1,z1..x2,y2,z2.. etc.
				uint32_t tcount,		// The number of triangles in the source triangle mesh
				const uint32_t *indices); // The triangle indices in the format of i1,i2,i3 ... i4,i5,i6, ...

			static RaycastMesh * createRaycastMesh(uint32_t vcount,		// The number of vertices in the source triangle mesh
				const float *vertices,		// The array of vertex positions in the format x1,y1,z1..x2,y2,z2.. etc.
				uint32_t tcount,		// The number of triangles in the source triangle mesh
				const uint32_t *indices); // The triangle indices in the format of i1,i2,i3 ... i4,i5,i6, ...


			virtual bool raycast(const double *from,			// The starting point of the raycast
				const double *to,				// The ending point of the raycast
				const double *closestToPoint,	// The point to match the nearest hit location (can just be the 'from' location of no specific point)
				double *hitLocation,			// The point where the ray hit nearest to the 'closestToPoint' location
				double *hitDistance) = 0;		// The distance the ray traveled to the hit location

			virtual void release(void) = 0;
		protected:
			virtual ~RaycastMesh(void) { }
		};

	} // end of VHACD namespace
}

#endif

/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
 All rights reserved.
 
 
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 
 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 
 3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ND_VHACD_H
#define ND_VHACD_H

#define ND_VHACD_VERSION_MAJOR 2
#define ND_VHACD_VERSION_MINOR 3

// Changes for version 2.3
//
// m_gamma : Has been removed.  This used to control the error metric to merge convex hulls.  Now it uses the 'm_maxConvexHulls' value instead.
// m_maxConvexHulls : This is the maximum number of convex hulls to produce from the merge operation; replaces 'm_gamma'.
//
// Note that decomposition depth is no longer a user provided value.  It is now derived from the 
// maximum number of hulls requested.
//
// As a convenience to the user, each convex hull produced now includes the volume of the hull as well as it's center.
//
// This version supports a convenience method to automatically make V-HACD run asynchronously in a background thread.
// To get a fully asynchronous version, call 'CreateVHACD_ASYNC' instead of 'CreateVHACD'.  You get the same interface however,
// now when computing convex hulls, it is no longer a blocking operation.  All callback messages are still returned
// in the application's thread so you don't need to worry about mutex locks or anything in that case.
// To tell if the operation is complete, the application should call 'IsReady'.  This will return true if
// the last approximation operation is complete and will dispatch any pending messages.
// If you call 'Compute' while a previous operation was still running, it will automatically cancel the last request
// and begin a new one.  To cancel a currently running approximation just call 'Cancel'.

#ifdef _WIN32
	#ifndef WIN32_LEAN_AND_MEAN
		#define WIN32_LEAN_AND_MEAN // Exclude rarely-used stuff from Windows headers
	#endif
	#include <windows.h>
#elif __MACH__
	#include <mach/mach.h>
#else
	#include <unistd.h>
#endif

#include <stdlib.h>

namespace nd
{
	namespace VHACD 
	{
		class IVHACD 
		{
			public:
			class ConvexHull 
			{
				public:
				double* m_points;
				ndInt32* m_triangles;
				ndInt32 m_nPoints;
				ndInt32 m_nTriangles;
			};

			class Parameters 
			{
				public:
				Parameters(void) 
				{ 
					Init(); 
				}
				void Init(void)
				{
					m_resolution = 1000000;
					m_concavity = 0.001;
					m_minMergeToleranace = 0.5e-3f;
					m_concavityToVolumeWeigh = 1.0f;
					m_planeDownsampling = 4;
					m_convexhullDownsampling = 4;
					m_alpha = 0.05;
					m_beta = 0.05;
					m_maxNumVerticesPerCH = 64;
					m_minVolumePerCH = 0.0001;
					//m_maxConvexHulls = 1024;
					m_maxConvexHulls = 128;
				}
				double m_concavity;
				double m_concavityToVolumeWeigh;
				double m_alpha;
				double m_beta;
				double m_minVolumePerCH;
				double m_minMergeToleranace;
				ndInt32 m_resolution;
				ndInt32 m_maxNumVerticesPerCH;
				ndInt32 m_planeDownsampling;
				ndInt32 m_convexhullDownsampling;
				ndInt32 m_maxConvexHulls;
			};

			virtual bool Compute(const float* const points,
				const uint32_t countPoints,
				const uint32_t* const triangles,
				const uint32_t countTriangles,
				const Parameters& params)
				= 0;
			virtual uint32_t GetNConvexHulls() const = 0;
			virtual void GetConvexHull(const uint32_t index, ConvexHull& ch) const = 0;
			virtual void Clean(void) = 0; // release internally allocated memory
			virtual void Release(void) = 0; // release IVHACD

			protected:
			virtual ~IVHACD(void) {}
		};
		IVHACD* CreateVHACD(void);
	}
}
#endif // VHACD_H

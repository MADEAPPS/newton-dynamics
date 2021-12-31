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
#include "ndDebug.h"
#include "ndVector.h"
#include "ndMatrix.h"
#include "ndIsoSurface.h"

const ndInt32 ndIsoSurface::m_edgeTable[256] =
{
	0x0  , 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
	0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
	0x190, 0x99 , 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
	0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
	0x230, 0x339, 0x33 , 0x13a, 0x636, 0x73f, 0x435, 0x53c,
	0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
	0x3a0, 0x2a9, 0x1a3, 0xaa , 0x7a6, 0x6af, 0x5a5, 0x4ac,
	0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
	0x460, 0x569, 0x663, 0x76a, 0x66 , 0x16f, 0x265, 0x36c,
	0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
	0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff , 0x3f5, 0x2fc,
	0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
	0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55 , 0x15c,
	0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
	0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc ,
	0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
	0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
	0xcc , 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
	0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
	0x15c, 0x55 , 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
	0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
	0x2fc, 0x3f5, 0xff , 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
	0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
	0x36c, 0x265, 0x16f, 0x66 , 0x76a, 0x663, 0x569, 0x460,
	0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac,
	0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa , 0x1a3, 0x2a9, 0x3a0,
	0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
	0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33 , 0x339, 0x230,
	0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
	0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x99 , 0x190,
	0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
	0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0
};

const ndInt32 ndIsoSurface::m_triangleTable[256][16] =
{
	{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1 },
	{ 2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1 },
	{ 8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1 },
	{ 3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1 },
	{ 4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1 },
	{ 4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1 },
	{ 5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1 },
	{ 2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1 },
	{ 9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1 },
	{ 2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1 },
	{ 10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1 },
	{ 5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1 },
	{ 5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1 },
	{ 10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1 },
	{ 8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1 },
	{ 2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1 },
	{ 7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1 },
	{ 2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1 },
	{ 11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1 },
	{ 5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1 },
	{ 11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1 },
	{ 11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1 },
	{ 5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1 },
	{ 2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1 },
	{ 5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1 },
	{ 6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1 },
	{ 3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1 },
	{ 6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1 },
	{ 5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1 },
	{ 10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1 },
	{ 6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1 },
	{ 8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1 },
	{ 7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1 },
	{ 3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1 },
	{ 5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1 },
	{ 0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1 },
	{ 9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1 },
	{ 8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1 },
	{ 5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1 },
	{ 0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1 },
	{ 6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1 },
	{ 10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1 },
	{ 10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1 },
	{ 8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1 },
	{ 1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1 },
	{ 0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1 },
	{ 10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1 },
	{ 3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1 },
	{ 6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1 },
	{ 9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1 },
	{ 8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1 },
	{ 3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1 },
	{ 6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1 },
	{ 10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1 },
	{ 10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1 },
	{ 2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1 },
	{ 7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1 },
	{ 7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1 },
	{ 2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1 },
	{ 1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1 },
	{ 11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1 },
	{ 8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1 },
	{ 0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1 },
	{ 7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1 },
	{ 10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1 },
	{ 2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1 },
	{ 6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1 },
	{ 7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1 },
	{ 2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1 },
	{ 10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1 },
	{ 10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1 },
	{ 0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1 },
	{ 7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1 },
	{ 6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1 },
	{ 8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1 },
	{ 6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1 },
	{ 4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1 },
	{ 10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1 },
	{ 8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1 },
	{ 1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1 },
	{ 8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1 },
	{ 10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1 },
	{ 10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1 },
	{ 5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1 },
	{ 11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1 },
	{ 9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1 },
	{ 6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1 },
	{ 7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1 },
	{ 3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1 },
	{ 7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1 },
	{ 3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1 },
	{ 6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1 },
	{ 9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1 },
	{ 1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1 },
	{ 4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1 },
	{ 7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1 },
	{ 6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1 },
	{ 0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1 },
	{ 6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1 },
	{ 0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1 },
	{ 11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1 },
	{ 6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1 },
	{ 5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1 },
	{ 9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1 },
	{ 1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1 },
	{ 10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1 },
	{ 0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1 },
	{ 5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1 },
	{ 10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1 },
	{ 11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1 },
	{ 9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1 },
	{ 7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1 },
	{ 2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1 },
	{ 8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1 },
	{ 9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1 },
	{ 9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1 },
	{ 1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1 },
	{ 5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1 },
	{ 0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1 },
	{ 10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1 },
	{ 2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1 },
	{ 0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1 },
	{ 0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1 },
	{ 9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1 },
	{ 5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1 },
	{ 5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1 },
	{ 8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1 },
	{ 9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1 },
	{ 1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1 },
	{ 3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1 },
	{ 4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1 },
	{ 9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1 },
	{ 11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1 },
	{ 11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1 },
	{ 2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1 },
	{ 9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1 },
	{ 3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1 },
	{ 1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1 },
	{ 4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1 },
	{ 0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1 },
	{ 1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }
};

ndVector ndIsoSurface::ndOctreeInterface::m_neighbors[3][3][3] =
{
	// z = -1
	ndVector(ndFloat32(-1.0f), ndFloat32(-1.0f), ndFloat32(-1.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32( 0.0f), ndFloat32(-1.0f), ndFloat32(-1.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32( 1.0f), ndFloat32(-1.0f), ndFloat32(-1.0f), ndFloat32(0.0f)),

	ndVector(ndFloat32(-1.0f), ndFloat32( 0.0f), ndFloat32(-1.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32( 0.0f), ndFloat32( 0.0f), ndFloat32(-1.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32( 0.0f), ndFloat32( 0.0f), ndFloat32(-1.0f), ndFloat32(0.0f)),

	ndVector(ndFloat32(-1.0f), ndFloat32( 1.0f), ndFloat32(-1.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32(-1.0f), ndFloat32( 1.0f), ndFloat32(-1.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32(-1.0f), ndFloat32( 1.0f), ndFloat32(-1.0f), ndFloat32(0.0f)),

	// z = 0
	ndVector(ndFloat32(-1.0f), ndFloat32(-1.0f), ndFloat32( 0.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32( 0.0f), ndFloat32(-1.0f), ndFloat32( 0.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32( 1.0f), ndFloat32(-1.0f), ndFloat32( 0.0f), ndFloat32(0.0f)),

	ndVector(ndFloat32(-1.0f), ndFloat32( 0.0f), ndFloat32( 0.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32( 0.0f), ndFloat32( 0.0f), ndFloat32( 0.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32( 1.0f), ndFloat32( 0.0f), ndFloat32( 0.0f), ndFloat32(0.0f)),

	ndVector(ndFloat32(-1.0f), ndFloat32( 1.0f), ndFloat32( 0.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32( 0.0f), ndFloat32( 1.0f), ndFloat32( 0.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32( 1.0f), ndFloat32( 1.0f), ndFloat32( 0.0f), ndFloat32(0.0f)),

	// z = 1
	ndVector(ndFloat32(-1.0f), ndFloat32(-1.0f), ndFloat32( 1.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32( 0.0f), ndFloat32(-1.0f), ndFloat32( 1.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32( 1.0f), ndFloat32(-1.0f), ndFloat32( 1.0f), ndFloat32(0.0f)),

	ndVector(ndFloat32(-1.0f), ndFloat32( 0.0f), ndFloat32( 1.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32( 0.0f), ndFloat32( 0.0f), ndFloat32( 1.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32( 1.0f), ndFloat32( 0.0f), ndFloat32( 1.0f), ndFloat32(0.0f)),

	ndVector(ndFloat32(-1.0f), ndFloat32( 1.0f), ndFloat32(1.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32( 0.0f), ndFloat32( 1.0f), ndFloat32(1.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32( 1.0f), ndFloat32( 1.0f), ndFloat32(1.0f), ndFloat32(0.0f)),
};

ndVector ndIsoSurface::ndOctree::m_quadrantCode(1, 2, 4, 0);


class ndIsoSurface::ndIsoCell
{
	public:
	ndFloat32 m_isoValues[2][2][2];
	ndInt32 m_x;
	ndInt32 m_y;
	ndInt32 m_z;
};

ndIsoSurface::ndIsoSurface()
	:m_origin(ndVector::m_zero)
	,m_gridSize(ndVector::m_zero)
	,m_invGridSize(ndVector::m_zero)
	,m_octree()
	,m_points(1024)
	,m_normals(1024)
	,m_trianglesList(1024)
	,m_vertexMap()
	,m_triangleMap()
	,m_xCellSize(0)
	,m_yCellSize(0)
	,m_zCellSize(0)
	,m_isoValue(ndFloat32(0.0f))
{
}

ndIsoSurface::~ndIsoSurface()
{
}

void ndIsoSurface::Begin(const ndVector& boxP0, const ndVector& boxP1, ndFloat32 gridSize)
{
	m_isoValue = ndFloat32(0.5f);
	m_gridSize = ndVector(gridSize) & ndVector::m_triplexMask;
	m_invGridSize = ndVector(ndFloat32(1.0f) / gridSize) & ndVector::m_triplexMask;

	m_origin = boxP0 - m_gridSize - m_gridSize * ndVector::m_half;
	ndVector size((boxP1 - m_origin) * m_invGridSize + ndVector::m_half);

	ndVector cells(size.Floor().GetInt());
	m_xCellSize = cells.m_ix + 2;
	m_yCellSize = cells.m_iy + 2;
	m_zCellSize = cells.m_iz + 2;
	ndInt32 maxAxis = dMax(dMax(m_xCellSize, m_yCellSize), m_zCellSize);

	ndInt32 maxAxisTwosPower = 1;
	for (; maxAxisTwosPower < maxAxis; maxAxisTwosPower *= 2);

	const ndVector boundP0(ndVector::m_zero);
	const ndVector boundP1(ndVector(ndFloat32(maxAxisTwosPower)) & ndVector::m_triplexMask);
	m_octree = new ndOctree(boundP0, boundP1, nullptr);
}

void ndIsoSurface::AddPoint(const ndVector& point)
{
	const ndVector p(ndVector::m_triplexMask & (m_invGridSize * (point - m_origin) + ndVector::m_half));
	const ndVector cellPoint(p.Floor() - ndVector::m_half);
	m_octree->Insert(cellPoint, 0);
}

void ndIsoSurface::End()
{
	dAssert(m_octree);

	m_points.SetCount(0);
	m_trianglesList.SetCount(0);

	m_octree->ProccessCells(this);

	m_trianglesList.SetCount(m_triangleMap.GetCount());
	ndIsoTriangleMap::Iterator iter(m_triangleMap);
	ndInt32 index = 0;
	for (iter.Begin(); iter; iter++)
	{
		m_trianglesList[index] = iter.GetKey();
		index++;
	}

	RemapIndexList();
	CalculateNormals();

	delete m_octree;
	m_octree = nullptr;
	m_vertexMap.RemoveAll();
	m_triangleMap.RemoveAll();
}

ndUnsigned64 ndIsoSurface::GetVertexID(ndInt32 gridX, ndInt32 gridY, ndInt32 gridZ)
{
	//return 3 * (gridZ*(m_nCellsY + 1)*(m_nCellsX + 1) + gridY*(m_nCellsX + 1) + gridX);
	return 3 * (m_xCellSize * ndUnsigned64(gridZ * m_yCellSize + gridY) + gridX);
}

ndUnsigned64 ndIsoSurface::GetEdgeID(const ndIsoCell& cell, ndInt32 edgeCode)
{
	const ndInt32 gridX = cell.m_x;
	const ndInt32 gridY = cell.m_y;
	const ndInt32 gridZ = cell.m_z;
	switch (edgeCode)
	{
		case 0:
			return GetVertexID(gridX, gridY, gridZ) + 1;
		case 1:
			return GetVertexID(gridX, gridY + 1, gridZ);
		case 2:
			return GetVertexID(gridX + 1, gridY, gridZ) + 1;
		case 3:
			return GetVertexID(gridX, gridY, gridZ);
		case 4:
			return GetVertexID(gridX, gridY, gridZ + 1) + 1;
		case 5:
			return GetVertexID(gridX, gridY + 1, gridZ + 1);
		case 6:
			return GetVertexID(gridX + 1, gridY, gridZ + 1) + 1;
		case 7:
			return GetVertexID(gridX, gridY, gridZ + 1);
		case 8:
			return GetVertexID(gridX, gridY, gridZ) + 2;
		case 9:
			return GetVertexID(gridX, gridY + 1, gridZ) + 2;
		case 10:
			return GetVertexID(gridX + 1, gridY + 1, gridZ) + 2;
		case 11:
			return GetVertexID(gridX + 1, gridY, gridZ) + 2;
		default:
			// Invalid edge no.
			return ndUnsigned64 (-1);
	}
}

ndVector ndIsoSurface::InterpolateEdge(ndFloat32 fX1, ndFloat32 fY1, ndFloat32 fZ1, ndFloat32 fX2, ndFloat32 fY2, ndFloat32 fZ2, ndFloat32 tVal1, ndFloat32 tVal2)
{
	ndFloat32 mu = (m_isoValue - tVal1) / (tVal2 - tVal1);
	ndFloat32 x = fX1 + mu*(fX2 - fX1);
	ndFloat32 y = fY1 + mu*(fY2 - fY1);
	ndFloat32 z = fZ1 + mu*(fZ2 - fZ1);
	return ndVector(x, y, z, ndFloat32 (0.0f));
}

ndVector ndIsoSurface::CalculateIntersection(const ndIsoCell& cell, ndInt32 edgeCode)
{
	ndInt32 v1x = cell.m_x;
	ndInt32 v1y = cell.m_y;
	ndInt32 v1z = cell.m_z;
	ndInt32 v2x = cell.m_x;
	ndInt32 v2y = cell.m_y;
	ndInt32 v2z = cell.m_z;
	
	switch (edgeCode)
	{
		case 0:
			v2y += 1;
			break;
		case 1:
			v1y += 1;
			v2x += 1;
			v2y += 1;
			break;
		case 2:
			v1x += 1;
			v1y += 1;
			v2x += 1;
			break;
		case 3:
			v1x += 1;
			break;
		case 4:
			v1z += 1;
			v2y += 1;
			v2z += 1;
			break;
		case 5:
			v1y += 1;
			v1z += 1;
			v2x += 1;
			v2y += 1;
			v2z += 1;
			break;
		case 6:
			v1x += 1;
			v1y += 1;
			v1z += 1;
			v2x += 1;
			v2z += 1;
			break;
		case 7:
			v1x += 1;
			v1z += 1;
			v2z += 1;
			break;
		case 8:
			v2z += 1;
			break;
		case 9:
			v1y += 1;
			v2y += 1;
			v2z += 1;
			break;
		case 10:
			v1x += 1;
			v1y += 1;
			v2x += 1;
			v2y += 1;
			v2z += 1;
			break;
		case 11:
			v1x += 1;
			v2x += 1;
			v2z += 1;
			break;
	}
	
	ndFloat32 x1 = v1x * m_gridSize.m_x;
	ndFloat32 y1 = v1y * m_gridSize.m_y;
	ndFloat32 z1 = v1z * m_gridSize.m_z;
	ndFloat32 x2 = v2x * m_gridSize.m_x;
	ndFloat32 y2 = v2y * m_gridSize.m_y;
	ndFloat32 z2 = v2z * m_gridSize.m_z;
	
	ndFloat32 val1 = cell.m_isoValues[v1z-cell.m_z][v1y-cell.m_y][v1x-cell.m_x];
	ndFloat32 val2 = cell.m_isoValues[v2z - cell.m_z][v2y - cell.m_y][v2x - cell.m_x];
	return InterpolateEdge(x1, y1, z1, x2, y2, z2, val1, val2);
}

void ndIsoSurface::ProcessCell(const ndIsoCell& cell)
{
	dAssert(cell.m_x < (m_xCellSize));
	dAssert(cell.m_y < (m_yCellSize));
	dAssert(cell.m_z < (m_zCellSize));
	
	ndInt32 tableIndex = 0;
	if (cell.m_isoValues[0][0][0] > m_isoValue)
	{
		tableIndex |= 1;
	}
	if (cell.m_isoValues[0][1][0] > m_isoValue)
	{
		tableIndex |= 2;
	}
	if (cell.m_isoValues[0][1][1] > m_isoValue)
	{
		tableIndex |= 4;
	}
	if (cell.m_isoValues[0][0][1] > m_isoValue)
	{
		tableIndex |= 8;
	}
	if (cell.m_isoValues[1][0][0] > m_isoValue)
	{
		tableIndex |= 16;
	}
	if (cell.m_isoValues[1][1][0] > m_isoValue)
	{
		tableIndex |= 32;
	}
	if (cell.m_isoValues[1][1][1] > m_isoValue)
	{
		tableIndex |= 64;
	}
	if (cell.m_isoValues[1][0][1] > m_isoValue)
	{
		tableIndex |= 128;
	}

	// Now create a triangulation of the iso surface in this cell.
	ndInt32 edgeBits = m_edgeTable[tableIndex];
	if (edgeBits != 0)
	{
		if (edgeBits & 8)
		{
			ndVector pt (CalculateIntersection(cell, 3));
			ndUnsigned64 id = GetEdgeID(cell, 3);
			m_vertexMap.Insert(pt, id);
		}
		if (edgeBits & 1)
		{
			ndVector pt (CalculateIntersection(cell, 0));
			ndUnsigned64 id = GetEdgeID(cell, 0);
			m_vertexMap.Insert(pt, id);
		}
		if (edgeBits & 256)
		{
			ndVector pt (CalculateIntersection(cell, 8));
			ndUnsigned64 id = GetEdgeID(cell, 8);
			m_vertexMap.Insert(pt, id);
		}
	
		for (ndInt32 i = 0; m_triangleTable[tableIndex][i] != -1; i += 3)
		{
			ndIsoTriangle triangle;
			ndUnsigned64 pointID0 = GetEdgeID(cell, m_triangleTable[tableIndex][i + 0]);
			ndUnsigned64 pointID1 = GetEdgeID(cell, m_triangleTable[tableIndex][i + 1]);
			ndUnsigned64 pointID2 = GetEdgeID(cell, m_triangleTable[tableIndex][i + 2]);
			triangle.m_pointId[0] = pointID0;
			triangle.m_pointId[1] = pointID1;
			triangle.m_pointId[2] = pointID2;
			m_triangleMap.Insert(0, triangle);
			#ifdef _DEBUG
				ndIsoTriangle triangle1;
				triangle1.m_pointId[0] = triangle.m_pointId[1];
				triangle1.m_pointId[1] = triangle.m_pointId[2];
				triangle1.m_pointId[2] = triangle.m_pointId[0];
				dAssert(!m_triangleMap.Find(triangle1));

				triangle1.m_pointId[0] = triangle.m_pointId[2];
				triangle1.m_pointId[1] = triangle.m_pointId[0];
				triangle1.m_pointId[2] = triangle.m_pointId[1];
				dAssert(!m_triangleMap.Find(triangle1));
			#endif
			
		}
	}
}

void ndIsoSurface::RemapIndexList()
{
	ndInt32 nextID = 0;

	// calculate monotonic index list
	m_points.SetCount(m_vertexMap.GetCount());
	ndIsoVertexMap::Iterator iter(m_vertexMap);
	for (iter.Begin(); iter; iter++)
	{
		ndVector& point = iter.GetNode()->GetInfo();
		m_points[nextID] = point + m_origin;
		point.m_w = ndFloat32(nextID);
		nextID ++;
	}

	// Now remap triangles.
	for (ndInt32 k = 0; k < m_trianglesList.GetCount(); k++)
	{
		const ndIsoTriangle& triangle = m_trianglesList[k];
		for (ndInt32 i = 0; i < 3; i++)
		{
			ndIsoVertexMap::ndNode* const node = m_vertexMap.Find(triangle.m_pointId[i]);
			dAssert(node);
			ndInt32 id = ndInt32 (node->GetInfo().m_w);
			m_trianglesList[k].m_pointId[i] = id;
		}
	}
}

void ndIsoSurface::CalculateNormals()
{
	m_normals.SetCount(m_points.GetCount());

	// Set all normals to 0.
	if (m_normals.GetCount())
	{
		memset(&m_normals[0], 0, m_normals.GetCount() * sizeof(ndVector));

		for (ndInt32 i = 0; i < m_trianglesList.GetCount(); i++)
		{
			ndInt32 id0 = ndInt32(m_trianglesList[i].m_pointId[0]);
			ndInt32 id1 = ndInt32(m_trianglesList[i].m_pointId[1]);
			ndInt32 id2 = ndInt32(m_trianglesList[i].m_pointId[2]);
			ndVector vec1(m_points[id1] - m_points[id0]);
			ndVector vec2(m_points[id2] - m_points[id0]);
			ndVector normal = vec1.CrossProduct(vec2);
			m_normals[id0] += normal;
			m_normals[id1] += normal;
			m_normals[id2] += normal;
		}
		
		// Normalize normals.
		for (ndInt32 i = 0; i < m_normals.GetCount(); i++)
		{
			//m_normals[i] = m_normals[i].Normalize();
			m_normals[i] = m_normals[i] * m_normals[i].InvMagSqrt();
		}
	}
}


ndIsoSurface::ndOctree::ndOctree(const ndVector& box0, const ndVector& box1, ndOctreeInterface* const parent)
	:ndOctreeInterface(parent)
	,m_box0(box0)
	,m_box1(box1)
{
	for (ndInt32 i = 0; i < 8; ++i)
	{
		m_children[i] = nullptr;
	}
}

ndIsoSurface::ndOctree::~ndOctree()
{
	for (ndInt32 i = 0; i < 8; ++i)
	{
		if (m_children[i])
		{
			delete m_children[i];
		}
	}
}

void ndIsoSurface::ndOctree::Insert(const ndVector& point, ndInt32 parentQuadrant)
{
	if (m_fullMask == 0xff)
	{
		dAssert(0);
		return;
	}

	const ndVector size(ndVector::m_half * (m_box1 - m_box0));
	const ndVector center(ndVector::m_half * (m_box1 + m_box0));
	const ndVector mask(point > center);
	const ndVector box0(m_box0.Select(center, mask));
	const ndVector box1(box0 + size);
	const ndVector quadrant(mask & m_quadrantCode);
	const ndInt32 code = quadrant.m_iz + quadrant.m_iy + quadrant.m_ix;
	
	if (size.m_x > ndFloat32 (1.0f))
	{
		if (!m_children[code])
		{
			m_children[code] = new ndOctree(box0, box1, this);
		}
		m_children[code]->Insert(point, code);
	
		if (m_fullMask == 0xff)
		{
			for (ndInt32 i = 0; i < 8; ++i)
			{
				dAssert(m_children[i]);
				delete m_children[i];
				m_children[i] = nullptr;
			}
			if (m_parent)
			{
				m_parent->m_fullMask = m_parent->m_fullMask | (1 << parentQuadrant);
			}
		}
	}
	else if (!m_children[code])
	{
		m_children[code] = new ndOctreeLeaf(point, this);
		m_fullMask = m_fullMask | (1 << code);
		if (m_fullMask == 0xff)
		{
			for (ndInt32 i = 0; i < 8; ++i)
			{
				dAssert(m_children[i]);
				delete m_children[i];
				m_children[i] = nullptr;
			}
			if (m_parent)
			{
				m_parent->m_fullMask = m_parent->m_fullMask | (1 << parentQuadrant);
			}
		}
	}
}

bool ndIsoSurface::ndOctree::Find(const ndVector& point) const
{
	const ndOctree* root = this;
	
	ndInt32 isInsize = CheckInside(point, root->m_box0, root->m_box1);
	while (!isInsize)
	{
		root = (ndOctree*)root->m_parent;
		dAssert(root);
		isInsize = CheckInside(point, root->m_box0, root->m_box1);
	}
	
	ndInt32 stack = 1;
	const ndOctreeInterface* stackPool[32];
	stackPool[0] = root;
	while (stack)
	{
		stack--;
		const ndOctreeInterface* const node = stackPool[stack];
		if (node->m_fullMask == 0xff)
		{
			return true;
		}
		if (node->m_isLeaf)
		{
			const ndOctreeLeaf* const leafNode = (ndOctreeLeaf*)node;
			const ndVector diff(leafNode->m_point - point);
			const ndVector mask(diff == ndVector::m_zero);
			ndInt32 test = mask.m_ix & mask.m_iy & mask.m_iz;
			if (test) 
			{
				return true;
			}
		}
		else
		{
			const ndOctree* const parent = (ndOctree*)node;
			for (ndInt32 i = 0; i < 8; i++)
			{
				const ndOctreeInterface* const child = parent->m_children[i];
				if (child)
				{
					if (child->m_isLeaf) 
					{
						stackPool[stack] = child;
						stack++;
					}
					else
					{
						const ndOctree* const childNode = (ndOctree*)child;
						ndInt32 test = CheckInside(point, childNode->m_box0, childNode->m_box1);
						if (test)
						{
							stackPool[stack] = childNode;
							stack++;
						}
					}
				}
			}
		}
	}
	return false;
}

void ndIsoSurface::ndOctreeLeaf::ProccessCells(ndIsoSurface* const isoSurface)
{
	ndFloat32 isoValue[3][3][3];
	const ndVector* const src = &m_neighbors[0][0][0];
	ndFloat32* const dstIsoValue = &isoValue[0][0][0];
	
	for (ndInt32 i = 0; i < sizeof (isoValue)/sizeof (ndFloat32); ++i)
	{
		ndVector p(m_point + src[i]);
		bool hasNeigborg = m_parent->Find(p);
		dstIsoValue[i] = hasNeigborg ? ndFloat32(1.0f) : ndFloat32(0.0f);
	}

	ndIsoCell cell;
	ndVector point((m_point + ndVector::m_half).GetInt());
	for (ndInt32 z = 0; z < 2; ++z)
	{
		cell.m_z = point.m_iz + z;
		for (ndInt32 y = 0; y < 2; ++y)
		{
			cell.m_y = point.m_iy + y;
			for (ndInt32 x = 0; x < 2; ++x)
			{
				cell.m_x = point.m_ix + x;
				cell.m_isoValues[0][0][0] = isoValue[z + 0][y + 0][x + 0];
				cell.m_isoValues[0][0][1] = isoValue[z + 0][y + 0][x + 1];
				cell.m_isoValues[0][1][0] = isoValue[z + 0][y + 1][x + 0];
				cell.m_isoValues[0][1][1] = isoValue[z + 0][y + 1][x + 1];
				cell.m_isoValues[1][0][0] = isoValue[z + 1][y + 0][x + 0];
				cell.m_isoValues[1][0][1] = isoValue[z + 1][y + 0][x + 1];
				cell.m_isoValues[1][1][0] = isoValue[z + 1][y + 1][x + 0];
				cell.m_isoValues[1][1][1] = isoValue[z + 1][y + 1][x + 1];
				isoSurface->ProcessCell(cell);
			}
		}
	}
}

void ndIsoSurface::ndOctree::ProccessCells(ndIsoSurface* const isoSurface)
{
	if (m_fullMask == 0xff)
	{
		dAssert(0);
	}
	else
	{
		for (ndInt32 i = 0; i < 8; i++)
		{
			if (m_children[i])
			{
				m_children[i]->ProccessCells(isoSurface);
			}
		}
	}
}


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

#ifndef _ND_DEEP_BRAIN_VECTOR_H__
#define _ND_DEEP_BRAIN_VECTOR_H__

#include "ndDeepBrainStdafx.h"

class ndDeepBrainVector: public ndArray<ndReal> 
{
	public: 
	ndDeepBrainVector();
	~ndDeepBrainVector();
	
	void InitGaussianWeights(ndReal mean, ndReal variance);

	void Set(ndReal value);
	void Set(const ndDeepBrainVector& data);

	ndReal Dot(const ndDeepBrainVector& a) const;
	void ScaleSet(const ndDeepBrainVector& a, ndReal scale);
	void Add(const ndDeepBrainVector& a, const ndDeepBrainVector& b);
	void Sub(const ndDeepBrainVector& a, const ndDeepBrainVector& b);
	void Mul(const ndDeepBrainVector& a, const ndDeepBrainVector& b);

	void ScaleAdd(const ndDeepBrainVector& a, ndReal b);
	void MulAdd(const ndDeepBrainVector& a, const ndDeepBrainVector& b);
	void MulSub(const ndDeepBrainVector& a, const ndDeepBrainVector& b);
};

class ndDeepBrainMemVector : public ndDeepBrainVector
{
	public:
	ndDeepBrainMemVector(const ndReal* const mem, ndInt32 size);
	~ndDeepBrainMemVector();
};

#endif 

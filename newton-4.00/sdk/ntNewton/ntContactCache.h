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

#ifndef __D_CONTACT_CACHE_H__
#define __D_CONTACT_CACHE_H__

#include "ntStdafx.h"

#define D_CONTACT_CACHE_LINE_SIZE 4

class ntContact;

class ntCacheEntryTag
{
	public:
	ntCacheEntryTag() {}
	ntCacheEntryTag(dUnsigned32 tag0, dUnsigned32 tag1)
		:m_tagLow(dMin(tag0, tag1))
		,m_tagHigh(dMax(tag0, tag1))
	{
	}

	dUnsigned32 GetHash() const
	{
		return m_tagHigh * 31415821u + m_tagLow;
	}

	union
	{
		dUnsigned64 m_tag;
		struct
		{
			dUnsigned32 m_tagLow;
			dUnsigned32 m_tagHigh;
		};
	};
};

class ntContactCacheLine
{
	public:
	ntContactCacheLine()
	{
	}

	dInt32 m_count;
	dUnsigned32 m_key;
	ntContact* m_contact[D_CONTACT_CACHE_LINE_SIZE];
	dInt32 m_hashKey[D_CONTACT_CACHE_LINE_SIZE];
	ntCacheEntryTag m_tags[D_CONTACT_CACHE_LINE_SIZE];
};

class ntContactCache: public dArray<ntContactCacheLine>
{
	public:
	ntContactCache()
		:dArray<ntContactCacheLine>()
		,m_count(1 << 10)
	{
		Init();
	}

	void Flush()
	{
		Clear();
		Init();
	}

	void Init()
	{
		m_count = 1 << 10;
		ResizeIfNecessary(m_count);
		ntContactCacheLine* const cache = &(*this)[0];
		memset(cache, 0, m_count * sizeof(ntContactCacheLine));
	}

	ntContact* FindContactJoint(const ntBody* const body0, const ntBody* const body1)
	{
		ntCacheEntryTag tag(body0->GetID(), body1->GetID());
		dUnsigned32 hash = tag.GetHash();

		dInt32 entry = hash & (m_count - 1);

		ntContactCacheLine& cacheLine = (*this)[entry];
		for (dInt32 i = 0; i < cacheLine.m_count; i++) 
		{
			if (cacheLine.m_tags[i].m_tag == tag.m_tag) 
			{
				return cacheLine.m_contact[i];
			}
		}
		return nullptr;
	}

	bool AddContactJoint(ntContact* const joint)
	{
		dAssert(0);
		return false;
		//// note this function is not thread safe
		//ntCacheEntryTag tag(joint->GetBody0()->m_uniqueID, joint->GetBody1()->m_uniqueID);
		//dUnsigned32 hash = tag.GetHash();
		//
		//dInt32 entry = hash & (m_count - 1);
		//ntContactCacheLine* cacheLine = &(*this)[entry];
		//
		//for (dInt32 i = cacheLine->m_count - 1; i >= 0; i--) 
		//{
		//	if (cacheLine->m_tags[i].m_tag == tag.m_tag) 
		//	{
		//		return false;
		//	}
		//}
		//
		//while (cacheLine->m_count == 4) 
		//{
		//	Rehash();
		//	entry = hash & (m_count - 1);
		//	cacheLine = &(*this)[entry];
		//}
		//if (cacheLine->m_count == 0) 
		//{
		//	cacheLine->m_key = hash;
		//}
		//
		//const dInt32 index = cacheLine->m_count;
		//cacheLine->m_count++;
		//cacheLine->m_tags[index] = tag;
		//cacheLine->m_hashKey[index] = hash;
		//cacheLine->m_contact[index] = joint;
		//return true;
	}

	void RemoveContactJoint(ntContact* const joint)
	{
		dAssert(0);
		//ntCacheEntryTag tag(joint->GetBody0()->m_uniqueID, joint->GetBody1()->m_uniqueID);
		//dUnsigned32 hash = tag.GetHash();
		//
		//dInt32 entry = hash & (m_count - 1);
		//ntContactCacheLine* const cacheLine = &(*this)[entry];
		//for (dInt32 i = cacheLine->m_count - 1; i >= 0; i--) 
		//{
		//	if (cacheLine->m_tags[i].m_tag == tag.m_tag) 
		//	{
		//		cacheLine->m_count--;
		//		const dInt32 index = cacheLine->m_count;
		//		cacheLine->m_tags[i] = cacheLine->m_tags[index];
		//		cacheLine->m_hashKey[i] = cacheLine->m_hashKey[index];
		//		cacheLine->m_contact[i] = cacheLine->m_contact[index];
		//		break;
		//	}
		//}
	}

	private:
	void Rehash()
	{
		const dInt32 newCount = m_count * 2;
		ResizeIfNecessary(newCount);
		ntContactCacheLine* const cache0 = &(*this)[0];
		ntContactCacheLine* const cache1 = &cache0[m_count];

		const dInt32 mask = newCount - 1;
		for (dInt32 i = 0; i < m_count; i++) 
		{
			ntContactCacheLine* const src = &cache0[i];
			ntContactCacheLine* const dst = &cache1[i];
			dst->m_count = 0;
			for (dInt32 j = src->m_count - 1; j >= 0; j--) 
			{
				dInt32 entry = src->m_hashKey[j] & mask;
				if (entry >= m_count) 
				{
					const dInt32 dstIndex = dst->m_count;
					dst->m_count++;
					dst->m_tags[dstIndex] = src->m_tags[j];
					dst->m_hashKey[dstIndex] = src->m_hashKey[j];
					dst->m_contact[dstIndex] = src->m_contact[j];

					src->m_count--;
					const dInt32 srcIndex = src->m_count;
					src->m_tags[j] = src->m_tags[srcIndex];
					src->m_hashKey[j] = src->m_hashKey[srcIndex];
					src->m_contact[j] = src->m_contact[srcIndex];
				}
			}
		}
		m_count = newCount;
	}

	dInt32 m_count;
};

#endif
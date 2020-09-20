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

#include "dCoreStdafx.h"
#include "dString.h"
#include "dMemory.h"

//#define D_USE_POOL_BUKECT_ALLOCATOR
#define D_STRING_MEM_GRANULARITY		16
#define D_STRING_MEM_MAX_BUCKET_SIZE	256
#define D_STRING_MEM_BUCKETS			(D_STRING_MEM_MAX_BUCKET_SIZE / D_STRING_MEM_GRANULARITY)
#define D_DSTRING_ENTRIES_IN_FREELIST	32

class dString::dStringAllocator
{
	public:
	#ifdef D_USE_POOL_BUKECT_ALLOCATOR
		class dMemBucket
		{
			public:

			class dDataChunk
			{
				public: 
				dInt32 m_size;
				dInt32 m_count;
				dDataChunk* m_next;
				
			};

			dMemBucket()
				:m_freeListDataChunk(nullptr)
			{
			}

			~dMemBucket()
			{
			}

			void Prefetch (dInt32 chunckSize)
			{
				for (dInt32 i = 0; i < D_DSTRING_ENTRIES_IN_FREELIST; i ++) 
				{
					dDataChunk* const data = (dDataChunk*) dMemory::Malloc(chunckSize + sizeof (dInt32));
					data->m_count = i + 1; 
					data->m_size = chunckSize;
					data->m_next = m_freeListDataChunk; 
					m_freeListDataChunk = data;
				}
			}

			void Flush ()
			{
				for (dInt32 i = 0; m_freeListDataChunk && (i < D_DSTRING_ENTRIES_IN_FREELIST); i ++) 
				{
					dDataChunk* const ptr = m_freeListDataChunk;
					m_freeListDataChunk = m_freeListDataChunk->m_next;
					dMemory::Free (ptr);
				}
			}

			char* Alloc(dInt32 size)
			{
				dAssert (size < 1024 * 4);
				if (!m_freeListDataChunk) 
				{
					Prefetch (size);
				}
				dDataChunk* const data = m_freeListDataChunk;
				dAssert (size == data->m_size);
				m_freeListDataChunk = m_freeListDataChunk->m_next;
				return ((char*)data) + sizeof (dInt32);
			}

			void Free(char * const ptr)
			{
				char* const realPtr = ptr - sizeof (dInt32);
				dMemBucket::dDataChunk* const dataChunck = (dMemBucket::dDataChunk*) (realPtr);

				dataChunck->m_count = m_freeListDataChunk ? m_freeListDataChunk->m_count + 1 : 1;
				dataChunck->m_next = m_freeListDataChunk;
				m_freeListDataChunk = dataChunck;
				if (dataChunck->m_count >= 2 * D_DSTRING_ENTRIES_IN_FREELIST) 
				{
					Flush();
				}
			}

			dDataChunk* m_freeListDataChunk;
		};

		dStringAllocator()
		{
			for (dInt32 i = 0; i < dInt32 (sizeof (m_buckects) / sizeof (m_buckects[0])); i ++) 
			{
				m_buckects[i].Prefetch ((i + 1)* D_STRING_MEM_GRANULARITY);
			}
		}

		~dStringAllocator()
		{
			for (dInt32 i = 0; i < dInt32 (sizeof (m_buckects) / sizeof (m_buckects[0])); i ++) 
			{
				m_buckects[i].Flush();
			}
		}

		char* Alloc(dInt32 size)
		{
			dAssert (size >= 1);
			if (size <= D_STRING_MEM_MAX_BUCKET_SIZE) 
			{
				dInt32 buckectEntry = (size - 1) / D_STRING_MEM_GRANULARITY;
				dInt32 buckectSize = (buckectEntry + 1) * D_STRING_MEM_GRANULARITY;
				return m_buckects[buckectEntry].Alloc(buckectSize);
			}
			dMemBucket::dDataChunk* const ptr = (dMemBucket::dDataChunk*) dMemory::Malloc (size + sizeof (dInt32));
			ptr->m_size = size;
			return ((char*)ptr) + sizeof (dInt32);
		}

		void Free(char* const ptr)
		{
			char* const realPtr = ptr-sizeof (dInt32);
			dMemBucket::dDataChunk* const dataChunck = (dMemBucket::dDataChunk*) (realPtr);
			if (dataChunck->m_size <= D_STRING_MEM_MAX_BUCKET_SIZE) 
			{
				dInt32 buckectEntry = dataChunck->m_size / D_STRING_MEM_GRANULARITY - 1;
				m_buckects[buckectEntry].Free(ptr);
			} 
			else 
			{
				void* const ptr1 = ((char*)ptr) - sizeof (dInt32);
				dMemory::Free (ptr1);
			}
		}

		dMemBucket m_buckects [D_STRING_MEM_BUCKETS];

	#else 
		char* Alloc(dInt32 size)
		{
			return (char*) dMemory::Malloc (size);
		}

		void Free(char* const ptr)
		{
			dMemory::Free (ptr);
		}
	#endif
};

dString::dString ()
	:m_string(nullptr)
	,m_size(0)
	,m_capacity(0)
{
}

dString::dString (const dString& src)
	:m_string(nullptr)
	,m_size(0)
	,m_capacity(0)
{
	if (src.m_string) 
	{
		m_size = src.m_size;
		m_capacity = m_size + 1;

		m_string = AllocMem (src.m_size + 1);
		CopyData (m_string, src.m_string, src.m_size + 1);
		m_string[m_size] = 0;
	}
}

dString::dString (const char* const data)
	:m_string(nullptr)
	,m_size(0)
	,m_capacity(0)
{
	if (data) 
	{
		m_size = CalculateSize (data);
		m_capacity = m_size + 1; 

		m_string = AllocMem (m_size + 1);
		CopyData (m_string, data, m_size + 1);
		m_string[m_size] = 0;
	}
}

dString::dString (const char* const data, dInt32 maxSize)
	:m_string(nullptr)
	,m_size(0)
	,m_capacity(0)
{
	if (data) 
	{
		m_size = dMin (CalculateSize (data), maxSize);
		m_capacity = m_size + 1; 
		m_string = AllocMem (m_size + 1);
		CopyData (m_string, data, m_size + 1);
		m_string[m_size] = 0;
	}
}

dString::dString (const dString& src, const char* const concatenate, dInt32 concatenateSize)
	:m_string(nullptr)
	,m_size(0)
	,m_capacity(0)
{
	m_string = AllocMem (src.m_size + concatenateSize + 1);
	memcpy (m_string, src.m_string, src.m_size);
	memcpy (&m_string[src.m_size], concatenate, concatenateSize);
	m_size = src.m_size + concatenateSize;
	m_string[m_size] = 0;
	m_capacity = m_size + 1;
}

dString::dString (char chr)
	:m_string(nullptr)
	,m_size(0)
	,m_capacity(0)
{
	m_string = AllocMem (2);
	m_string[0] = chr;
	m_string[1] = 0;
	m_size = 1;
	m_capacity = m_size + 1;
}

dString::dString (dInt32 val)
	:m_string(nullptr)
	,m_size(0)
	,m_capacity(0)
{
	char tmp[256];

	dInt32 count = 0;
	unsigned mag = abs (val);
	do 
	{
		unsigned digit = mag % 10;
		mag /= 10;  
		tmp[count] = '0' + char(digit);
		count ++;
	} while (mag > 0);

	dInt32 offset = (val >= 0) ? 0: 1;
	m_string = AllocMem (count + offset + 1);
	if (offset) {
		m_string[0] = '-';
	}
	for (dInt32 i = 0; i < count; i ++) {
		m_string[i + offset] = tmp[count - i - 1];
	}

	m_string[count + offset] = 0;
	m_size = count + offset;
	m_capacity = m_size + 1;
}

dString::dString (dUnsigned64 input)
	:m_string(nullptr)
	,m_size(0)
	,m_capacity(0)
{
	char tmp[256];

	dInt32 count = 0;
	dInt64 val = input;
	dUnsigned64 mag = (val > 0ll) ? val : -val;
	do {
		dUnsigned64 digit = mag % 10ll;
		mag /= 10ll;  
		tmp[count] = '0' + char(digit);
		count ++;
	} while (mag > 0);

	dInt32 offset = (val >= 0ll) ? 0: 1;
	m_string = AllocMem (count + offset + 1);
	if (offset) 
	{
		m_string[0] = '-';
	}
	for (dInt32 i = 0; i < count; i ++) 
	{
		m_string[i + offset] = tmp[count - i - 1];
	}

	m_string[count + offset] = 0;
	m_size = count + offset;
	m_capacity = m_size + 1;
}

dString::~dString ()
{
	Clear();
}

void dString::Clear()
{
	if (m_capacity && m_string) 
	{
		FreeMem(m_string);
	}
	m_size = 0;
	m_capacity = 0;
	m_string = nullptr;
}

void dString::Empty()
{
	m_size = 0;
	m_string[0] = 0;
}

void dString::LoadFile (FILE* const file)
{
	Clear();
	dInt32 size = 0;
	fseek (file, 0, SEEK_SET);
	for (;!feof(file); size ++) 
	{
		fgetc (file);
	}
	fseek (file, 0, SEEK_SET);
	Expand (size);
	size_t ret = fread (m_string, 1, size, file);
	ret = 0;
	m_string[size-1] = 0;
	m_size = size-1;
	m_capacity = m_size + 1;
}

void dString::operator+= (const char* const src)
{
	char* const oldData = m_string;
	dInt32 size = CalculateSize (src);
	if ((m_size + size + 1) > m_capacity) 
	{
		dInt32 newCapacity = dMax (m_capacity - 1, 1);
		while (newCapacity < (m_size + size)) 
		{
			newCapacity *= 2;
		}
		m_string = AllocMem (newCapacity + 1);
		memcpy (m_string, oldData, m_size);
		memcpy (&m_string[m_size], src, size);
		m_size = m_size + size;
		m_string[m_size] = 0;
		m_capacity = newCapacity + 1;
		FreeMem(oldData);
	} 
	else 
	{
		memcpy (&m_string[m_size], src, size);
		m_size = m_size + size;
		m_string[m_size] = 0;
	}
}

dInt32 dString::ToInteger() const
{
	dInt32 value = 0;
	if (m_size) 
	{
		dInt32 base = (m_string[0] == '-') ? 1 : 0;
		for (dInt32 i = base; i < m_size; i ++) 
		{
			char ch = m_string[i]; 		
			if ((ch >= '0') && (ch <= '9')) 
			{
				value = value * 10 + ch - '0';
			} 
			else 
			{
				break;
			}
		}
		value *= base ? -1 : 1;
	}
	return value;
}


dUnsigned64 dString::ToInteger64() const
{
	dUnsigned64 value = 0;
	if (m_size) 
	{
		dInt32 base = (m_string[0] == '-') ? 1 : 0;
		for (dInt32 i = base; i < m_size; i ++) 
		{
			char ch = m_string[i]; 		
			if ((ch >= '0') && (ch <= '9')) 
			{
				value = value * 10ll + ch - '0';
			} 
			else 
			{
				break;
			}
		}
		value *= base ? -1 : 1;
	}
	return value;
}

dFloat64 dString::ToFloat() const
{
	dFloat64 value = 0.0;
	dFloat64 power = 1.0;
	dFloat64 decimalBase = 1.0;
	if (m_size) 
	{
		dInt32 base = (m_string[0] == '-') ? 1 : 0;
		for (dInt32 i = base; i < m_size; i ++) 
		{
			char ch = m_string[i]; 		
			if ((ch >= '0') && (ch <= '9')) 
			{
				value = value * 10ll + ch - '0';
				power *= decimalBase;
			} 
			else if (ch == '.') 
			{
				decimalBase = 10.0;
			} 
			else 
			{
				break;
			}
		}
		value *= base ? -1 : 1;
	}
	value /= power;

	return value;
}

dString& dString::operator= (const dString& src)
{
	if (m_capacity && m_string) 
	{
		FreeMem (m_string);
	}
	m_string = nullptr;
	m_capacity = 0;
	m_size = src.m_size;
	if (src.m_string) 
	{
		m_capacity = src.m_size + 1;
		m_string = AllocMem (src.m_size + 1);
		CopyData (m_string, src.m_string, src.m_size + 1);
	}
	return *this;
}

dInt32 dString::CalculateSize (const char* const data) const
{
	dInt32 size = 0;
	if (data) 
	{
		for (dInt32 i = 0; data[i]; i ++) 
		{
			size ++;
		}
	}
	return size;
}

void dString::ToUpper()
{
	if (m_string) 
	{
		for (char * cp = m_string; *cp; ++cp) 
		{
			if ((*cp >= 'a') && (*cp <= 'z'))
			{
				*cp += 'A' - 'a';
			}
		}
	}
}

void dString::ToLower()
{
	if (m_string) 
	{
		for (char * cp = m_string; *cp; ++cp) 
		{
			if ((*cp >= 'A') && (*cp <= 'Z'))
			{
				*cp += 'a' - 'A';
			}
		}
	}
}

dInt32 dString::Find (char ch, dInt32 from) const
{
	for (dInt32 i = from; i < m_size; i ++) 
	{
		if (m_string[i] == ch) 
		{
			return i;
		}
	}
	return -1;
}

//dInt32 dString::Find (const dString& subStream, dInt32 from) const
dInt32 dString::Find (const char* const subString, dInt32 subStringLength, dInt32 from, dInt32 lenght) const
{
	dAssert (from >= 0);
	//dAssert (subStream.m_size >= 0);
	dAssert (subStringLength >= 1);

	dInt32 location = -1;
	if (m_size) 
	{
		const dInt32 str2Size = dMin (subStringLength, lenght);
		if (str2Size == 1) 
		{
			char ch = subString[0];
			const char* const ptr1 = m_string;
			for (dInt32 i = 0; i < m_size; i ++) 
			{
				if (ch == ptr1[i]) 
				{
					return i;
				}
			}
		} 
		else if ((str2Size < 4) || (m_size < 64)) 
		{
			const dInt32 size = m_size - str2Size;
			for (dInt32 j = from; j <= size; j ++) 
			{
				const char* const ptr1 = &m_string[j];
				dInt32 i = 0;
				while (subString[i] && (ptr1[i] == subString[i])) 
				{
					i ++;
				}
				if (!subString[i]) 
				{
					return j;
				}
			}
		} 
		else 
		{
			// for large strings smart search
			dInt16 frequency[256];
			memset (frequency, -1, sizeof (frequency));
			for (dInt32 i = 0; i < str2Size; i ++) 
			{
				frequency[dInt32 (subString[i])] = dInt16(i);
			}

			dInt32 j = from;
			const dInt32 size = m_size - str2Size;
			while (j <= size) 
			{
				const char* const ptr1 = &m_string[j];
				dInt32 i = str2Size - 1;
				while ((i >= 0) && (ptr1[i] == subString[i])) 
				{
					i --;
				}
				if (i < 0) 
				{
					return j;
				}
				j += dMax(i - frequency[dInt32 (ptr1[i])], 1);
			}
		}
	}
	return location;
}


void dString::Replace (dInt32 start, dInt32 size, const char* const str, dInt32 strSize)
{
	char* const oldData = m_string;
	m_string = AllocMem (m_size - size + strSize + 1);
	memcpy (m_string, oldData, start);
	memcpy (&m_string[start], str, strSize);
	memcpy (&m_string[start + strSize], &oldData[start + size], m_size - (start + size));
	m_size = m_size - size + strSize;
	m_capacity = m_size + 1;
	m_string[m_size] = 0;
	FreeMem(oldData);
}

void dString::Expand (dInt32 size)
{
	char* const oldData = m_string;
	m_string = AllocMem (m_size + size + 1);
	
	if (m_capacity) 
	{
		memcpy (m_string, oldData, m_size);
		FreeMem(oldData);
	}
	m_string[m_size] = 0;
	m_capacity = m_size + size + 1;
}

dString::dStringAllocator& dString::GetAllocator() const
{
	static dStringAllocator allocator;
	return allocator;
}

char* dString::AllocMem(dInt32 size)
{
	return GetAllocator().Alloc(size);
}

void dString::FreeMem (char* const ptr)
{
	if (ptr) 
	{
		GetAllocator().Free(ptr);
	}
}

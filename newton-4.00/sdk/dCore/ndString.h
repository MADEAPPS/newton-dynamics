/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


#ifndef __NDSTRING_H_
#define __NDSTRING_H_

#include "ndCoreStdafx.h"
#include "ndClassAlloc.h"

class ndString: public ndClassAlloc
{
	class ndStringAllocator;
	public:
	D_CORE_API ndString ();
	D_CORE_API ndString (char chr);
	D_CORE_API ndString (const ndString& src);
	D_CORE_API ndString (const char* const data);
	D_CORE_API ndString (const char* const data, ndInt32 maxSize);
	D_CORE_API ndString (ndInt32 val);
	D_CORE_API ndString (ndUnsigned64 val);
	D_CORE_API ~ndString ();

	char& operator[] (ndInt32 index);
	char operator[] (ndInt32 index) const;
	
	D_CORE_API ndString& operator= (const ndString& src);
	bool operator== (const ndString& src) const;
	bool operator!= (const ndString& src) const;
	bool operator< (const ndString& src) const;
	bool operator> (const ndString& src) const;
	bool operator<= (const ndString& src) const;
	bool operator>= (const ndString& src) const;

	D_CORE_API void operator+= (const char* const src);
	void operator+= (const ndString& src);

	ndString operator+ (const char* const src) const;
	ndString operator+ (const ndString& src) const;

	D_CORE_API ndInt32 Find (char ch, ndInt32 from = 0) const;
	ndInt32 Find (const ndString& subString, ndInt32 from = 0) const;
	D_CORE_API ndInt32 Find (const char* const subString, ndInt32 from = 0, ndInt32 lenght = 0x7ffffff) const;

	D_CORE_API void Replace (ndInt32 start, ndInt32 size, const char* const str, ndInt32 strSize);
	void Replace (ndInt32 start, ndInt32 size, const ndString& str);

	void Clear();
	void Empty();

	D_CORE_API void ToUpper();
	D_CORE_API void ToLower();
	D_CORE_API ndInt32 ToInteger() const;
	D_CORE_API ndFloat64 ToFloat() const;
	D_CORE_API ndUnsigned64 ToInteger64() const;

	ndInt32 Size() const;
	ndInt32 Capacity() const;
	D_CORE_API void Expand (ndInt32 size);

	D_CORE_API void LoadFile (FILE* const file);
	ndString SubString(ndInt32 start = 0, ndInt32 size = 0x7fffffff) const;

	const char* GetStr () const;

	private:
	D_CORE_API ndInt32 CalculateSize (const char* const data) const;
	ndInt32 Compare (const char* const str0, const char* const str1) const;
	void CopyData (char* const dst, const char* const src, ndInt32 size) const;

	D_CORE_API ndInt32 Find (const char* const subString, ndInt32 stringSize, ndInt32 from, ndInt32 lenght) const;

	protected:
	char* AllocMem(ndInt32 size);
	void FreeMem (char* const ptr);
	D_CORE_API ndString (const ndString& src, const char* const concatenate, ndInt32 maxSize);
	
	char* m_string;
	ndInt32 m_size;
	ndInt32 m_capacity;

	private:
	ndStringAllocator& GetAllocator() const;
};

inline char& ndString::operator[] (ndInt32 index)
{
	dAssert (m_string);
	dAssert (index >= 0);
	dAssert (index < m_size);
	return m_string[index];
}

inline char ndString::operator[] (ndInt32 index) const
{
	dAssert (m_string);
	dAssert (index >= 0);
	dAssert (index < m_size);
	return m_string[index];
}

inline const char* ndString::GetStr () const
{
	return m_string;
}

inline ndInt32 ndString::Size() const
{
	return m_size;
}

inline ndInt32 ndString::Find (const char* const subString, ndInt32 from, ndInt32 lenght) const
{
	return Find (subString, CalculateSize(subString), from, lenght);
}

inline ndInt32 ndString::Find (const ndString& subStream, ndInt32 from) const
{
	dAssert (subStream.m_string);
	return Find (subStream.m_string, subStream.m_size, from, subStream.m_size);
}

inline void ndString::Replace (ndInt32 start, ndInt32 size, const ndString& str)
{
	Replace(start, size, str.m_string, str.m_size);
}

inline void ndString::operator+= (const ndString& src)
{
	*this += src.m_string;
}

inline ndString ndString::operator+ (const ndString& src) const
{
	return ndString (*this, src.m_string, src.m_size);
}

inline ndString ndString::operator+ (const char* const copy) const
{
	return ndString (*this, copy, CalculateSize (copy));
}


inline ndInt32 ndString::Capacity() const
{
	return m_capacity;
}

inline void ndString::CopyData (char* const dst, const char* const src, ndInt32 size) const
{
	dAssert (dst);
	dAssert (src);
	memcpy (dst, src, size);
}

inline ndInt32 ndString::Compare (const char* const str0, const char* const str1) const
{
	dAssert (str0);
	dAssert (str1);
	return strcmp (str0, str1);
}


inline bool ndString::operator== (const ndString& src) const
{
	return Compare (m_string, src.m_string) == 0;
}

inline bool ndString::operator!= (const ndString& src) const
{
	return Compare (m_string, src.m_string) != 0;
}


inline bool ndString::operator< (const ndString& src) const
{
	return Compare (m_string, src.m_string) < 0;
}

inline bool ndString::operator> (const ndString& src) const
{
	return Compare (m_string, src.m_string) > 0;
}

inline bool ndString::operator<= (const ndString& src) const
{
	return Compare (m_string, src.m_string) <= 0;
}

inline bool ndString::operator>= (const ndString& src) const
{
	return Compare (m_string, src.m_string) >= 0;
}

inline ndString ndString::SubString(ndInt32 start, ndInt32 size) const
{
	dAssert (m_string);
	return ndString (&m_string[start], size);
}


#endif




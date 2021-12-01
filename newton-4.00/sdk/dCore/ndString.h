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

class dString: public dClassAlloc
{
	class dStringAllocator;
	public:
	D_CORE_API dString ();
	D_CORE_API dString (char chr);
	D_CORE_API dString (const dString& src);
	D_CORE_API dString (const char* const data);
	D_CORE_API dString (const char* const data, dInt32 maxSize);
	D_CORE_API dString (dInt32 val);
	D_CORE_API dString (dUnsigned64 val);
	D_CORE_API ~dString ();

	char& operator[] (dInt32 index);
	char operator[] (dInt32 index) const;
	
	D_CORE_API dString& operator= (const dString& src);
	bool operator== (const dString& src) const;
	bool operator!= (const dString& src) const;
	bool operator< (const dString& src) const;
	bool operator> (const dString& src) const;
	bool operator<= (const dString& src) const;
	bool operator>= (const dString& src) const;

	D_CORE_API void operator+= (const char* const src);
	void operator+= (const dString& src);

	dString operator+ (const char* const src) const;
	dString operator+ (const dString& src) const;

	D_CORE_API dInt32 Find (char ch, dInt32 from = 0) const;
	dInt32 Find (const dString& subString, dInt32 from = 0) const;
	D_CORE_API dInt32 Find (const char* const subString, dInt32 from = 0, dInt32 lenght = 0x7ffffff) const;

	D_CORE_API void Replace (dInt32 start, dInt32 size, const char* const str, dInt32 strSize);
	void Replace (dInt32 start, dInt32 size, const dString& str);

	void Clear();
	void Empty();

	D_CORE_API void ToUpper();
	D_CORE_API void ToLower();
	D_CORE_API dInt32 ToInteger() const;
	D_CORE_API dFloat64 ToFloat() const;
	D_CORE_API dUnsigned64 ToInteger64() const;

	dInt32 Size() const;
	dInt32 Capacity() const;
	D_CORE_API void Expand (dInt32 size);

	D_CORE_API void LoadFile (FILE* const file);
	dString SubString(dInt32 start = 0, dInt32 size = 0x7fffffff) const;

	const char* GetStr () const;

	private:
	D_CORE_API dInt32 CalculateSize (const char* const data) const;
	dInt32 Compare (const char* const str0, const char* const str1) const;
	void CopyData (char* const dst, const char* const src, dInt32 size) const;

	D_CORE_API dInt32 Find (const char* const subString, dInt32 stringSize, dInt32 from, dInt32 lenght) const;

	protected:
	char* AllocMem(dInt32 size);
	void FreeMem (char* const ptr);
	D_CORE_API dString (const dString& src, const char* const concatenate, dInt32 maxSize);
	
	char* m_string;
	dInt32 m_size;
	dInt32 m_capacity;

	private:
	dStringAllocator& GetAllocator() const;
};

inline char& dString::operator[] (dInt32 index)
{
	dAssert (m_string);
	dAssert (index >= 0);
	dAssert (index < m_size);
	return m_string[index];
}

inline char dString::operator[] (dInt32 index) const
{
	dAssert (m_string);
	dAssert (index >= 0);
	dAssert (index < m_size);
	return m_string[index];
}

inline const char* dString::GetStr () const
{
	return m_string;
}

inline dInt32 dString::Size() const
{
	return m_size;
}

inline dInt32 dString::Find (const char* const subString, dInt32 from, dInt32 lenght) const
{
	return Find (subString, CalculateSize(subString), from, lenght);
}

inline dInt32 dString::Find (const dString& subStream, dInt32 from) const
{
	dAssert (subStream.m_string);
	return Find (subStream.m_string, subStream.m_size, from, subStream.m_size);
}

inline void dString::Replace (dInt32 start, dInt32 size, const dString& str)
{
	Replace(start, size, str.m_string, str.m_size);
}

inline void dString::operator+= (const dString& src)
{
	*this += src.m_string;
}

inline dString dString::operator+ (const dString& src) const
{
	return dString (*this, src.m_string, src.m_size);
}

inline dString dString::operator+ (const char* const copy) const
{
	return dString (*this, copy, CalculateSize (copy));
}


inline dInt32 dString::Capacity() const
{
	return m_capacity;
}

inline void dString::CopyData (char* const dst, const char* const src, dInt32 size) const
{
	dAssert (dst);
	dAssert (src);
	memcpy (dst, src, size);
}

inline dInt32 dString::Compare (const char* const str0, const char* const str1) const
{
	dAssert (str0);
	dAssert (str1);
	return strcmp (str0, str1);
}


inline bool dString::operator== (const dString& src) const
{
	return Compare (m_string, src.m_string) == 0;
}

inline bool dString::operator!= (const dString& src) const
{
	return Compare (m_string, src.m_string) != 0;
}


inline bool dString::operator< (const dString& src) const
{
	return Compare (m_string, src.m_string) < 0;
}

inline bool dString::operator> (const dString& src) const
{
	return Compare (m_string, src.m_string) > 0;
}

inline bool dString::operator<= (const dString& src) const
{
	return Compare (m_string, src.m_string) <= 0;
}

inline bool dString::operator>= (const dString& src) const
{
	return Compare (m_string, src.m_string) >= 0;
}

inline dString dString::SubString(dInt32 start, dInt32 size) const
{
	dAssert (m_string);
	return dString (&m_string[start], size);
}


#endif




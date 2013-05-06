/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


#ifndef __DSTRING_H_
#define __DSTRING_H_

#include "dContainersStdAfx.h"



class dString
{
	class dStringAllocator;
	public:
	dString ();
	dString (char chr);
	dString (const dString& src);
	dString (const char* const data);
	dString (const char* const data, int maxSize);
	dString (int val);
	dString (long long val);
	~dString ();

	char& operator[] (int index);
	char operator[] (int index) const;
	
	dString& operator= (const dString& src);
	bool operator== (const dString& src) const;
	bool operator!= (const dString& src) const;
	bool operator< (const dString& src) const;
	bool operator> (const dString& src) const;
	bool operator<= (const dString& src) const;
	bool operator>= (const dString& src) const;

	void operator+= (const char* const src);
	void operator+= (const dString& src);

	dString operator+ (const char* const src) const;
	dString operator+ (const dString& src) const;

	int Find (char ch, int from = 0) const;
	int Find (const dString& subString, int from = 0) const;
	int Find (const char* const subString, int from = 0, int lenght = 0x7ffffff) const;

	void Replace (int start, int size, const char* const str, int strSize);
	void Replace (int start, int size, const dString& str);
	void Empty();

	void ToUpper();
	void ToLower();
	int ToInteger() const;
	long long ToInteger64() const;

	int Size() const;
	int Capacity() const;
	void Expand (int size);

	void LoadFile (FILE* const file);
	dString SubString(int start = 0, int size = 0x7fffffff) const;

	const char* GetStr () const;

	private:
	int CalculateSize (const char* const data) const;
	int Compare (const char* const str0, const char* const str1) const;
	void CopyData (char* const dst, const char* const src, int size) const;

	int Find (const char* const subString, int stringSize, int from, int lenght) const;

	char* AllocMem(int size);
	void FreeMem (char* const ptr);

	protected:
	dString (const dString& src, const char* const concatenate, int maxSize);
	
	char* m_string;
	int m_size;
	int m_capacity;

	private:
	dStringAllocator& GetAllocator() const;
};


inline char& dString::operator[] (int index)
{
	dAssert (m_string);
	dAssert (index >= 0);
	dAssert (index < m_size);
	return m_string[index];
}

inline char dString::operator[] (int index) const
{
	dAssert (m_string);
	dAssert (index >= 0);
	dAssert (index < m_size);
	return m_string[index];
}


inline int dString::Find (const char* const subString, int from, int lenght) const
{
	return Find (subString, CalculateSize(subString), from, lenght);
}

inline int dString::Find (const dString& subStream, int from) const
{
	dAssert (subStream.m_string);
	return Find (subStream.m_string, subStream.m_size, from, subStream.m_size);
}

inline void dString::Replace (int start, int size, const dString& str)
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


#endif



/****************************************************************************************
 
   Copyright (C) 2012 Autodesk, Inc.
   All rights reserved.
 
   Use of this software is subject to the terms of the Autodesk license agreement
   provided at the time of installation or download, or which otherwise accompanies
   this software in either electronic or hard copy form.
 
****************************************************************************************/

//! \file fbxstring.h
#ifndef _FBXSDK_CORE_BASE_STRING_H_
#define _FBXSDK_CORE_BASE_STRING_H_

#include <fbxsdk/fbxsdk_def.h>

#if defined(FBXSDK_COMPILER_MSC)
	#if defined(FBXSDK_ARCH_32)
		#if _MSC_VER <= 1600
			#define SIZE_OF_STDSTRING 32
		#elif _MSC_VER == 1700
			#define SIZE_OF_STDSTRING 28
		#endif
	#else
		#if _MSC_VER <= 1600
			#define SIZE_OF_STDSTRING 48
		#elif _MSC_VER == 1700
			#define SIZE_OF_STDSTRING 40
		#endif
	#endif
#elif defined(FBXSDK_COMPILER_GNU)
	#if defined(FBXSDK_ARCH_32)
		#define SIZE_OF_STDSTRING 4
	#else
		#define SIZE_OF_STDSTRING 8
	#endif
#endif

#if !defined(SIZE_OF_STDSTRING)
    #error Unsupported platform
#endif

#define FBXSDK_STRING_OPAQUE_OBJECT_HEADER void* mOpaqueChunk[SIZE_OF_STDSTRING/sizeof(void*)]
#ifndef FBXSDK_STRING_OPAQUE_OBJECT
    #define FBXSDK_STRING_OPAQUE_OBJECT FBXSDK_STRING_OPAQUE_OBJECT_HEADER
#endif

#include <fbxsdk/fbxsdk_nsbegin.h>

//Conversions between WideChar, UTF8 and ANSI

/** Convert string from UTF8 to wide-char
  * \param inUTF8               input string
  * \param pOutWideChar         output string
  * \param pOutWideCharSize     size of the allocated output string buffer
  * \remark Output buffer should be release by caller
  */
FBXSDK_DLL void FbxUTF8ToWC(const char* pInUTF8, wchar_t*& pOutWideChar, size_t* pOutWideCharSize = NULL);

/** Convert string from wide-char to UTF8
  * \param inWideChar            input string
  * \param pOutUTF8              output string
  * \param pOutUTF8Size          size of the allocated output string buffer
  * \remark Output buffer should be release by caller
  */
FBXSDK_DLL void FbxWCToUTF8(const wchar_t* pInWideChar, char*& pOutUTF8, size_t* pOutUTF8Size = NULL);

#if defined(FBXSDK_ENV_WIN)
    /** Convert string from wide-char to ANSI
      * \param pInWideChar          input string
      * \param pOutANSI             output string
      * \param pOutANSISize         size of the allocated output string buffer
      * \remark Output buffer should be release by caller
      */
    FBXSDK_DLL void FbxWCToAnsi(const wchar_t* pInWideChar, char*& pOutANSI, size_t* pOutANSISize = NULL);

    /** Convert string from ANSI to wide-char
      * \param inANSI               input string
      * \param pOutWideChar         output string
      * \param pOutWideCharSize     size of the allocated output string buffer
      * \remark Output buffer should be release by caller
      */
    FBXSDK_DLL void FbxAnsiToWC(const char* pInANSI, wchar_t*& pOutWideChar, size_t* pOutWideCharSize = NULL);

    /** Convert string from ANSI to UTF8
      * \param inANSI               input string
      * \param outUTF8              output string
      * \param pOutUTF8Size         size of the allocated output string buffer
      * \remark Output buffer should be release by caller
      */
    FBXSDK_DLL void FbxAnsiToUTF8(const char* pInANSI, char*& pOutUTF8, size_t* pOutUTF8Size = NULL);

    /** Convert string from UTF8 to ANSI
      * \param pInUTF8              input string
      * \param outANSI              output string
      * \param pOutANSISize         size of the allocated output string buffer
      * \remark Output buffer should be release by caller
      */
    FBXSDK_DLL void FbxUTF8ToAnsi(const char* pInUTF8, char*& pOutANSI, size_t* pOutANSISize = NULL);
#endif

/** Utility class to manipulate strings.
* \nosubgrouping */
class FBXSDK_DLL FbxString
{
public:
	/**
	* \name Constructors and Destructor
	*/
	//@{
		//! Default constructor.
		FbxString();

		/** Copy constructor.
		* \param pStr The FbxString to be copied. */
		FbxString(const FbxString& pStr);

		/** String constructor.
		* \param pStr The string used to construct FbxString. */
		FbxString(const char* pStr);

		/** Character constructor.
		* \param pChar The character used to construct FbxString.
		* \param pNbRepeat The number of times to repeat the character. Default value is 1 */
		FbxString(char pChar, size_t pNbRepeat=1);

		/** String constructor with maximum length.
		* \param pCharPtr The string used to construct FbxString. 
		* \param pLength  Maximum length. */
		FbxString(const char* pCharPtr, size_t pLength);

		/** Integer constructor.
		* \param pValue The int value used to construct FbxString. */
		FbxString(const int pValue);

		/** Float constructor.
		* \param pValue The float value used to construct FbxString. */
		FbxString(const float pValue);

		/** Double constructor.
		* \param pValue The double value used to construct FbxString. */
		FbxString(const double pValue);

		//! Destructor.
		~FbxString();
	//@}

	/**
	* \name Buffer Access and Validation
	*/
	//@{
		//! Get string length like "C" strlen().
		size_t GetLen() const;

		//! Return \c true if string length equal zero.
		bool IsEmpty() const;

		//! Discard the content of the string.
		FbxString& Empty();

		/** Access by reference.
		* \param pIndex   The index.
		* \return The reference of the char at pIndex. */
		char& operator[](int pIndex);

		/** Access by copy.
		* \param pIndex   The index.
		* \return The char at pIndex. */
		char operator[](int pIndex) const;

		//! Non-const buffer access.
		char* Buffer();

		//! Const buffer access.
		const char* Buffer()const;
	//@}

	/**
	* \name String Operations
	*/
	//@{
		/** FbxString assignment operator.
		* \param pStr The FbxString to be assigned. */
		const FbxString& operator=(const FbxString& pStr);

		/** Character assignment operator.
		* \param pChar The character to be assigned. */
		const FbxString& operator=(char pChar);

		/** String assignment operator.
		* \param pStr The string to be assigned. */
		const FbxString& operator=(const char* pStr);

		/** Int assignment operator.
		* \param pValue The int value to be assigned. */
		const FbxString& operator=(int pValue);

		/** Float assignment operator.
		* \param pValue The float value to be assigned. */
		const FbxString& operator=(float pValue);

		/** Double assignment operator.
		* \param pValue The double value to be assigned. */
		const FbxString& operator=(double pValue);

		/** FbxString append.
		* \param pKStr  The FbxString to be appended. */
		const FbxString& operator+=(const FbxString& pKStr);

		/** Character append.
		* \param pChar  The character to be appended. */
		const FbxString& operator+=(char pChar);

		/** String append.
		* \param pStr    The string to be appended. */
		const FbxString& operator+=(const char* pStr);

		/** Integer append.
		* \param pValue  The int value to be appended. */
		const FbxString& operator+=(int pValue);

		/** Float append.
		* \param pValue  The float value to be appended. */
		const FbxString& operator+=(float pValue);

		/** Double append.
		* \param pValue  The double value to be appended. */
		const FbxString& operator+=(double pValue);

		/** Equality operator.
		* \param pStr    The FbxString to be compared. */
		bool operator== (const FbxString& pStr) const;

		/** Inequality operator.
		* \param pStr    The FbxString to be compared. */
		bool operator!= (const FbxString& pStr) const;

		/** Inferior to operator.
		* \param pStr    The FbxString to be compared. */
		bool operator< (const FbxString& pStr) const;

		/** Inferior or equal to operator.
		* \param pStr    The FbxString to be compared. */
		bool operator<= (const FbxString& pStr) const;

		/** Superior or equal to operator.
		* \param pStr    The FbxString to be compared. */
		bool operator>= (const FbxString& pStr) const;

		/** Superior to operator.
		* \param pStr    The FbxString to be compared. */
		bool operator> (const FbxString& pStr) const;

		/** Equality operator.
		* \param pStr    The string to be compared. */
		bool operator== (const char* pStr) const;

		/** Inequality operator.
		* \param pStr    The string to be compared. */
		bool operator!= (const char* pStr) const;

		/** Inferior to operator.
		* \param pStr    The string to be compared. */
		bool operator< (const char* pStr) const;

		/** Inferior or equal to operator.
		* \param pStr    The string to be compared. */
		bool operator<= (const char* pStr) const;

		/** Superior or equal to operator.
		* \param pStr    The string to be compared. */
		bool operator>= (const char* pStr) const;

		/** Superior to operator.
		* \param pStr    The string to be compared. */
		bool operator> (const char* pStr) const;

		/** FbxString concatenation.
		* \param pKStr1  FbxString 1 to be concatenated to FbxString 2.
		* \param pKStr2  FbxString 2 to be concatenated to FbxString 1 */
		friend FBXSDK_DLL FbxString operator+(const FbxString& pKStr1, const FbxString& pKStr2);

		/** Character concatenation.
		* \param pKStr  FbxString to be concatenated to Character.
		* \param pChar  Character to be concatenated to FbxString */
		friend FBXSDK_DLL FbxString operator+(const FbxString& pKStr, char pChar);

		/** String concatenation.
		* \param pKStr  FbxString to be concatenated to String.
		* \param pStr  String to be concatenated to FbxString */
		friend FBXSDK_DLL FbxString operator+(const FbxString& pKStr, const char* pStr);

		/** Integer concatenation.
		* \param pKStr  FbxString to be concatenated to Integer.
		* \param pValue  Integer to be concatenated to FbxString */
		friend FBXSDK_DLL FbxString operator+(const FbxString& pKStr, int pValue);

		/** Float concatenation.
		* \param pKStr  FbxString to be concatenated to Float.
		* \param pValue  Float to be concatenated to FbxString */
		friend FBXSDK_DLL FbxString operator+(const FbxString& pKStr, float pValue);

		/** Double concatenation.
		* \param pKStr  FbxString to be concatenated to Double.
		* \param pValue  Double to be concatenated to FbxString */
		friend FBXSDK_DLL FbxString operator+(const FbxString& pKStr, double pValue);

		//! Cast operator.
		operator const char*() const;

		/** String assignment function with maximum length.
		  * \param pStr The string to be assigned.
		  * \param pLength The maximum length of string to be assigned. */
		const FbxString& Copy(const char* pStr, size_t pLength);

		/** Append as "C" strncat().
		* \param pStr The string to be appended.
		* \param pLength The length of chars to be appended. */
		const FbxString& Append(const char* pStr, size_t pLength);

		/** Compare as "C" strcmp().
		* \param pStr    The string to be compared. */
		int Compare(const char* pStr) const;

		/** Compare as "C" stricmp().
		* \param pStr    The string to be compared. */
		int CompareNoCase(const char* pStr) const;

		/** Swap the contents of two strings.
		* \param pStr The FbxString to be swapped. */
		void Swap(FbxString& pStr);

		//! Uppercase conversion.
		FbxString Upper() const;

		//! Lowercase conversion.
		FbxString Lower() const;
	//@}

    /**
    * \name Substring Extraction
    */
    //@{
		/** Extract middle string for a given length.
		* \param pFirst The start index of FbxString to be extracted.
		* \param pCount The length of sub-string to be extracted. */
		FbxString Mid(size_t pFirst, size_t pCount) const;

		/** Extract middle string up to the end.
		* \param pFirst The start index of FbxString to be extracted. */
		FbxString Mid(size_t pFirst) const;

		/** Extract left string.
		* \param pCount The length of sub-string to be extracted. */
		FbxString Left(size_t pCount) const;

		/** Extract right string.
		* \param pCount The length of sub-string to be extracted. */
		FbxString Right(size_t pCount) const;
	//@}

	/**
	* \name Padding
	*/
	//@{
		/** \enum EPaddingType      Padding types.
		* - \e eRight
		* - \e eLeft
		* - \e eBoth */
		enum EPaddingType {eRight, eLeft, eBoth};

		/** Add padding characters.
		* \param pPadding The padding type.
		* \param pLen The length limit of FbxString after padding. 
		* \param pCar The character to be padded. */
		FbxString Pad(EPaddingType pPadding, size_t pLen, char pCar=' ') const;

		/** Remove padding characters.
		* \param pPadding The padding type. */
		FbxString UnPad(EPaddingType pPadding) const;
	//@}

	/**
	* \name Search
	*/
	//@{
		/** Look for a single character match, like "C" strchr().
		* \param pChar The character to look for.
		* \param pStartPosition  Start position to look for.
		* \return Index or -1 if not found. */
		int Find(char pChar, size_t pStartPosition=0) const;

		/** Look for a substring match, like "C" strstr().
		* \param pStrSub The substring to look for.
		* \param pStartPosition  Start position to look for.
		* \return Starting index or -1 if not found. */
		int Find(const char* pStrSub, size_t pStartPosition=0) const;

		/** Look for the last occurrence of character in string, like "C" strrchr().
		* \param pChar The character to look for.
		* \return Index or -1 if not found. */
		int ReverseFind(char pChar) const;

		/** Look for a single character match, like "C" strpbrk().
		* \param pStrCharSet The character set.
		* \param pStartPosition The start position.
		* \return Index or -1 if not found. */
		int FindOneOf(const char* pStrCharSet, size_t pStartPosition=0) const;

		/** Replace a substring.
		* \param pFind The substring to look for.
		* \param pReplaceBy The string to replace by.
		* \param pStartPosition The start position. 
		* \return \c true if substring found and replaced. */
		bool FindAndReplace(const char* pFind, const char* pReplaceBy, size_t pStartPosition=0);

		/** Replace all occurrence of a substring.
		* \param pFind The substring to look for.
		* \param pReplaceBy The string to replace by.
		* \return \c true if something got replaced. */
		bool ReplaceAll(const char* pFind, const char* pReplaceBy);

        /** Replace all occurrence of character to find by replacement character.
		* \param pFind The character to look for.
		* \param pReplaceBy The character to replace by.
		* \return \c true if character found and replaced. */
		bool ReplaceAll(char pFind, char pReplaceBy);
	//@}

	/**
	* \name Token Extraction
	*/
	//@{
		/** Get number of tokens.
		* \param pSpans The span
		* \return The number of tokens. */
		int GetTokenCount(const char* pSpans) const;

		/** Get token at given index.
		* \param pTokenIndex The token index.
		* \param pSpans The span */
		FbxString GetToken(int pTokenIndex, const char* pSpans) const;
	//@}

private:
    FBXSDK_STRING_OPAQUE_OBJECT;
};

FBXSDK_INCOMPATIBLE_WITH_ARRAY(FbxString);

//! FbxString concatenation.
FBXSDK_DLL FbxString operator+(const FbxString& pKStr1, const FbxString& pKStr2);

//! Character concatenation.
FBXSDK_DLL FbxString operator+(const FbxString& pKStr, char pChar);

//! String concatenation.
FBXSDK_DLL FbxString operator+(const FbxString& pKStr, const char* pStr);

//! Integer concatenation.
FBXSDK_DLL FbxString operator+(const FbxString& pKStr, int pValue);

//! Float concatenation.
FBXSDK_DLL FbxString operator+(const FbxString& pKStr, float pValue);

//! Double concatenation.
FBXSDK_DLL FbxString operator+(const FbxString& pKStr, double pValue);

/** Functor class to compare FbxString, and is suitable for use in FbxMap. */
class FbxStringCompare
{
public:
	/** Compare two KStrings.
	* \param pKeyA The first FbxString to compare.
	* \param pKeyB The second FbxString to compare.
	* \return -1 indicates pKeyA is greater than pKeyB, 1 indicates pKeyB is greater than pKeyA, zero indicates they are equal.
	*/
	inline int operator()(const FbxString& pKeyA, const FbxString& pKeyB) const
	{
		return (pKeyA < pKeyB) ? -1 : ((pKeyB < pKeyA) ? 1 : 0);
	}
};

/** Functor class to compare "C" strings. */
class FbxCharCompare
{
public:
	//! Compare two strings like "C" strcmp().
	inline int operator()(char const* pKeyA, char const* pKeyB) const
	{
		return strcmp(pKeyA, pKeyB);
	}
};

/** Remove the given char in the given string.
* \param pString The given string.
* \param lToRemove The given char ought to be removed.
* \remarks Strings used in this function are case-sensitive. */
inline void FbxRemoveChar(FbxString& pString, char pToRemove)
{
    int lPos = pString.ReverseFind(pToRemove);
    while( lPos >= 0 )
    {
        pString = pString.Left(lPos) + pString.Mid(lPos + 1);
        lPos = pString.ReverseFind(pToRemove);
    }
}

#include <fbxsdk/fbxsdk_nsend.h>

#endif /* _FBXSDK_CORE_BASE_STRING_H_ */

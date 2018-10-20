/****************************************************************************************
 
   Copyright (C) 2012 Autodesk, Inc.
   All rights reserved.
 
   Use of this software is subject to the terms of the Autodesk license agreement
   provided at the time of installation or download, or which otherwise accompanies
   this software in either electronic or hard copy form.
 
****************************************************************************************/

/** \file fbxstdcompliant.h
  * macros to properly support the CRT secure functions.
  *
  */
#ifndef _FBXSDK_CORE_ARCH_STDCOMPLIANT_H_
#define _FBXSDK_CORE_ARCH_STDCOMPLIANT_H_

#include <fbxsdk/fbxsdk_def.h>

#include <fbxsdk/fbxsdk_nsbegin.h>

#if defined(FORCE_UNSECURE_CALLS) || defined (FBXSDK_ENV_MAC) || defined (FBXSDK_ENV_LINUX)
	inline int FBXSDK_sprintf(char* buffer, size_t dummy, const char* format, ...) 
	{ 
		va_list vl;
		va_start( vl, format);
	    
		int ret = vsprintf(buffer, format, vl); 
		va_end( vl );
		return ret;
	}
	inline int FBXSDK_snprintf(char* buffer, size_t dummy, size_t count, const char* format, ...) 
	{ 
		va_list vl;
		va_start( vl, format);
	    
		#if defined (FBXSDK_ENV_WIN)
			int ret = _vsnprintf(buffer, count, format, vl); 
		#else
			int ret = vsnprintf(buffer, count, format, vl); 
		#endif
		va_end( vl );
		return ret;
	}
	inline int FBXSDK_vsprintf(char* buffer, size_t dummy, const char* format, va_list vl) 
	{ 
		int ret = vsprintf(buffer, format, vl); 
		return ret;
	}
	inline int FBXSDK_vsnprintf(char* buffer, size_t dummy, size_t count, const char* format, va_list vl) 
	{ 
		int ret = vsnprintf(buffer, count, format, vl); 
		return ret;
	}

    #define FBXSDK_stricmp(dst, src)				stricmp(dst, src)
	#define FBXSDK_strnicmp(dst,src,count)			strnicmp(dst, src, count)
	#define FBXSDK_strcpy(dst,dstSize,src)			strcpy(dst, src)
	#define FBXSDK_strncpy(dst,dstSize,src,count)   strncpy(dst, src, count)
	#define FBXSDK_strcat(dst,dstSize,src)			strcat(dst, src)
	#define FBXSDK_strtok(str,delim,context)		strtok(str,delim)
	#define FBXSDK_strdup							strdup

	#define FBXSDK_wcscpy(dst,dstSize,src)			wcscpy(dst, src)
    #define FBXSDK_wcscat(dst,dstSize,src)          wcscat_s(dst,src)

	#define FBXSDK_getpid							getpid	

	#define FBXSDK_fopen(fp, name, mode)			fp=fopen(name, mode)
	#define FBXSDK_fclose							fclose

	#define FBXSDK_getcwd							getcwd

	#define FBXSDK_printf							printf
	#define FBXSDK_fprintf							fprintf

	#define FBXSDK_localtime(tm,time)				tm=localtime(time)
	#define FBXSDK_gmtime(tm, time)					tm=gmtime(time)


#elif defined (FBXSDK_ENV_WIN)
	// Microsoft secure calls
	#define FBXSDK_stricmp(dst, src)				_stricmp(dst, src)
	#define FBXSDK_strnicmp(dst,src,count)		    _strnicmp(dst, src, count)
	#define FBXSDK_strcpy(dst,dstSize,src)		    strcpy_s(dst, dstSize, src)
	#define FBXSDK_strncpy(dst,dstSize,src,count)   strncpy_s(dst, dstSize, src, count)
	#define FBXSDK_strcat(dst,dstSize,src)		    strcat_s(dst, dstSize, src)
	#define FBXSDK_strtok(str,delim,context)		strtok_s(str,delim,context)
	#define FBXSDK_strdup							_strdup

	#define FBXSDK_wcscpy(dst,dstSize,src)          wcscpy_s(dst, dstSize, src)
    #define FBXSDK_wcscat(dst,dstSize,src)          wcscat_s(dst,dstSize,src)

	#define FBXSDK_getpid							_getpid

	#define FBXSDK_fopen(fp, name, mode)			fopen_s(&fp, name, mode)
	#define FBXSDK_fclose							fclose

	#define FBXSDK_getcwd							_getcwd

	#define FBXSDK_printf							printf_s
	#define FBXSDK_fprintf						    fprintf_s
	#define FBXSDK_sprintf						    sprintf_s
	#define FBXSDK_vsprintf						    vsprintf_s
	#define FBXSDK_vsnprintf						vsnprintf_s
	#define FBXSDK_snprintf						    _snprintf_s

    #define FBXSDK_localtime(ptm,time)			    { struct tm tms; ptm = &tms; localtime_s(ptm, time); }
	#define FBXSDK_gmtime(ptm, time)				{ struct tm tms; ptm = &tms; gmtime_s(ptm, time); }
#else
	// Unknown platform
	#error Unsupported platform!
#endif

// The scanf family functions cannot easily be used in both secure and non-secure versions because
// Microsoft's secure version expects the size of the string/char* arguments following their address.
// On Unix machines the scanf family functions do not have this behavior and trying to use the same calls
// would result in compiler errors because the arguments would not match the format string.
//
// Using the following macros in the code will simply desable the warning at compile time.
//
#if defined(FBXSDK_COMPILER_MSC) && _MSC_VER >= 1300
    #define FBXSDK_CRT_SECURE_NO_WARNING_BEGIN  \
    { \
        __pragma(warning(push)) \
        __pragma(warning( disable : 4996 )) \
    }
    
	#define FBXSDK_CRT_SECURE_NO_WARNING_END \
    { \
        __pragma(warning(pop)) \
    }
#else
    #define FBXSDK_CRT_SECURE_NO_WARNING_BEGIN
    #define FBXSDK_CRT_SECURE_NO_WARNING_END
#endif

#include <fbxsdk/fbxsdk_nsend.h>

#endif /* _FBXSDK_CORE_ARCH_STDCOMPLIANT_H_ */

// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include <stdio.h>
#include <tchar.h>

// TODO: reference additional headers your program requires here

#include <dList.h>
#include <dString.h>

#include <imgui.h>
#include <glfw3.h>
#ifdef _WIN32
#undef APIENTRY
#define GLFW_EXPOSE_NATIVE_WIN32
#define GLFW_EXPOSE_NATIVE_WGL
#include <glfw3native.h>
#endif

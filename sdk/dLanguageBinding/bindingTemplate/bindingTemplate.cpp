// BindingTemplate.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "NewtonBinding.h"

#define targetPath			"./"

int _tmain(int argc, _TCHAR* argv[])
{
	NewtonBinding binding (targetPath);
	binding.Parse();
	return 0;
}


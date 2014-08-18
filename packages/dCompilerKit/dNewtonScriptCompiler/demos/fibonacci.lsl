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

//package demosPackage;


// naive method to calculate a fibonacci number
public class Fibonacci
{
/*
	// test inlining
	public static int test (int n)
	{
		return n;
	}
*/
	public static int Fibonacci (int n)
	{
//		n = test(n);
		if (n == 0)
			return 0;
		if (n == 1)
			return 1;
		return Fibonacci (n - 1) + Fibonacci (n - 2);
	}
}


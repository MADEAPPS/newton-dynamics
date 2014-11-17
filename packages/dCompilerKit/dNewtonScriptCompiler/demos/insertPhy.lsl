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


public class insertPhy
{

/*
	public static int insertPhy (int a)
	{
		int x = 0;
		if (a  > 2) {
			x = 1;
		} else {
			x = 2;
		}
		return x;
	}

	public static int insertPhy ()
	{
		int a = 0;
		int b = 0;
		int c = 0;
		do {
			b = a + 1;
			c = c + b;
			a = b * 2;
		} while (a < 200);
		return c;
	}
*/

	public static int insertPhy ()
	{
		int i = 1;
		int j = 1;
		int k = 0;
		while (k < 100) {
			if (j < 20) {
				//j = i;
				j = 1;
				k = k + 1;
			} else {
				j = k;
				k = k + 2;
			}
		}
		return j;
	}
}





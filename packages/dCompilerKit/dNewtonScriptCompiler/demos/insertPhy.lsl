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
/*
	public static int insertPhy ()
	{
		int i = 1;
		int j = 1;
		int k = 0;
		//while (k < 100) {
		do {
			if (j < 20) {
				j = i;
				k = k + 1;
			} else {
				j = k;
				k = k + 2;
			}
		} while (k < 100);
		return j;
	}
*/
	public static int insertPhy ()
	{
		int i = 6;
		int k = 1;
		int j = 100;
		do {
			i = i + k;
		} while (i == j);
		return i;
	}

/*
	public static int insertPhy ()
	{
		int i = 6;
		int j = 1;
		int k = 1;
		do {
			if (i == 6) {
				k = 0;
			} else {
				i = i + 1;
			}
			i = i + k;
			j = j + 1;
		} while (i == j);
		return i;
	}


	
	public static int insertPhy ()
	{
		int i = 12;
		do {
			int x = i + 17;
			int j = i;
			i = j;
		} while (i < 10 );
		return i;
	}
*/
}





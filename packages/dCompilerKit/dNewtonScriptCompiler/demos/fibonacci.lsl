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
	public static int main ()
	{
		return Calculate (2) + Calculate (3) + Calculate (5) + Calculate (7);
	}
*/	

	public static int Calculate (int n)
	{
		if (n == 0)
			return 0;
		if (n == 1)
			return 1;
		return Calculate (n - 1) + Calculate (n - 2);
	}

/*
	public static int Calculate1 (int a, int b, int c)
	{
		if (8 > 9) return 8; else if (5 >= 4) return 5; else return 9;
	}
*/	

/*
	public static int Calculate (int[] a, int b, int c)
	{
		int x = 1;
		int k;
		for (int i = 0; i < c; i++) {
//			x = 3;
			if (i < 10) {
				x = 2;
				i ++;
			}
			k = i;	
			a[i] = x;
		}
		return x + k;
	}
*/	
/*	
	public static void qsort (int[] a, int left, int right)
	{
	
      int i = left, j = right;
      
      int tmp;
      int pivot = a[(left + right) / 2];
      while (i <= j) {
         while (a[i] < pivot) i ++;
         while (a[j] > pivot) j--;

//			i--;
//			do {} while (a[++i] < pivot);
			
//			j++;
//			do {} while (a[--j] < pivot);

            if (i <= j) {
                  tmp = a[i];
                  a[i] = a[j];
                  a[j] = tmp;
                  i++;
                  j--;
            }
      }
      if (left < j)
		qsort (a, left, j);
	  if (i < right)
		qsort (a, i, right);
	}
*/	
}


/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "dCoreStdafx.h"
#include "dPerlinNoise.h"

// From Wikipedia page
//https://en.wikipedia.org/wiki/Perlin_noise


static const dUnsigned8 seed[256] = 
{
	151, 160, 137, 91, 90, 15,
	131, 13, 201, 95, 96, 53, 194, 233, 7, 225, 140, 36, 103, 30, 69, 142, 8, 99, 37, 240, 21, 10, 23,
	190, 6, 148, 247, 120, 234, 75, 0, 26, 197, 62, 94, 252, 219, 203, 117, 35, 11, 32, 57, 177, 33,
	88, 237, 149, 56, 87, 174, 20, 125, 136, 171, 168, 68, 175, 74, 165, 71, 134, 139, 48, 27, 166,
	77, 146, 158, 231, 83, 111, 229, 122, 60, 211, 133, 230, 220, 105, 92, 41, 55, 46, 245, 40, 244,
	102, 143, 54, 65, 25, 63, 161, 1, 216, 80, 73, 209, 76, 132, 187, 208, 89, 18, 169, 200, 196,
	135, 130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186, 3, 64, 52, 217, 226, 250, 124, 123,
	5, 202, 38, 147, 118, 126, 255, 82, 85, 212, 207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28, 42,
	223, 183, 170, 213, 119, 248, 152, 2, 44, 154, 163, 70, 221, 153, 101, 155, 167, 43, 172, 9,
	129, 22, 39, 253, 19, 98, 108, 110, 79, 113, 224, 232, 178, 185, 112, 104, 218, 246, 97, 228,
	251, 34, 242, 193, 238, 210, 144, 12, 191, 179, 162, 241, 81, 51, 145, 235, 249, 14, 239, 107,
	49, 192, 214, 31, 181, 199, 106, 157, 184, 84, 204, 176, 115, 121, 50, 45, 127, 4, 150, 254,
	138, 236, 205, 93, 222, 114, 67, 29, 24, 72, 243, 141, 128, 195, 78, 66, 215, 61, 156, 180
};


static inline dUnsigned8 randomSeed(dInt32 i)
{
	return seed[i];
}

static dFloat32 grad(dInt32 hash, dFloat32 x) 
{
	const dInt32 h = hash & 0x0F;  
	dFloat32 grad = 1.0f + (h & 7);    
	if ((h & 8) != 0)
	{
		grad = -grad;
	}
	return (grad * x);              
}

static dFloat32 grad(dInt32 hash, dFloat32 x, dFloat32 y) 
{
	const dInt32 h = hash & 0x3F;  
	const dFloat32 u = h < 4 ? x : y;  
	const dFloat32 v = h < 4 ? y : x;
	return ((h & 1) ? -u : u) + ((h & 2) ? -2.0f * v : 2.0f * v); 
}

static dFloat32 grad(dInt32 hash, dFloat32 x, dFloat32 y, dFloat32 z) 
{
	dInt32 h = hash & 15;     
	dFloat32 u = h < 8 ? x : y; 
	dFloat32 v = h < 4 ? y : h == 12 || h == 14 ? x : z; 
	return ((h & 1) ? -u : u) + ((h & 2) ? -v : v);
}


dFloat32 dPerlinNoise(dFloat32 x)
{
	dInt32 i0 = dInt32 (dFloor(x));
	dInt32 i1 = i0 + 1;
	dFloat32 x0 = x - i0;
	dFloat32 x1 = x0 - 1.0f;

	dFloat32 t0 = 1.0f - x0*x0;
	t0 *= t0;
	dFloat32 n0 = t0 * t0 * grad(randomSeed(i0), x0);

	dFloat32 t1 = 1.0f - x1*x1;
	t1 *= t1;
	dFloat32 n1 = t1 * t1 * grad(randomSeed(i1), x1);
	return 0.395f * (n0 + n1);
}

dFloat32 dPerlinNoise(dFloat32 x, dFloat32 y) 
{
	// F2 = (sqrt(3) - 1) / 2
	const dFloat32 F2 = 0.366025403f; 
	// G2 = (3 - sqrt(3)) / 6   = F2 / (1 + 2 * K)
	const dFloat32 G2 = 0.211324865f;  

	const dFloat32 s = (x + y) * F2; 
	const dFloat32 xs = x + s;
	const dFloat32 ys = y + s;
	const dInt32 i = dInt32(dFloor(xs));
	const dInt32 j = dInt32(dFloor(ys));

	const dFloat32 t = dFloat32(i + j) * G2;
	const dFloat32 X0 = i - t;
	const dFloat32 Y0 = j - t;
	const dFloat32 x0 = x - X0;
	const dFloat32 y0 = y - Y0;

	dInt32 i1, j1; 
	if (x0 > y0) 
	{ 
		i1 = 1;
		j1 = 0;
	}
	else 
	{
		i1 = 0;
		j1 = 1;
	}

	const dFloat32 x1 = x0 - i1 + G2;
	const dFloat32 y1 = y0 - j1 + G2;
	const dFloat32 x2 = x0 - 1.0f + 2.0f * G2;
	const dFloat32 y2 = y0 - 1.0f + 2.0f * G2;

	const dInt32 gi0 = randomSeed(i + randomSeed(j));
	const dInt32 gi1 = randomSeed(i + i1 + randomSeed(j + j1));
	const dInt32 gi2 = randomSeed(i + 1 + randomSeed(j + 1));

	dFloat32 t0 = 0.5f - x0*x0 - y0*y0;

	dFloat32 n0 = 0.0f;
	if (t0 >= 0.0f) 
	{
		t0 *= t0;
		n0 = t0 * t0 * grad(gi0, x0, y0);
	}

	dFloat32 n1 = 0.0f;
	dFloat32 t1 = 0.5f - x1*x1 - y1*y1;
	if (t1 >= 0.0f) 
	{
		t1 *= t1;
		n1 = t1 * t1 * grad(gi1, x1, y1);
	}

	dFloat32 n2 = 0.0f;
	dFloat32 t2 = 0.5f - x2*x2 - y2*y2;
	if (t2 >= 0.0f) 
	{
		t2 *= t2;
		n2 = t2 * t2 * grad(gi2, x2, y2);
	}

	return 45.23065f * (n0 + n1 + n2);
}

dFloat32 dPerlinNoise(dFloat32 x, dFloat32 y, dFloat32 z) 
{
	const dFloat32 F3 = 1.0f / 3.0f;
	const dFloat32 G3 = 1.0f / 6.0f;

	dFloat32 s = (x + y + z) * F3; 
	dInt32 i = dInt32 (dFloor(x + s));
	dInt32 j = dInt32 (dFloor(y + s));
	dInt32 k = dInt32 (dFloor(z + s));
	dFloat32 t = (i + j + k) * G3;
	dFloat32 X0 = i - t; 
	dFloat32 Y0 = j - t;
	dFloat32 Z0 = k - t;
	dFloat32 x0 = x - X0;
	dFloat32 y0 = y - Y0;
	dFloat32 z0 = z - Z0;

	dInt32 i1, j1, k1;
	dInt32 i2, j2, k2;
	if (x0 >= y0) 
	{
		if (y0 >= z0) 
		{
			i1 = 1; j1 = 0; k1 = 0; i2 = 1; j2 = 1; k2 = 0; // X Y Z order
		}
		else if (x0 >= z0) 
		{
			i1 = 1; j1 = 0; k1 = 0; i2 = 1; j2 = 0; k2 = 1; // X Z Y order
		}
		else 
		{
			i1 = 0; j1 = 0; k1 = 1; i2 = 1; j2 = 0; k2 = 1; // Z X Y order
		}
	}
	else 
	{ 
		if (y0 < z0) 
		{
			i1 = 0; j1 = 0; k1 = 1; i2 = 0; j2 = 1; k2 = 1; // Z Y X order
		}
		else if (x0 < z0) 
		{
			i1 = 0; j1 = 1; k1 = 0; i2 = 0; j2 = 1; k2 = 1; // Y Z X order
		}
		else 
		{
			i1 = 0; j1 = 1; k1 = 0; i2 = 1; j2 = 1; k2 = 0; // Y X Z order
		}
	}

	dFloat32 x1 = x0 - i1 + G3;
	dFloat32 y1 = y0 - j1 + G3;
	dFloat32 z1 = z0 - k1 + G3;
	dFloat32 x2 = x0 - i2 + 2.0f * G3;
	dFloat32 y2 = y0 - j2 + 2.0f * G3;
	dFloat32 z2 = z0 - k2 + 2.0f * G3;
	dFloat32 x3 = x0 - 1.0f + 3.0f * G3;
	dFloat32 y3 = y0 - 1.0f + 3.0f * G3;
	dFloat32 z3 = z0 - 1.0f + 3.0f * G3;

	dInt32 gi0 = randomSeed(i + randomSeed(j + randomSeed(k)));
	dInt32 gi1 = randomSeed(i + i1 + randomSeed(j + j1 + randomSeed(k + k1)));
	dInt32 gi2 = randomSeed(i + i2 + randomSeed(j + j2 + randomSeed(k + k2)));
	dInt32 gi3 = randomSeed(i + 1 + randomSeed(j + 1 + randomSeed(k + 1)));

	dFloat32 n0 = 0.0f;
	dFloat32 t0 = 0.6f - x0*x0 - y0*y0 - z0*z0;
	if (t0 >= 0) 
	{
		t0 *= t0;
		n0 = t0 * t0 * grad(gi0, x0, y0, z0);
	}

	dFloat32 n1 = 0.0f;
	dFloat32 t1 = 0.6f - x1*x1 - y1*y1 - z1*z1;
	if (t1 >= 0) 
	{
		t1 *= t1;
		n1 = t1 * t1 * grad(gi1, x1, y1, z1);
	}

	dFloat32 n2 = 0.0f;
	dFloat32 t2 = 0.6f - x2*x2 - y2*y2 - z2*z2;
	if (t2 >= 0) 
	{
		t2 *= t2;
		n2 = t2 * t2 * grad(gi2, x2, y2, z2);
	}

	dFloat32 n3 = 0.0f;
	dFloat32 t3 = 0.6f - x3*x3 - y3*y3 - z3*z3;
	if (t3 >= 0) 
	{
		t3 *= t3;
		n3 = t3 * t3 * grad(gi3, x3, y3, z3);
	}
	return 32.0f*(n0 + n1 + n2 + n3);
}

dFloat32 BrownianMotion(size_t octaves, dFloat32 x)
{
	dAssert(0);
	return 0;
	//dFloat32 output = 0.f;
	//dFloat32 denom = 0.f;
	//dFloat32 frequency = mFrequency;
	//dFloat32 amplitude = mAmplitude;
	//
	//for (size_t i = 0; i < octaves; i++) 
	//{
	//	output += (amplitude * noise(x * frequency));
	//	denom += amplitude;
	//
	//	frequency *= mLacunarity;
	//	amplitude *= mPersistence;
	//}
	//
	//return (output / denom);
}

dFloat32 BrownianMotion(size_t octaves, dFloat32 x, dFloat32 y)
{
	dAssert(0);
	return 0;
	//dFloat32 output = 0.f;
	//dFloat32 denom = 0.f;
	//dFloat32 frequency = mFrequency;
	//dFloat32 amplitude = mAmplitude;
	//
	//for (size_t i = 0; i < octaves; i++) {
	//	output += (amplitude * noise(x * frequency, y * frequency));
	//	denom += amplitude;
	//
	//	frequency *= mLacunarity;
	//	amplitude *= mPersistence;
	//}
	//
	//return (output / denom);
}

dFloat32 BrownianMotion(size_t octaves, dFloat32 x, dFloat32 y, dFloat32 z)
{
	dAssert(0);
	return 0;

	//dFloat32 output = 0.f;
	//dFloat32 denom = 0.f;
	//dFloat32 frequency = mFrequency;
	//dFloat32 amplitude = mAmplitude;
	//
	//for (size_t i = 0; i < octaves; i++) 
	//{
	//	output += (amplitude * noise(x * frequency, y * frequency, z * frequency));
	//	denom += amplitude;
	//
	//	frequency *= mLacunarity;
	//	amplitude *= mPersistence;
	//}
	//
	//return (output / denom);
}

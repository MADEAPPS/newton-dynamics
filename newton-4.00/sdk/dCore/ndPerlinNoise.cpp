/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndCoreStdafx.h"
#include "ndPerlinNoise.h"

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

static inline dFloat32 Interpolate(dFloat32 t, dFloat32 v1, dFloat32 v2)
{
	return v1 + t * (v2 - v1);
}

static inline dFloat32 CubicSmoothing(dFloat32 val) 
{
	dFloat32 val3 = val * val * val;
	dFloat32 val4 = val3 * val;
	return 6.0f * val4 * val - 15.0f * val4 + 10.0f * val3;
}

static inline dFloat32 Gradient(dInt32 x, dFloat32 dx)
{
	dInt32 h = seed[x & 255];
	return ((h & 1) ? -dx : dx);
}

static inline dFloat32 Gradient(int32_t x, int32_t y, dFloat32 dx, dFloat32 dy)
{
	int32_t h = seed[(seed[x & 255] + y) & 255];
	return ((h & 1) ? -dx : dx) + ((h & 2) ? -dy : dy);
}

static inline dFloat32 Gradient(int32_t x, int32_t y, int32_t z, dFloat32 dx, dFloat32 dy, dFloat32 dz)
{
	int32_t h = seed[seed[seed[x & 255] + y & 255] + z & 255];
	h &= 15;

	//	Ken Perlins original implementation
	dFloat32 u = h < 8 ? dx : dy;
	dFloat32 v = h < 4 ? dy : (h == 12 || h == 14) ? dx : dz;
	return ((h & 1) ? -u : u) + ((h & 2) ? -v : v);
}

dFloat32 dPerlinNoise(dFloat32 x)
{
	dInt32 ix = dInt32(dFloor(x));
	dFloat32 dx = x - ix;

	dFloat32 w00 = Gradient(ix, dx);
	dFloat32 w10 = Gradient(ix + 1, dx - 1.0f);
	dFloat32 wx = CubicSmoothing(dx);
	dFloat32 x0 = Interpolate(wx, w00, w10);
	return x0;
}

dFloat32 dPerlinNoise(dFloat32 x, dFloat32 y)
{
	dInt32 ix = dInt32(dFloor(x));
	dInt32 iy = dInt32(dFloor(y));
	dFloat32 dx = x - ix;
	dFloat32 dy = y - iy;

	dFloat32 w00 = Gradient(ix, iy, dx, dy);
	dFloat32 w10 = Gradient(ix + 1, iy, dx - 1.0f, dy);
	dFloat32 w01 = Gradient(ix, iy + 1, dx, dy - 1.0f);
	dFloat32 w11 = Gradient(ix + 1, iy + 1, dx - 1.0f, dy - 1.0f);

	dFloat32 wx = CubicSmoothing(dx);
	dFloat32 wy = CubicSmoothing(dy);

	dFloat32 x0 = Interpolate(wx, w00, w10);
	dFloat32 x1 = Interpolate(wx, w01, w11);

	return Interpolate(wy, x0, x1);
}

dFloat32 dPerlinNoise(dFloat32 x, dFloat32 y, dFloat32 z)
{
	dInt32 ix = dInt32(dFloor(x));
	dInt32 iy = dInt32(dFloor(y));
	dInt32 iz = dInt32(dFloor(z));

	dFloat32 dx = x - ix;
	dFloat32 dy = y - iy;
	dFloat32 dz = z - iz;

	dFloat32 w000 = Gradient(ix, iy, iz, dx, dy, dz);
	dFloat32 w100 = Gradient(ix + 1, iy, iz, dx - 1, dy, dz);
	dFloat32 w010 = Gradient(ix, iy + 1, iz, dx, dy - 1, dz);
	dFloat32 w110 = Gradient(ix + 1, iy + 1, iz, dx - 1, dy - 1, dz);
	dFloat32 w001 = Gradient(ix, iy, iz + 1, dx, dy, dz - 1);
	dFloat32 w101 = Gradient(ix + 1, iy, iz + 1, dx - 1, dy, dz - 1);
	dFloat32 w011 = Gradient(ix, iy + 1, iz + 1, dx, dy - 1, dz - 1);
	dFloat32 w111 = Gradient(ix + 1, iy + 1, iz + 1, dx - 1, dy - 1, dz - 1);
	
	dFloat32 wx = CubicSmoothing(dx);
	dFloat32 wy = CubicSmoothing(dy);
	dFloat32 wz = CubicSmoothing(dz);

	dFloat32 x00 = Interpolate(wx, w000, w100);
	dFloat32 x10 = Interpolate(wx, w010, w110);
	dFloat32 x01 = Interpolate(wx, w001, w101);
	dFloat32 x11 = Interpolate(wx, w011, w111);

	dFloat32 y0 = Interpolate(wy, x00, x10);
	dFloat32 y1 = Interpolate(wy, x01, x11);

	return Interpolate(wz, y0, y1);
}

dFloat32 BrownianMotion(dInt32 octaves, dFloat32 persistence, dFloat32 x)
{
	dFloat32 noise = dFloat32 (0.0f);
	dFloat32 amplitud = dFloat32 (1.0f);
	dFloat32 frequency = dFloat32 (1.0f);

	for (dInt32 i = 0; i < octaves; i++)
	{
		dFloat32 fx = x * frequency;
		noise += amplitud * dPerlinNoise(fx);
		amplitud *= persistence;
		frequency *= dFloat32 (2.0f);
	}
	return noise;
}

dFloat32 BrownianMotion(dInt32 octaves, dFloat32 persistence, dFloat32 x, dFloat32 y)
{
	dFloat32 noise = dFloat32 (0.0f);
	dFloat32 amplitud = dFloat32 (1.0f);
	dFloat32 frequency = dFloat32 (1.0f);

	for (dInt32 i = 0; i < octaves; i++)
	{
		dFloat32 fx = x * frequency;
		dFloat32 fy = y * frequency;
		noise += amplitud * dPerlinNoise (fx, fy);
		amplitud *= persistence;
		frequency *= dFloat32 (2.0f);
	}
	return noise;
}

dFloat32 BrownianMotion(dInt32 octaves, dFloat32 persistence, dFloat32 x, dFloat32 y, dFloat32 z)
{
	dFloat32 noise = dFloat32 (0.0f);
	dFloat32 amplitud = dFloat32 (1.0f);
	dFloat32 frequency = dFloat32 (1.0f);

	for (dInt32 i = 0; i < octaves; i++)
	{
		dFloat32 fx = x * frequency;
		dFloat32 fy = y * frequency;
		dFloat32 fz = z * frequency;
		noise += amplitud * dPerlinNoise(fx, fy, fz);
		amplitud *= persistence;
		frequency *= dFloat32 (2.0f);
	}
	return noise;
}

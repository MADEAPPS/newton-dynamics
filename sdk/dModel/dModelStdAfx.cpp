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

// stdafx.cpp : source file that includes just the standard includes
// containers.pch will be the pre-compiled header
// stdafx.obj will contain the pre-compiled type information
#include "dModelStdAfx.h"

static int PruneSupport(int count, const dVector& dir, const dVector* points)
{
	int index = 0;
	dFloat maxVal = dFloat(-1.0e20f);
	for (int i = 0; i < count; i++) {
		dFloat dist = dir.DotProduct3(points[i]);
		if (dist > maxVal) {
			index = i;
			maxVal = dist;
		}
	}
	return index;
}

int Calculate2dConvexHullProjection (int count, dVector* const points)
{
	dVector origin(0.0f);
	for (int i = 0; i < count; i++) {
		origin += points[i];
	}
	dVector scale(dFloat(1.0f) / count);
	origin = origin * scale;
	origin.m_w = dFloat(1.0f);

	dMatrix covariance(dGetZeroMatrix());
	for (int i = 0; i < count; i++) {
		dVector p(points[i] - origin);
		p.m_w = 0.0f;
		covariance.m_front += p * dVector (p.m_x);
		covariance.m_up += p * dVector (p.m_y);
		covariance.m_right += p * dVector (p.m_z);
	}

	for (int i = 0; i < 3; i++) {
		if (dAbs(covariance[i][i]) < (1.0e-6f)) {
			for (int j = 0; j < 3; j++) {
				covariance[i][j] = dFloat(0.0f);
				covariance[j][i] = dFloat(0.0f);
			}
		}
	}

	dVector eigen;
	covariance = covariance.JacobiDiagonalization (eigen);
	covariance.m_posit = origin;

	if (eigen[1] < eigen[2]) {
		dSwap(eigen[1], eigen[2]);
		dSwap(covariance[1], covariance[2]);
	}
	if (eigen[0] < eigen[1]) {
		dSwap(eigen[0], eigen[1]);
		dSwap(covariance[0], covariance[1]);
	}
	if (eigen[1] < eigen[2]) {
		dSwap(eigen[1], eigen[2]);
		dSwap(covariance[1], covariance[2]);
	}

	class dConveFaceNode
	{
		public:
		dVector m_point2d;
		dVector m_contact;
		dConveFaceNode* m_next;
		dConveFaceNode* m_prev;
		int m_mask;
	};

	class dHullStackSegment
	{
		public:
		dVector m_p0;
		dVector m_p1;
		dConveFaceNode* m_edgeP0;
	};

	dVector array[32];
	dHullStackSegment stack[32];
	dConveFaceNode convexHull[32];
	dVector buffer[32];

	// it is a big mistake to set contact to deepest penetration because si cause unwanted pops.
	// is better to present the original contact penetrations
	//dFloat maxPenetration = dFloat(0.0f);
	for (int i = 0; i < count; i++) {
		array[i] = covariance.UntransformVector(points[i]);
		array[i].m_z = 0.0f;
	}


	dVector m_pruneUpDir(dFloat(0.0f), dFloat(0.0f), dFloat(1.0f), dFloat(0.0f));
	dVector m_pruneSupportX(dFloat(1.0f), dFloat(0.0f), dFloat(0.0f), dFloat(0.0f));
	int i0 = PruneSupport(count, m_pruneSupportX, array);
	count--;

	convexHull[0].m_point2d = array[i0];
	convexHull[0].m_contact = points[i0];
	stack[0].m_p0 = array[i0];
	array[i0] = array[count];
	points[i0] = points[count];

	int i1 = PruneSupport(count, m_pruneSupportX.Scale(dFloat(-1.0f)), array);
	count--;
	convexHull[1].m_point2d = array[i1];
	convexHull[1].m_contact = points[i1];
	stack[0].m_p1 = array[i1];
	array[i1] = array[count];
	points[i1] = points[count];

	stack[0].m_edgeP0 = &convexHull[0];
	convexHull[0].m_next = &convexHull[1];
	convexHull[0].m_prev = &convexHull[1];
	convexHull[1].m_next = &convexHull[0];
	convexHull[1].m_prev = &convexHull[0];

	stack[1].m_edgeP0 = &convexHull[1];
	stack[1].m_p0 = stack[0].m_p1;
	stack[1].m_p1 = stack[0].m_p0;

	int hullCount = 2;
	int stackIndex = 2;
	dFloat totalArea = dFloat(0.0f);
	while (stackIndex && count && (hullCount < sizeof (convexHull) / sizeof (convexHull[0]))) {
		stackIndex--;

		dHullStackSegment segment(stack[stackIndex]);
		dVector p1p0((segment.m_p1 - segment.m_p0));
		dFloat mag2 = p1p0.DotProduct3(p1p0);

		if (mag2 > dFloat(1.0e-5f)) {
			dVector dir(m_pruneUpDir.CrossProduct(p1p0));
			int newIndex = PruneSupport(count, dir, array);

			dVector edge(array[newIndex] - segment.m_p0);
			dVector normal(p1p0.CrossProduct(edge));
			if (normal.m_z > dFloat(1.e-4f)) {
				totalArea += normal.m_z;
				dAssert(stackIndex < 30);
				convexHull[hullCount].m_point2d = array[newIndex];
				convexHull[hullCount].m_contact = points[newIndex];
				convexHull[hullCount].m_next = segment.m_edgeP0->m_next;
				segment.m_edgeP0->m_next->m_prev = &convexHull[hullCount];

				convexHull[hullCount].m_prev = segment.m_edgeP0;
				segment.m_edgeP0->m_next = &convexHull[hullCount];

				stack[stackIndex + 0].m_p0 = segment.m_p0;
				stack[stackIndex + 0].m_p1 = array[newIndex];
				stack[stackIndex + 0].m_edgeP0 = segment.m_edgeP0;

				stack[stackIndex + 1].m_p0 = array[newIndex];
				stack[stackIndex + 1].m_p1 = segment.m_p1;
				stack[stackIndex + 1].m_edgeP0 = &convexHull[hullCount];

				hullCount++;
				stackIndex += 2;
				count--;
				array[newIndex] = array[count];
				points[newIndex] = points[count];
			}
		}
	}

	dAssert(hullCount < sizeof (convexHull) / sizeof (convexHull[0]));
	dConveFaceNode* hullPoint = &convexHull[0];
/*
	dgUpHeap<dConveFaceNode*, dFloat> sortHeap(array, sizeof (array));
	bool hasLinearCombination = true;
	while (hasLinearCombination) {
		sortHeap.Flush();
		hasLinearCombination = false;
		dConveFaceNode* ptr = hullPoint;
		dVector e0(ptr->m_next->m_point2d - ptr->m_point2d);
		do {
			dVector e1(ptr->m_next->m_next->m_point2d - ptr->m_next->m_point2d);
			dFloat area = e0.m_y * e1.m_x - e0.m_x * e1.m_y;
			sortHeap.Push(ptr->m_next, area);
			e0 = e1;
			ptr->m_mask = 1;
			ptr = ptr->m_next;
		} while (ptr != hullPoint);

		while (sortHeap.GetCount() && (sortHeap.Value() * dFloat(16.0f) < totalArea)) {
			dConveFaceNode* const corner = sortHeap[0];
			if (corner->m_mask && corner->m_prev->m_mask) {
				if (hullPoint == corner) {
					hullPoint = corner->m_prev;
				}
				hullCount--;
				hasLinearCombination = true;
				corner->m_prev->m_mask = 0;
				corner->m_next->m_prev = corner->m_prev;
				corner->m_prev->m_next = corner->m_next;
			}
			sortHeap.Pop();
		}
	}

	while (hullCount > maxCount) {
		sortHeap.Flush();
		dConveFaceNode* ptr = hullPoint;
		dVector e0(ptr->m_next->m_point2d - ptr->m_point2d);
		do {
			dVector e1(ptr->m_next->m_next->m_point2d - ptr->m_next->m_point2d);
			dFloat area = e0.m_y * e1.m_x - e0.m_x * e1.m_y;
			sortHeap.Push(ptr->m_next, area);
			e0 = e1;
			ptr->m_mask = 1;
			ptr = ptr->m_next;
		} while (ptr != hullPoint);

		while (sortHeap.GetCount() && (hullCount > maxCount)) {
			dConveFaceNode* const corner = sortHeap[0];
			if (corner->m_mask && corner->m_prev->m_mask) {
				if (hullPoint == corner) {
					hullPoint = corner->m_prev;
				}
				hullCount--;
				hasLinearCombination = true;
				corner->m_prev->m_mask = 0;
				corner->m_next->m_prev = corner->m_prev;
				corner->m_prev->m_next = corner->m_next;
			}
			sortHeap.Pop();
		}
	}
*/
	hullCount = 0;
	dConveFaceNode* ptr = hullPoint;
	do {
		points[hullCount] = ptr->m_contact;
		hullCount++;
		ptr = ptr->m_next;
	} while (ptr != hullPoint);
	return hullCount;
}

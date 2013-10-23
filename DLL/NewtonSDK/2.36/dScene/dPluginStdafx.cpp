/////////////////////////////////////////////////////////////////////////////
// Name:        dPluginStdafx.h
// Purpose:     
// Author:      Julio Jerez
// Modified by: 
// Created:     22/05/2010 07:45:05
// RCS-ID:      
// Copyright:   Copyright (c) <2010> <Newton Game Dynamics>
// License:     
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
// 
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely
/////////////////////////////////////////////////////////////////////////////



#include "dPluginStdafx.h"

void dIntArrayToString (const int* const array, int count, char* const string, int maxSixeInBytes)
{
	if (count) {
		char* ptr = string;
		sprintf (string, " ");
		for (int i = 0; i < count; i ++) {
			sprintf (ptr, "%d ", array[i]);
			ptr += strlen (ptr);
			_ASSERTE ((ptr - string) < maxSixeInBytes);
		}
		string[ptr - string - 1] = 0;
	}
}

void dFloatArrayToString (const dFloat* const array, int count, char* const string, int maxSixeInBytes)
{
	if (count) {
		char* ptr = string;
		sprintf (string, " ");
		for (int i = 0; i < count; i ++) {
			sprintf (ptr, "%f ", array[i]);
			ptr += strlen (ptr);
			_ASSERTE ((ptr - string) < maxSixeInBytes);
		}
		string[ptr - string - 1] = 0;
	}
}

void dStringToIntArray (const char* const string, int* const array, int maxCount)
{
	const char* ptr = string;
	for (int i = 0; i < maxCount; i ++) {
		char value[128];		
		while (*ptr == ' ') {
			ptr ++;
		}
		int j = 0;
		while (*ptr != ' ' && *ptr) {
			value[j] = *ptr;
			ptr ++;
			j ++;
		}
		value[j] = 0;
		int val = atoi (value);
		array[i] = val;
	}
}

void dStringToFloatArray (const char* const string, dFloat* const array, int maxCount)
{
	const char* ptr = string;
	for (int i = 0; i < maxCount; i ++) {
		char value[128];
		while (*ptr == ' ') {
			ptr ++;
		}
		int j = 0;
		while (*ptr != ' ' && *ptr) {
			value[j] = *ptr;
			ptr ++;
			j ++;
		}
		value[j] = 0;
		dFloat val = dFloat (atof (value));
		array[i] = val;
	}
}
/*
void dStringToFloat64Array (const char* const string, dFloat64* const array, int maxCount)
{
	const char* ptr = string;
	for (int i = 0; i < maxCount; i ++) {
		char value[128];
		while (*ptr == ' ') {
			ptr ++;
		}
		int j = 0;
		while (*ptr != ' ' && *ptr) {
			value[j] = *ptr;
			ptr ++;
			j ++;
		}
		value[j] = 0;
		dFloat64 val = dFloat64 (atof (value));
		array[i] = val;
	}
}
*/

static int SortVertexArray (const void *A, const void *B) 
{
	const dFloat* const vertexA = (dFloat*) A;
	const dFloat* const vertexB = (dFloat*) B;

	if (vertexA[0] < vertexB[0]) {
		return -1;
	} else if (vertexA[0] > vertexB[0]) {
		return 1;
	} else {
		return 0;
	}
}


int dPackVertexArray (dFloat* const vertexList, int compareElements, int strideInBytes, int vertexCount, int* const indexListOut)
{
	int stride = strideInBytes / sizeof (dFloat);
	dFloat errorTol = dFloat (1.0e-4f);

	dFloat* const array = new dFloat[(stride + 2) * vertexCount];
	for (int i = 0; i < vertexCount; i ++) {
		memcpy (&array[i * (stride + 2)], &vertexList[i * stride], strideInBytes);
		array[i * (stride + 2) + stride + 0] = dFloat(i);
		array[i * (stride + 2) + stride + 1] = 0.0f;
	}

	qsort(array, vertexCount, (stride + 2) * sizeof (dFloat), SortVertexArray);
	int indexCount = 0;
	for (int i = 0; i < vertexCount; i ++) {
		int index = i * (stride + 2);
		if (array[index + stride + 1] == 0.0f) {
			dFloat window = array[index] + errorTol; 
			for (int j = i + 1; j < vertexCount; j ++) {
				int index2 = j * (stride + 2);
				if (array[index2] >= window) {
					break;
				}
				if (array[index2 + stride + 1] == 0.0f) {
					int k;
					for (k = 0; k < compareElements; k ++) {
						dFloat error;
						error = array[index + k] - array[index2+ k];
						if (dAbs (error) >= errorTol) {
							break;
						}
					}
					if (k >= compareElements) {
						int m = int (array[index2 + stride]);
						memcpy (&array[indexCount * (stride + 2)], &array[index2], sizeof (dFloat) * stride);
						indexListOut[m] = indexCount;
						array[index2 + stride + 1] = 1.0f;
					}
				}
			}
			int m = int (array[index + stride]);
			memcpy (&array[indexCount * (stride + 2)], &array[index], sizeof (dFloat) * stride);
			indexListOut[m] = indexCount;
			array[indexCount * (stride + 2) + stride + 1] = 1.0f;
			indexCount ++;
		}
	}

	for (int i = 0; i < indexCount; i ++) {
		memcpy (&vertexList[i * stride], &array[i * (stride + 2)], sizeof (dFloat) * stride);
	}

	delete[] array;
	return indexCount;
}

const char* dGetNameFromPath (const char* const fullName)
{
	const char* ptr = strrchr ((char*) fullName, '\\');
	if (!ptr) {
		ptr = strrchr ((char*) fullName, '/');
	}
	if (ptr) {
		ptr ++;
	} else {
		ptr = fullName;
	}
	return ptr;
}

void dExtractPathFromFullName (const char* const fullName, char* const path)
{
	strcpy (path, fullName);
	char* ptr = (char*)dGetNameFromPath(path);
	if (ptr != path) {
		ptr --;
	}
	ptr[0] = 0;
}


dFloat dBoxRayCast (const dVector& line0, const dVector& line1, const dVector& boxP0, const dVector& boxP1) 
{	
	int index = 0;
	dFloat signDir = dFloat (0.0f);
	dFloat tmin = dFloat (0.0f);
	dFloat tmax = dFloat (1.0f);

	for (int i = 0; i < 3; i++) {
		dFloat dp = line1[i] - line0[i];

		if (dAbs (dp) < dFloat (1.0e-8f)) {
			if (line0[i] <= boxP0[i] || line0[i] >= boxP1[i]) {
				return dFloat (1.2f);
			}
		} else {

			dp = dFloat (1.0f) / dp; 
			dFloat t1 = (boxP1[i] - line0[i]) * dp;
			dFloat t2 = (boxP0[i] - line0[i]) * dp;

			dFloat sign = dFloat (-1.0f);
			if (t1 > t2) {
				dFloat t = t1;
				t1 = t2;
				t2 = t;
				sign = 1;
			}

			if (t1 > tmin) {
				signDir = sign;
				index = i;
				tmin = t1;
			}
			if (t2 < tmax) {
				tmax = t2;
			}
			if (tmin > tmax) {
				return dFloat (1.2f);
			}
		}
	}

	if (tmin <= 0.0f) {
		tmin = 1.2f;
	}

	return tmin;
}

dVector dPolygonNormal (int indexCount, const dFloat* const vertex, int strideInBytes, const int* const indices)
{
	int stride = strideInBytes / sizeof (dFloat);
	int index = indices[0] * stride;
	dVector p0 (vertex[index], vertex[index + 1], vertex[index + 2], 0.0f);

	index = indices[1] * stride;
	dVector p1 (vertex[index], vertex[index + 1], vertex[index + 2], 0.0f);

	dVector e0 (p1 - p0);
	dVector normal (0.0, 0.0f, 0.0f);
	for (int i = 2; i < indexCount; i ++) {
		index = indices[i] * stride;
		dVector p2 (vertex[index], vertex[index + 1], vertex[index + 2], 0.0f);
		dVector e1 (p2 - p0);
		normal += e0 * e1;
		e0 = e1;
	}
	return normal;
}


dFloat dPolygonRayCast (const dVector& l0, const dVector& l1, int indexCount, const dFloat* const vertex, int strideInBytes, const int* const indices)
{
	int stride = strideInBytes / sizeof (dFloat);

	dBigVector line0 (l0);
	dBigVector line1(l1);
	dBigVector segment (line1 - line0);
	dBigVector normal (dPolygonNormal (indexCount, vertex, strideInBytes, indices));
	double den = normal % segment;
	if (dAbs(den) < 1.0e-6) {
		return 1.2f;
	}
	
	double sign = (den < 0.0f) ? 1.0f : -1.0f;

	int index = indices[indexCount - 1] * stride;
	dBigVector v0 (vertex[index], vertex[index + 1], vertex[index + 2], 0.0f);
	dBigVector p0v0 (v0 - line0);
	for (int i = 0; i < indexCount; i ++) {
		index = indices[i] * stride;
		dBigVector v1 (vertex[index], vertex[index + 1], vertex[index + 2], 0.0f);
		dBigVector p0v1 (v1 - line0);
		double alpha = sign * ((p0v1 * p0v0) % segment);
		if (alpha < 1.0e-3f) {
			return 1.2f;
		}
		p0v0 = p0v1;
	}

	double t = - ((line0 - v0) % normal) / den;
	if ((t < 0.0f) || (t > 1.0f)) {
		return 1.2f;
	}
	return dFloat (t);
}

void dRayToRayCast (
	const dVector& ray_p0, const dVector& ray_p1, 
	const dVector& ray_q0, const dVector& ray_q1,
	dVector& pOut, dVector& qOut)
{
	dVector u (ray_p1 - ray_p0);
	dVector v (ray_q1 - ray_q0);
	dVector w (ray_p0 - ray_q0);

	dFloat a = u % u;        // always >= 0
	dFloat b = u % v;
	dFloat c = v % v;        // always >= 0
	dFloat d = u % w;
	dFloat e = v % w;
	dFloat D = a*c - b*b;   // always >= 0
	dFloat sD = D;			// sc = sN / sD, default sD = D >= 0
	dFloat tD = D;			// tc = tN / tD, default tD = D >= 0

	// compute the line parameters of the two closest points
	dFloat sN;
	dFloat tN;
	if (D < dFloat (1.0e-8f)) { 
		// the lines are almost parallel
		sN = dFloat (0.0f);  // force using point P0 on segment S1
		sD = dFloat (1.0f);  // to prevent possible division by 0.0 later
		tN = e;
		tD = c;
	} else {                
		// get the closest points on the infinite lines
		sN = (b*e - c*d);
		tN = (a*e - b*d);
		if (sN < dFloat (0.0f)) {       
			// sc < 0 => the s=0 edge is visible
			sN = dFloat (0.0f);
			tN = e;
			tD = c;
		}
		else if (sN > sD) {  // sc > 1 => the s=1 edge is visible
			sN = sD;
			tN = e + b;
			tD = c;
		}
	}


	if (tN < dFloat (0.0f)) {           // tc < 0 => the t=0 edge is visible
		tN = dFloat (0.0f);
		// recompute sc for this edge
		if (-d < dFloat (0.0f))
			sN = dFloat (0.0f);
		else if (-d > a)
			sN = sD;
		else {
			sN = -d;
			sD = a;
		}
	}
	else if (tN > tD) {      // tc > 1 => the t=1 edge is visible
		tN = tD;
		// recompute sc for this edge
		if ((-d + b) < dFloat (0.0f))
			sN = dFloat (0.0f);
		else if ((-d + b) > a)
			sN = sD;
		else {
			sN = (-d + b);
			sD = a;
		}
	}

	// finally do the division to get sc and tc
	dFloat sc = (dAbs(sN) < dFloat(1.0e-8f) ? dFloat (0.0f) : sN / sD);
	dFloat tc = (dAbs(tN) < dFloat(1.0e-8f) ? dFloat (0.0f) : tN / tD);

	pOut = ray_p0 + u.Scale (sc);
	qOut = ray_q0 + v.Scale (tc);
}

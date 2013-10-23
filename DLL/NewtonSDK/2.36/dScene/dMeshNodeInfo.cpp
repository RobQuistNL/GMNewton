/////////////////////////////////////////////////////////////////////////////
// Name:        dMeshNodeInfo.h
// Purpose:     
// Author:      Julio Jerez
// Modified by: 
// Created:     22/05/2010 08:02:08
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
#include "dScene.h"
#include "dMeshNodeInfo.h"
#include "dTextureNodeInfo.h"
#include "dMaterialNodeInfo.h"
#include "dGeometryNodeModifierInfo.h"

D_IMPLEMENT_CLASS_NODE(dMeshNodeInfo);



dMeshNodeInfo::dMeshNodeInfo()
	:dGeometryNodeInfo (NULL), m_mesh (NULL)
{

}

dMeshNodeInfo::dMeshNodeInfo(dScene* const world)
	:dGeometryNodeInfo (world), m_mesh (NewtonMeshCreate(world->GetNewtonWorld()))
{
}

dMeshNodeInfo::dMeshNodeInfo(NewtonMesh* mesh)
	:dGeometryNodeInfo (), m_mesh (mesh)
{
	SetName ("mesh");
}

dMeshNodeInfo::dMeshNodeInfo(const dMeshNodeInfo& me)
	:dGeometryNodeInfo (me), m_mesh (NewtonMeshCreateFromMesh(me.m_mesh))
{
}

dMeshNodeInfo::~dMeshNodeInfo(void)
{
	ReplaceMesh (NULL);
}

NewtonMesh* dMeshNodeInfo::GetMesh () const 
{
	return m_mesh;
}

void dMeshNodeInfo::ReplaceMesh (NewtonMesh* mesh)
{
	if (m_mesh) {
		NewtonMeshDestroy(m_mesh);
	}
	m_mesh = mesh;
}




void dMeshNodeInfo::BeginBuild ()
{
	NewtonMeshBeginFace(m_mesh);
}


void dMeshNodeInfo::AddPolygon (int pointsCount, const neMeshInfoFlatPoint* const points, int materialID)
{
	NewtonMeshAddFace(m_mesh, pointsCount, &points[0].m_x, sizeof (neMeshInfoFlatPoint), materialID);
}

void dMeshNodeInfo::EndBuild ()
{
	NewtonMeshEndFace(m_mesh);
}

void dMeshNodeInfo::ConvertToTriangles()
{
	NewtonMeshTriangulate (m_mesh);
}

void dMeshNodeInfo::ConvertToPolygons()
{
	NewtonMeshPolygonize(m_mesh);
}
void dMeshNodeInfo::RepairTJoints ()
{
	NewtonMeshFixTJoints (m_mesh);
}


void dMeshNodeInfo::SmoothNormals (dFloat angleInRadiants)
{
	NewtonMeshCalculateVertexNormals(m_mesh, angleInRadiants);
}

void dMeshNodeInfo::BakeTransform (const dMatrix& transform)
{
	dVector scale; 
	dMatrix stretchMatrix;

//	dMatrix matrix (m_matrix * transform);
//	matrix.PolarDecomposition (m_matrix, scale, stretchMatrix);
//	matrix = dMatrix (GetIdentityMatrix(), scale, stretchMatrix);

	dMatrix tmp (m_matrix);
	dMatrix matrix (transform.Inverse4x4() * m_matrix * transform);
	matrix.PolarDecomposition (m_matrix, scale, stretchMatrix);
	matrix = transform * dMatrix (GetIdentityMatrix(), scale, stretchMatrix);

	int pointCount = NewtonMeshGetPointCount (m_mesh); 
	int pointStride = NewtonMeshGetPointStrideInByte (m_mesh) / sizeof (dFloat64);
	dFloat64* const points = NewtonMeshGetPointArray (m_mesh); 
	matrix.TransformTriplex(points, pointStride * sizeof (dFloat64), points, pointStride * sizeof (dFloat64), pointCount);


	dFloat64* const normals = NewtonMeshGetNormalArray(m_mesh); 
	dMatrix rotation (matrix.Inverse4x4().Transpose() * matrix);
	rotation.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);
	rotation.TransformTriplex(normals, pointStride * sizeof (dFloat64), normals, pointStride * sizeof (dFloat64), pointCount);

	int vertexCount = NewtonMeshGetVertexCount (m_mesh); 
	int vertexStride = NewtonMeshGetVertexStrideInByte (m_mesh) / sizeof (dFloat64);
	dFloat64* const vertex = NewtonMeshGetVertexArray (m_mesh); 
	matrix.TransformTriplex(vertex, vertexStride * sizeof (dFloat64), vertex, vertexStride * sizeof (dFloat64), vertexCount);
}

void dMeshNodeInfo::BuildFromVertexListIndexList(int faceCount, const int* const faceIndexCount, const int* faceMaterialIndex, 
	const dFloat* const vertex, int vertexStrideInBytes, const int* vertexIndex,
	const dFloat* const normal, int normalStrideInBytes, const int* normalIndex,
	const dFloat* const uv0, int uv0StrideInBytes, const int* uv0Index,
	const dFloat* const uv1, int uv1StrideInBytes, const int* uv1Index)
{
	NewtonMeshBuildFromVertexListIndexList(m_mesh, faceCount, faceIndexCount, faceMaterialIndex, 
		vertex, vertexStrideInBytes, vertexIndex,normal, normalStrideInBytes, normalIndex,
		uv0, uv0StrideInBytes, uv0Index, uv1, uv1StrideInBytes, uv1Index);
}

void dMeshNodeInfo::RemoveUnusedVertices(dScene* const world, dScene::dTreeNode* const myNode)
{
	_ASSERTE (world->GetInfoFromNode(myNode) == this);

	int vertexCount = NewtonMeshGetVertexCount(m_mesh);
	int* vertexRemapArray = new int [vertexCount];

	NewtonRemoveUnusedVertices(m_mesh, vertexRemapArray);

	for (void* ptr0 = world->GetFirstChild (myNode); ptr0; ptr0 = world->GetNextChild(myNode, ptr0)) {
		dScene::dTreeNode* node = world->GetNodeFromLink(ptr0);
		dNodeInfo* info = world->GetInfoFromNode(node);
		if (info->IsType(dGeometryNodeModifierInfo::GetRttiType())) {
			dGeometryNodeModifierInfo* modifier = (dGeometryNodeModifierInfo*) info;
			modifier->RemoveUnusedVertices(vertexRemapArray);
		}
	}

	delete vertexRemapArray;
}


void dMeshNodeInfo::Serialize (TiXmlElement* const rootNode) const
{
 	SerialiseBase(dGeometryNodeInfo, rootNode);

	TiXmlElement* pointElement = new TiXmlElement ("points");
	rootNode->LinkEndChild(pointElement);

	int bufferCount = max (NewtonMeshGetVertexCount(m_mesh), NewtonMeshGetPointCount(m_mesh));
	char* const buffer = new char[bufferCount * sizeof (dFloat) * 4 * 12];
	dFloat* const packVertex = new dFloat [4 * bufferCount];

	int vertexCount = NewtonMeshGetVertexCount (m_mesh); 
	int vertexStride = NewtonMeshGetVertexStrideInByte(m_mesh) / sizeof (dFloat64);
	const dFloat64* const vertex = NewtonMeshGetVertexArray(m_mesh); 

	// pack the vertex Array
	int* const vertexIndexList = new int [vertexCount];
	for (int i = 0; i < vertexCount; i ++) {
		packVertex[i * 4 + 0] = dFloat(vertex[i * vertexStride + 0]);
		packVertex[i * 4 + 1] = dFloat(vertex[i * vertexStride + 1]);
		packVertex[i * 4 + 2] = dFloat(vertex[i * vertexStride + 2]);
		packVertex[i * 4 + 3] = dFloat(vertex[i * vertexStride + 3]);
		vertexIndexList[i] = i;
	}
	dFloatArrayToString (packVertex, vertexCount * 4, buffer, vertexCount * sizeof (dFloat) * 4 * 12);
	
	TiXmlElement* const position = new TiXmlElement ("position");
	pointElement->LinkEndChild(position);
	position->SetAttribute("float4", vertexCount);
	position->SetAttribute("floats", buffer);

	// pack the normal array
	int pointCount = NewtonMeshGetPointCount (m_mesh); 
	int pointStride = NewtonMeshGetPointStrideInByte(m_mesh) / sizeof (dFloat64);
	const dFloat64* const normals = NewtonMeshGetNormalArray(m_mesh); 
	int* const normalIndexList = new int [pointCount];
	for (int i = 0; i < pointCount; i ++) {
		packVertex[i * 3 + 0] = dFloat(normals[i * pointStride + 0]);
		packVertex[i * 3 + 1] = dFloat(normals[i * pointStride + 1]);
		packVertex[i * 3 + 2] = dFloat(normals[i * pointStride + 2]);
	}
	int count = dPackVertexArray (packVertex, 3, 3 * sizeof (dFloat), pointCount, normalIndexList);
	dFloatArrayToString (packVertex, count * 3, buffer, pointCount * sizeof (dFloat) * 3 * 12);

	TiXmlElement* const normal = new TiXmlElement ("normal");
	pointElement->LinkEndChild(normal);
	normal->SetAttribute("float3", count);
	normal->SetAttribute("floats", buffer);

	// pack the uv0 array
	int* const uv0IndexList = new int [pointCount];
	const dFloat64* const uv0s = NewtonMeshGetUV0Array(m_mesh); 
	for (int i = 0; i < pointCount; i ++) {
		packVertex[i * 3 + 0] = dFloat(uv0s[i * pointStride + 0]);
		packVertex[i * 3 + 1] = dFloat(uv0s[i * pointStride + 1]);
		packVertex[i * 3 + 2] = 0.0f;
	}
	count = dPackVertexArray (packVertex, 3, 3 * sizeof (dFloat), pointCount, uv0IndexList);
	for (int i = 0; i < pointCount; i ++) {
		packVertex[i * 2 + 0] = packVertex[i * 3 + 0];
		packVertex[i * 2 + 1] = packVertex[i * 3 + 1];
	}
	dFloatArrayToString (packVertex, count * 2, buffer, pointCount * sizeof (dFloat) * 3 * 12);

	TiXmlElement* const uv0 = new TiXmlElement ("uv0");
	pointElement->LinkEndChild(uv0);
	uv0->SetAttribute("float2", count);
	uv0->SetAttribute("floats", buffer);

	// pack the uv1 array
	int* const uv1IndexList = new int [pointCount];
	const dFloat64* const uv1s = NewtonMeshGetUV1Array(m_mesh); 
	for (int i = 0; i < pointCount; i ++) {
		packVertex[i * 3 + 0] = dFloat(uv1s[i * pointStride + 0]);
		packVertex[i * 3 + 1] = dFloat(uv1s[i * pointStride + 1]);
		packVertex[i * 3 + 2] = 0.0f;
	}
	count = dPackVertexArray (packVertex, 3, 3 * sizeof (dFloat), pointCount, uv1IndexList);
	for (int i = 0; i < pointCount; i ++) {
		packVertex[i * 2 + 0] = packVertex[i * 3 + 0];
		packVertex[i * 2 + 1] = packVertex[i * 3 + 1];
	}
	dFloatArrayToString (packVertex, count * 2, buffer, pointCount * sizeof (dFloat) * 3 * 12);

	TiXmlElement* const uv1 = new TiXmlElement ("uv1");
	pointElement->LinkEndChild(uv1);
	uv1->SetAttribute("float2", count);
	uv1->SetAttribute("floats", buffer);

	// save the polygon array
	int faceCount = NewtonMeshGetTotalFaceCount (m_mesh); 
	int indexCount = NewtonMeshGetTotalIndexCount (m_mesh); 

	int* const faceArray = new int [faceCount];
	void** const indexArray = new void* [indexCount];
	int* const materialIndexArray = new int [faceCount];
	int* const remapedIndexArray = new int [indexCount];

	NewtonMeshGetFaces (m_mesh, faceArray, materialIndexArray, indexArray); 

	// save the faces vertex Count
	dIntArrayToString (faceArray, faceCount, buffer, pointCount * sizeof (dFloat) * 3 * 12);
	TiXmlElement* const polygons = new TiXmlElement ("polygons");
	rootNode->LinkEndChild(polygons);
	polygons->SetAttribute("count", faceCount);
	polygons->SetAttribute("faceIndexCount", buffer);

	dIntArrayToString (materialIndexArray, faceCount, buffer, pointCount * sizeof (dFloat) * 3 * 12);
	TiXmlElement* const faceMaterial = new TiXmlElement ("faceMaterial");
	polygons->LinkEndChild(faceMaterial);
	faceMaterial->SetAttribute("index", buffer);

	for (int i = 0; i < indexCount; i ++) {
		int index = NewtonMeshGetVertexIndex (m_mesh, indexArray[i]);
		remapedIndexArray[i] = vertexIndexList[index];
	}
	dIntArrayToString (remapedIndexArray, indexCount, buffer, pointCount * sizeof (dFloat) * 3 * 12);
	TiXmlElement* const positionIndex = new TiXmlElement ("position");
	polygons->LinkEndChild(positionIndex);
	positionIndex->SetAttribute("index", buffer);


	for (int i = 0; i < indexCount; i ++) {
		int index = NewtonMeshGetPointIndex(m_mesh, indexArray[i]);
		remapedIndexArray[i] = normalIndexList[index];
	}
	dIntArrayToString (remapedIndexArray, indexCount, buffer, pointCount * sizeof (dFloat) * 3 * 12);
	TiXmlElement* const normalIndex = new TiXmlElement ("normal");
	polygons->LinkEndChild(normalIndex);
	normalIndex->SetAttribute("index", buffer);

	for (int i = 0; i < indexCount; i ++) {
		int index = NewtonMeshGetPointIndex(m_mesh, indexArray[i]);
		remapedIndexArray[i] = uv0IndexList[index];
	}
	dIntArrayToString (remapedIndexArray, indexCount, buffer, pointCount * sizeof (dFloat) * 3 * 12);
	TiXmlElement* const uv0Index = new TiXmlElement ("uv0");
	polygons->LinkEndChild(uv0Index);
	uv0Index->SetAttribute("index", buffer);

	for (int i = 0; i < indexCount; i ++) {
		int index = NewtonMeshGetPointIndex(m_mesh, indexArray[i]);
		remapedIndexArray[i] = uv1IndexList[index];
	}
	dIntArrayToString (remapedIndexArray, indexCount, buffer, pointCount * sizeof (dFloat) * 3 * 12);
	TiXmlElement* const uv1Index = new TiXmlElement ("uv1");
	polygons->LinkEndChild(uv1Index);
	uv1Index->SetAttribute("index", buffer);

	delete[] remapedIndexArray;
	delete[] faceArray;
	delete[] indexArray;
	delete[] materialIndexArray;
	delete[] uv1IndexList;
	delete[] uv0IndexList;
	delete[] normalIndexList;
	delete[] vertexIndexList;
	delete[] packVertex;
	delete[] buffer;
}

bool dMeshNodeInfo::Deserialize (TiXmlElement* const rootNode, int revisionNumber) 
{
	DeserialiseBase(dGeometryNodeInfo, rootNode, revisionNumber);

	// load all the vertexData
	TiXmlElement* const pointElement = (TiXmlElement*) rootNode->FirstChild ("points");

	int positionCount;
	TiXmlElement* const positionsElement = (TiXmlElement*) pointElement->FirstChild ("position");
	positionsElement->Attribute("float4", &positionCount);
	dFloat* const positions = new dFloat[4 * positionCount];
	dStringToFloatArray (positionsElement->Attribute("floats"), positions, 4 * positionCount);

	int normalCount;
	TiXmlElement* normalsElement = (TiXmlElement*) pointElement->FirstChild ("normal");
	normalsElement->Attribute("float3", &normalCount);
	dFloat* const normals = new dFloat[3 * normalCount];
	dStringToFloatArray (normalsElement->Attribute("floats"), normals, 3 * normalCount);

	int uv0Count;
	TiXmlElement* uv0Element = (TiXmlElement*) pointElement->FirstChild ("uv0");
	uv0Element->Attribute("float2", &uv0Count);
	dFloat* const uv0 = new dFloat[2 * uv0Count];
	dStringToFloatArray (uv0Element->Attribute("floats"), uv0, 2 * uv0Count);

	int uv1Count;
	TiXmlElement* uv1Element = (TiXmlElement*) pointElement->FirstChild ("uv1");
	uv1Element->Attribute("float2", &uv1Count);
	dFloat* const uv1 = new dFloat[2 * uv1Count];
	dStringToFloatArray (uv1Element->Attribute("floats"), uv1, 2 * uv1Count);

	//load face informations
	TiXmlElement* polygonsElement = (TiXmlElement*) rootNode->FirstChild ("polygons");

	int faceCount;
	polygonsElement->Attribute("count", &faceCount);
	int* const faceIndexCount = new int[faceCount];
	dStringToIntArray (polygonsElement->Attribute("faceIndexCount"), faceIndexCount, faceCount);

	int* const faceMaterials = new int [faceCount]; 
	TiXmlElement* materialElement = (TiXmlElement*) polygonsElement->FirstChild ("faceMaterial");
	dStringToIntArray (materialElement->Attribute("index"), faceMaterials, faceCount);

	int indexCount = 0;
	for (int i = 0; i < faceCount; i ++) {
		indexCount += faceIndexCount[i];
	}

	int* const positionVertexIndex = new int [indexCount]; 
	TiXmlElement* const positionVertexIndexElement = (TiXmlElement*) polygonsElement->FirstChild ("position");
	dStringToIntArray (positionVertexIndexElement->Attribute("index"), positionVertexIndex, indexCount);

	int* const normalVertexIndex = new int [indexCount]; 
	TiXmlElement* const normalVertexIndexElement = (TiXmlElement*) polygonsElement->FirstChild ("normal");
	dStringToIntArray (normalVertexIndexElement->Attribute("index"), normalVertexIndex, indexCount);

	int* const uv0VertexIndex = new int [indexCount]; 
	TiXmlElement* const uv0VertexIndexElement = (TiXmlElement*) polygonsElement->FirstChild ("uv0");
	dStringToIntArray (uv0VertexIndexElement->Attribute("index"), uv0VertexIndex, indexCount);

	int* const uv1VertexIndex = new int [indexCount]; 
	TiXmlElement* const uv1VertexIndexElement = (TiXmlElement*) polygonsElement->FirstChild ("uv1");
	dStringToIntArray (uv1VertexIndexElement->Attribute("index"), uv1VertexIndex, indexCount);

	BuildFromVertexListIndexList(faceCount, faceIndexCount, faceMaterials, 
								&positions[0], 4 * sizeof (dFloat), positionVertexIndex,
								&normals[0], 3 * sizeof (dFloat), normalVertexIndex,
								&uv0[0], 2 * sizeof (dFloat), uv0VertexIndex,
								&uv1[0], 2 * sizeof (dFloat), uv1VertexIndex);


	delete uv1VertexIndex;
	delete uv0VertexIndex;
	delete normalVertexIndex;
	delete positionVertexIndex;
	delete faceMaterials;
	delete faceIndexCount;
	delete uv1;	
	delete uv0;	
	delete normals;	
	delete positions;	
	return true;
}


void dMeshNodeInfo::SerializeBinary (FILE* const file) 
{
	_ASSERTE (0);
/*
	fprintf (file, "%s\n%s\n", GetClassName(), GetName());

	fwrite (&m_matrix[0][0], 1, sizeof (dMatrix), file);

	int pointCount = NewtonMeshGetPointCount (m_mesh); 
	int pointStride = NewtonMeshGetPointStrideInByte (m_mesh) / sizeof (dFloat);
	dFloat* const points = NewtonMeshGetPointArray (m_mesh); 

	dFloat* packVertex = new dFloat [pointCount * 3];

	// pack the vertex Array
	int* vertexIndexList = new int [pointCount];
	for (int i = 0; i < pointCount; i ++) {
		packVertex[i * 3 + 0] = points[i * pointStride + 0];
		packVertex[i * 3 + 1] = points[i * pointStride + 1];
		packVertex[i * 3 + 2] = points[i * pointStride + 2];
	}
	int count = dPackVertexArray (packVertex, 3, 3 * sizeof (dFloat), pointCount, vertexIndexList);
//	dFloatArrayToString (packVertex, count * 3, buffer, pointCount * sizeof (dFloat) * 3 * 12);
//	TiXmlElement* position = new TiXmlElement ("position");
//	pointElement->LinkEndChild(position);
//	position->SetAttribute("float3", count);
//	position->SetAttribute("floats", buffer);
	fwrite (&count, 1, sizeof (int), file);
	fwrite (&packVertex, count, 3 * sizeof (dFloat), file);

	// pack the normal array
	int* normalIndexList = new int [pointCount];
	for (int i = 0; i < pointCount; i ++) {
		packVertex[i * 3 + 0] = points[i * pointStride + 3];
		packVertex[i * 3 + 1] = points[i * pointStride + 4];
		packVertex[i * 3 + 2] = points[i * pointStride + 5];
	}
	count = dPackVertexArray (packVertex, 3, 3 * sizeof (dFloat), pointCount, normalIndexList);
//	dFloatArrayToString (packVertex, count * 3, buffer, pointCount * sizeof (dFloat) * 3 * 12);
//	TiXmlElement* normal = new TiXmlElement ("normal");
//	pointElement->LinkEndChild(normal);
//	normal->SetAttribute("float3", count);
//	normal->SetAttribute("floats", buffer);
	fwrite (&count, 1, sizeof (int), file);
	fwrite (&packVertex, count, 3 * sizeof (dFloat), file);


	// pack the uv0 array
	int* uv0IndexList = new int [pointCount];
	for (int i = 0; i < pointCount; i ++) {
		packVertex[i * 3 + 0] = points[i * pointStride + 6];
		packVertex[i * 3 + 1] = points[i * pointStride + 7];
		packVertex[i * 3 + 2] = 0.0f;
	}
	count = dPackVertexArray (packVertex, 3, 3 * sizeof (dFloat), pointCount, uv0IndexList);
	for (int i = 0; i < pointCount; i ++) {
		packVertex[i * 2 + 0] = packVertex[i * 3 + 0];
		packVertex[i * 2 + 1] = packVertex[i * 3 + 1];
	}
//	dFloatArrayToString (packVertex, count * 2, buffer, pointCount * sizeof (dFloat) * 3 * 12);
//	TiXmlElement* uv0 = new TiXmlElement ("uv0");
//	pointElement->LinkEndChild(uv0);
//	uv0->SetAttribute("float2", count);
//	uv0->SetAttribute("floats", buffer);
	fwrite (&count, 1, sizeof (int), file);
	fwrite (&packVertex, count, 2 * sizeof (dFloat), file);


	// pack the uv1 array
	int* uv1IndexList = new int [pointCount];
	for (int i = 0; i < pointCount; i ++) {
		packVertex[i * 3 + 0] = points[i * pointStride + 8];
		packVertex[i * 3 + 1] = points[i * pointStride + 9];
		packVertex[i * 3 + 2] = 0.0f;
	}
	count = dPackVertexArray (packVertex, 3, 3 * sizeof (dFloat), pointCount, uv1IndexList);
	for (int i = 0; i < pointCount; i ++) {
		packVertex[i * 2 + 0] = packVertex[i * 3 + 0];
		packVertex[i * 2 + 1] = packVertex[i * 3 + 1];
	}
//	dFloatArrayToString (packVertex, count * 2, buffer, pointCount * sizeof (dFloat) * 3 * 12);
//	TiXmlElement* uv1 = new TiXmlElement ("uv1");
//	pointElement->LinkEndChild(uv1);
//	uv1->SetAttribute("float2", count);
//	uv1->SetAttribute("floats", buffer);
	fwrite (&count, 1, sizeof (int), file);
	fwrite (&packVertex, count, 2 * sizeof (dFloat), file);


	// save the polygon array
	int faceCount = NewtonMeshGetTotalFaceCount (m_mesh); 
	int indexCount = NewtonMeshGetTotalIndexCount (m_mesh); 

	int* faceArray = new int [faceCount];
	int* indexArray = new int [indexCount];
	int* materialIndexArray = new int [faceCount];
	int* remapedIndexArray = new int [indexCount];

	NewtonMeshGetFaces (m_mesh, faceArray, materialIndexArray, indexArray); 

	// save the faces vertex Count
//	dIntArrayToString (faceArray, faceCount, buffer, pointCount * sizeof (dFloat) * 3 * 12);
//	TiXmlElement* polygons = new TiXmlElement ("polygons");
//	node->LinkEndChild(polygons);
//	polygons->SetAttribute("count", faceCount);
//	polygons->SetAttribute("faceIndexCount", buffer);
	fwrite (&faceCount, 1, sizeof (int), file);
	fwrite (faceArray, faceCount, sizeof (int), file);

//	dIntArrayToString (materialIndexArray, faceCount, buffer, pointCount * sizeof (dFloat) * 3 * 12);
//	TiXmlElement* faceMaterial = new TiXmlElement ("faceMaterial");
//	polygons->LinkEndChild(faceMaterial);
//	faceMaterial->SetAttribute("index", buffer);
	fwrite (materialIndexArray, faceCount, sizeof (int), file);

	for (int i = 0; i < indexCount; i ++) {
		int index = indexArray[i];
		remapedIndexArray[i] = vertexIndexList[index];
	}
//	dIntArrayToString (remapedIndexArray, indexCount, buffer, pointCount * sizeof (dFloat) * 3 * 12);
//	TiXmlElement* positionIndex = new TiXmlElement ("position");
//	polygons->LinkEndChild(positionIndex);
//	positionIndex->SetAttribute("index", buffer);
	fwrite (&indexCount, 1, sizeof (int), file);
	fwrite (remapedIndexArray, indexCount, sizeof (int), file);

	for (int i = 0; i < indexCount; i ++) {
		int index = indexArray[i];
		remapedIndexArray[i] = normalIndexList[index];
	}
//	dIntArrayToString (remapedIndexArray, indexCount, buffer, pointCount * sizeof (dFloat) * 3 * 12);
//	TiXmlElement* normalIndex = new TiXmlElement ("normal");
//	polygons->LinkEndChild(normalIndex);
//	normalIndex->SetAttribute("index", buffer);
	fwrite (&indexCount, 1, sizeof (int), file);
	fwrite (remapedIndexArray, indexCount, sizeof (int), file);

	for (int i = 0; i < indexCount; i ++) {
		int index = indexArray[i];
		remapedIndexArray[i] = uv0IndexList[index];
	}
//	dIntArrayToString (remapedIndexArray, indexCount, buffer, pointCount * sizeof (dFloat) * 3 * 12);
//	TiXmlElement* uv0Index = new TiXmlElement ("uv0");
//	polygons->LinkEndChild(uv0Index);
//	uv0Index->SetAttribute("index", buffer);
	fwrite (&indexCount, 1, sizeof (int), file);
	fwrite (remapedIndexArray, indexCount, sizeof (int), file);


	for (int i = 0; i < indexCount; i ++) {
		int index = indexArray[i];
		remapedIndexArray[i] = uv1IndexList[index];
	}
//	dIntArrayToString (remapedIndexArray, indexCount, buffer, pointCount * sizeof (dFloat) * 3 * 12);
//	TiXmlElement* uv1Index = new TiXmlElement ("uv1");
//	polygons->LinkEndChild(uv1Index);
//	uv1Index->SetAttribute("index", buffer);
	fwrite (&indexCount, 1, sizeof (int), file);
	fwrite (remapedIndexArray, indexCount, sizeof (int), file);


	delete[] remapedIndexArray;
	delete[] faceArray;
	delete[] indexArray;
	delete[] materialIndexArray;
	delete[] uv1IndexList;
	delete[] uv0IndexList;
	delete[] normalIndexList;
	delete[] vertexIndexList;
	delete[] packVertex;
*/
}


void dMeshNodeInfo::CalculateOOBBGizmo (const dMatrix& matrix, dVector& p0, dVector& p1) const
{
	p0 = dVector ( 1.0e10f,  1.0e10f,  1.0e10f, 1.0f);
	p1 = dVector (-1.0e10f, -1.0e10f, -1.0e10f, 1.0f);

	dMatrix tranform (m_matrix * matrix) ;

	int stride = NewtonMeshGetVertexStrideInByte(m_mesh) / sizeof(dFloat64);
	dFloat64* const vertexList = NewtonMeshGetVertexArray(m_mesh);

	for (void* vertex = NewtonMeshGetFirstVertex(m_mesh); vertex; vertex = NewtonMeshGetNextVertex(m_mesh, vertex)) {
		int index = NewtonMeshGetVertexIndex(m_mesh, vertex) * stride;

		dVector p (dFloat(vertexList[index + 0]), dFloat(vertexList[index + 1]), dFloat(vertexList[index + 2]), 1.0f);
		p = tranform.TransformVector(p);
		p0.m_x = min (p.m_x, p0.m_x);
		p0.m_y = min (p.m_y, p0.m_y);
		p0.m_z = min (p.m_z, p0.m_z);

		p1.m_x = max (p.m_x, p1.m_x);
		p1.m_y = max (p.m_y, p1.m_y);
		p1.m_z = max (p.m_z, p1.m_z);
	}
}

dFloat dMeshNodeInfo::RayCast (const dVector& p0, const dVector& p1) const
{
	_ASSERTE (0);
	return 0;
/*
	dVector q0 (m_matrix.UntransformVector(p0));
	dVector q1 (m_matrix.UntransformVector(p1));

	//	int vertexCount = NewtonMeshGetVertexCount(m_mesh);
	int strideInBytes = NewtonMeshGetVertexStrideInByte(m_mesh);
	dFloat64* const vertexList = NewtonMeshGetVertexArray(m_mesh);
	dFloat t = 1.2f;
int xxx = 0;
int xxx2 = 0;
	for (void* face = NewtonMeshGetFirstFace (m_mesh); face; face = NewtonMeshGetNextFace (m_mesh, face)) {
		if (!NewtonMeshIsFaceOpen (m_mesh, face)) {

xxx ++;
			int indices[1024];
			int vertexCount = NewtonMeshGetFaceIndexCount (m_mesh, face);
			NewtonMeshGetFaceIndices (m_mesh, face, indices);

			dFloat t1 = dPolygonRayCast (q0, q1, vertexCount, vertexList, strideInBytes, indices);
			if (t1 < t) {
xxx2 = xxx;
if (xxx2 == 40276)
xxx2 = xxx;
				t = t1;		
			}
		}
	}
	return t;
*/
}


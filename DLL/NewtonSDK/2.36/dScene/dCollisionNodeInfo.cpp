/////////////////////////////////////////////////////////////////////////////
// Name:        dCollisionNodeInfo.h
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
#include "dDrawUtils.h"
#include "dCollisionNodeInfo.h"


D_IMPLEMENT_CLASS_NODE(dCollisionNodeInfo);

dCollisionNodeInfo::dCollisionNodeInfo(dScene* const world) 
	:dNodeInfo (), 
	m_matrix (GetIdentityMatrix()),
	m_geometricInertia(0.0f, 0.0f, 0.0f, 0.0f),
	m_geometricCenterAndVolume(0.0f, 0.0f, 0.0f, 0.0f),
	m_shapeID(0)
{
	SetName ("collision");
}

dCollisionNodeInfo::dCollisionNodeInfo()
	:dNodeInfo (), 
	m_matrix (GetIdentityMatrix()), 
	m_geometricInertia(0.0f, 0.0f, 0.0f, 0.0f),
	m_geometricCenterAndVolume(0.0f, 0.0f, 0.0f, 0.0f),
	m_shapeID(0)
{
	SetName ("collision");
}

dCollisionNodeInfo::~dCollisionNodeInfo(void)
{
}


dMatrix dCollisionNodeInfo::GetTransform () const
{
//	dMatrix matrix (dPitchMatrix(m_euler.m_x) * dYawMatrix(m_euler.m_y) * dRollMatrix(m_euler.m_z));
//	matrix.m_posit = m_position;
//	return dMatrix (matrix, m_scale, m_eigenScaleAsis);
	return m_matrix;
}

void dCollisionNodeInfo::SetTransform (const dMatrix& matrix)
{
//	dMatrix transform;
//	matrix.PolarDecomposition(transform, m_scale, m_eigenScaleAsis);
//	m_position = matrix.m_posit;
//	m_euler = transform.GetXYZ_EulerAngles ();
	m_matrix = matrix;
}


void dCollisionNodeInfo::BakeTransform (const dMatrix& transform)
{
	SetTransform (transform.Inverse4x4() * GetTransform() * transform);
}

void dCollisionNodeInfo::CalculateInertiaGeometry (dScene* world, dVector& inertia, dVector& centerOfMass) const
{
	inertia = dVector (0.0f, 0.0f, 0.0f, 0.0f);
	centerOfMass = dVector (0.0f, 0.0f, 0.0f, 0.0f);
}


void dCollisionNodeInfo::SetShapeId (int id)
{
	m_shapeID = id;
}

int dCollisionNodeInfo::GetShapeId () const
{
	return m_shapeID;
}

void dCollisionNodeInfo::SetInertiaGeometry (const dVector& inertia)
{
	m_geometricInertia = inertia;
}

const dVector& dCollisionNodeInfo::GetInertiaGeometry () const
{
	return m_geometricInertia;
}

void dCollisionNodeInfo::SetCenterOfMassAndVolume (const dVector& comAndVolume)
{
	m_geometricCenterAndVolume = comAndVolume;
}

const dVector& dCollisionNodeInfo::GetCenterMassAndVolume () const
{
	return m_geometricCenterAndVolume;
}


void dCollisionNodeInfo::CalculateGeometryProperies (NewtonCollision* shape, dVector& inertia, dVector& centerOfMass) const
{
	NewtonConvexCollisionCalculateInertialMatrix (shape, &inertia[0], &centerOfMass[0]);	
	centerOfMass.m_w = NewtonConvexCollisionCalculateVolume (shape);
	inertia.m_w = centerOfMass.m_w;
}


bool dCollisionNodeInfo::Deserialize (TiXmlElement* const rootNode, int revisionNumber) 
{
	DeserialiseBase(dNodeInfo, rootNode, revisionNumber);

	rootNode->Attribute("shapeId", &m_shapeID);

	TiXmlElement* childNode = (TiXmlElement*) rootNode->FirstChild ("offsetMatrix");

	dStringToFloatArray (childNode->Attribute("float16"), &m_matrix[0][0], 16);

	childNode = (TiXmlElement*) rootNode->FirstChild ("geometricInertia");
	dStringToFloatArray (childNode->Attribute("float4"), &m_geometricInertia[0], 4);

	childNode = (TiXmlElement*) rootNode->FirstChild ("geometricCenterAndVolume");
	dStringToFloatArray (childNode->Attribute("float4"), &m_geometricCenterAndVolume[0], 4);

	return true;
}



void dCollisionNodeInfo::Serialize (TiXmlElement* const rootNode) const
{
	SerialiseBase(dNodeInfo, rootNode);

	rootNode->SetAttribute("shapeId", m_shapeID);

	char tmp[1024];
	TiXmlElement* dataNode = new TiXmlElement ("offsetMatrix");
	rootNode->LinkEndChild(dataNode);
	dFloatArrayToString (&m_matrix[0][0], 16, tmp, sizeof (tmp));
	dataNode->SetAttribute("float16", tmp);

	dataNode = new TiXmlElement ("geometricInertia");
	rootNode->LinkEndChild(dataNode);
	dFloatArrayToString (&m_geometricInertia[0], 4, tmp, sizeof (tmp));
	dataNode->SetAttribute("float4", tmp);

	dataNode = new TiXmlElement ("geometricCenterAndVolume");
	rootNode->LinkEndChild(dataNode);
	dFloatArrayToString (&m_geometricCenterAndVolume[0], 4, tmp, sizeof (tmp));
	dataNode->SetAttribute("float4", tmp);
}



void dCollisionNodeInfo::SerializeBinary (FILE* const file) 
{
_ASSERTE (0);
	fprintf (file, "%s\n%s\n", GetClassName(), GetName());
	_ASSERTE (0);
//	fwrite (&m_matrix[0][0], 1, sizeof (dMatrix), file);
}


void dCollisionNodeInfo::DrawWireFrame(dScene* const world, dScene::dTreeNode* const myNode, const dVector& color) const
{
	_ASSERTE (world->GetInfoFromNode(myNode) == this);
	_ASSERTE (myNode == world->Find(GetUniqueID()));

_ASSERTE (0);
/*
	dScene::dTreeNode* geomNode = world->FindChildByType(myNode, dGeometryNodeInfo::GetRttiType());
	if (geomNode) {

		glPushMatrix();
		dMatrix matrix (GetTransform());
		//glMultMatrix(&matrix[0][0]);
		glLoadMatrix(&matrix[0][0]);

		dGeometryNodeInfo* gemInfo = (dGeometryNodeInfo*) world->GetInfoFromNode(geomNode);
		gemInfo->DrawWireFrame(world, geomNode, color, workBuffer, worlfBufferInBytes);

		glPopMatrix();
	}
*/
}


// draw scene in solid wire frame mode
void dCollisionNodeInfo::DrawSolidWireFrame(dScene* const world, dScene::dTreeNode* const myNode, const dVector& color) const
{
	_ASSERTE (world->GetInfoFromNode(myNode) == this);
	_ASSERTE (myNode == world->Find(GetUniqueID()));
_ASSERTE (0);
/*
	
	dScene::dTreeNode* geomNode = world->FindChildByType(myNode, dGeometryNodeInfo::GetRttiType());
	if (geomNode) {

		glPushMatrix();
		dMatrix matrix (GetTransform());
		//glMultMatrix(&matrix[0][0]);
		glLoadMatrix(&matrix[0][0]);

		dGeometryNodeInfo* gemInfo = (dGeometryNodeInfo*) world->GetInfoFromNode(geomNode);
		gemInfo->DrawSolidWireFrame(world, geomNode, color, workBuffer, worlfBufferInBytes);

		glPopMatrix();
	}
*/
}


// draw scene in Gouraud shaded normal textured mode 
void dCollisionNodeInfo::DrawGouraudShaded(dScene* const world, dScene::dTreeNode* const myNode, const dVector& color) const
{
	_ASSERTE (world->GetInfoFromNode(myNode) == this);
	_ASSERTE (myNode == world->Find(GetUniqueID()));

_ASSERTE (0);
/*
	dScene::dTreeNode* geomNode = world->FindChildByType(myNode, dGeometryNodeInfo::GetRttiType());
	if (geomNode) {

		glPushMatrix();
		dMatrix matrix (GetTransform());
		//glMultMatrix(&matrix[0][0]);
		glLoadMatrix(&matrix[0][0]);

		dGeometryNodeInfo* gemInfo = (dGeometryNodeInfo*) world->GetInfoFromNode(geomNode);
		gemInfo->DrawGouraudShaded(world, geomNode, color, workBuffer, worlfBufferInBytes);

		glPopMatrix();
	}
*/
}




dCollisionNodeInfo::dGizmoHandle dCollisionNodeInfo::GetHighlightedGizmoHandle(
	dScene* world, 
	dScene::dTreeNode* myNode, 
	const dMatrix& coordinaSystem,
	const dVector& screenPosition, 
	dGizmoMode mode, 
	dFloat size) const
{
_ASSERTE (0);
return m_noHandle;
/*
	dGizmoHandle handle = m_noHandle;

	dVector l0 (ScreenToWorld(dVector (screenPosition.m_x, screenPosition.m_y, 0.0f, 1.0f)));
	dVector l1 (ScreenToWorld(dVector (screenPosition.m_x, screenPosition.m_y, 1.0f, 1.0f)));

	dVector euler (GetEulers ());
	dMatrix matrix (euler.m_x, euler.m_y, euler.m_z, GetPosition());
	matrix = matrix * coordinaSystem;
	int indices[] = {0, 1, 2, 3};
	switch (mode)
	{
		case dNodeInfo::m_translation:
		{
			dVector p[4];
			dFloat planeSize = size * NE_TRANSLATE_LINEAR_FACTOR;
			p[0] = dVector (planeSize * 0.0f, planeSize * 0.0f, 0.0f, 0.0f);
			p[1] = dVector (planeSize * 1.0f, planeSize * 0.0f, 0.0f, 0.0f);
			p[2] = dVector (planeSize * 1.0f, planeSize * 1.0f, 0.0f, 0.0f);
			p[3] = dVector (planeSize * 0.0f, planeSize * 1.0f, 0.0f, 0.0f);

			dVector localL0 (matrix.UntransformVector(l0)); 
			dVector localL1 (matrix.UntransformVector(l1)); 
			dFloat t = dPolygonRayCast (localL0, localL1, 4, &p[0].m_x, sizeof (dVector), indices);
			if (t < 1.0f) {
				handle = m_xyHandle;
			}

			dMatrix matrix1 (dPitchMatrix(90.0f * 3.1416f/180.0f) * matrix);
			localL0 = matrix1.UntransformVector(l0); 
			localL1 = matrix1.UntransformVector(l1); 
			dFloat t1 = dPolygonRayCast (localL0, localL1, 4, &p[0].m_x, sizeof (dVector), indices);
			if (t1 < t) {
				t = t1;
				handle = m_zxHandle;
			}

			matrix1 = dYawMatrix(-90.0f * 3.1416f/180.0f) * matrix;
			localL0 = matrix1.UntransformVector(l0); 
			localL1 = matrix1.UntransformVector(l1); 
			t1 = dPolygonRayCast (localL0, localL1, 4, &p[0].m_x, sizeof (dVector), indices);
			if (t1 < t) {
				t = t1;
				handle = m_yzHandle;
			}


			dFloat radius = NE_TRANSLATE_ARROW_RADIUS_FACTOR * size;
			dFloat height = NE_TRANSLATE_ARROW_HEIGHT_FACTOR * size;
			dVector points[128 * 3];  
			dVector normals[128];  
			int count = CreateCone (points, normals, 8, radius, height, 128);
			matrix1 = GetIdentityMatrix();
			matrix1.m_posit.m_x = size;

			matrix1 = matrix1 * matrix;
			localL0 = matrix1.UntransformVector(l0); 
			localL1 = matrix1.UntransformVector(l1); 
			for (int i = 0; i < count; i ++) {
				t1 = dPolygonRayCast (localL0, localL1, 3, &points[i * 3].m_x, sizeof (dVector), indices);
				if (t1 < t) {
					t = t1;
					handle = m_xHandle;
					break;
				}
			}

			matrix1 = dRollMatrix(90.0f * 3.141592f / 180.0f);
			matrix1.m_posit.m_y = size;
			matrix1 = matrix1 * matrix;
			localL0 = matrix1.UntransformVector(l0); 
			localL1 = matrix1.UntransformVector(l1); 
			for (int i = 0; i < count; i ++) {
				t1 = dPolygonRayCast (localL0, localL1, 3, &points[i * 3].m_x, sizeof (dVector), indices);
				if (t1 < t) {
					t = t1;
					handle = m_yHandle;
					break;
				}
			}

			matrix1 = dYawMatrix(-90.0f * 3.141592f / 180.0f);
			matrix1.m_posit.m_z = size;
			matrix1 = matrix1 * matrix;
			localL0 = matrix1.UntransformVector(l0); 
			localL1 = matrix1.UntransformVector(l1); 
			for (int i = 0; i < count; i ++) {
				t1 = dPolygonRayCast (localL0, localL1, 3, &points[i * 3].m_x, sizeof (dVector), indices);
				if (t1 < t) {
					t = t1;
					handle = m_zHandle;
					break;
				}
			}
			break;
		}

		case dNodeInfo::m_rotation:
		{
			dVector localL0; 
			dVector localL1; 
			dVector points[128 * 3];  
			dVector normals[128];  
			dFloat radius = NE_ROTATE_RADIUS_FACTOR * size;
			dFloat height = NE_ROTATE_HEIGHT_FACTOR * size;

			dFloat t = 1.2f;
			int count = CreateCylinder(points, normals, 8, radius, height, 128);

			dMatrix matrix1 = GetIdentityMatrix();
			matrix1.m_posit.m_x = size * NE_ROTATE_GIZMO_LOCATION_FACTOR;
			matrix1 = matrix1 * matrix;
			localL0 = matrix1.UntransformVector(l0); 
			localL1 = matrix1.UntransformVector(l1); 
			for (int i = 0; i < count; i ++) {
				dFloat t1 = dPolygonRayCast (localL0, localL1, 3, &points[i * 3].m_x, sizeof (dVector), indices);
				if (t1 < t) {
					t = t1;
					handle = m_xHandle;
					break;
				}
			}

			matrix1 = dRollMatrix(90.0f * 3.141592f / 180.0f);
			matrix1.m_posit.m_y = size * NE_ROTATE_GIZMO_LOCATION_FACTOR;
			matrix1 = matrix1 * matrix;
			localL0 = matrix1.UntransformVector(l0); 
			localL1 = matrix1.UntransformVector(l1); 
			for (int i = 0; i < count; i ++) {
				dFloat t1 = dPolygonRayCast (localL0, localL1, 3, &points[i * 3].m_x, sizeof (dVector), indices);
				if (t1 < t) {
					t = t1;
					handle = m_yHandle;
					break;
				}
			}


			matrix1 = dYawMatrix(-90.0f * 3.141592f / 180.0f);
			matrix1.m_posit.m_z = size * NE_ROTATE_GIZMO_LOCATION_FACTOR;
			matrix1 = matrix1 * matrix;
			localL0 = matrix1.UntransformVector(l0); 
			localL1 = matrix1.UntransformVector(l1); 
			for (int i = 0; i < count; i ++) {
				dFloat t1 = dPolygonRayCast (localL0, localL1, 3, &points[i * 3].m_x, sizeof (dVector), indices);
				if (t1 < t) {
					t = t1;
					handle = m_zHandle;
					break;
				}
			}
			break;
		}


		case dNodeInfo::m_scale:
		{
			int indices[] = {0, 1, 2, 3};
			dVector p[4];
			dVector q[4];
			dVector localL0 (matrix.UntransformVector(l0)); 
			dVector localL1 (matrix.UntransformVector(l1)); 

			dFloat t = 1.2f;

			p[0] = dVector (0.0f, 0.0f, 0.0f, 0.0f);
			for (int i = 0; i < 3; i ++) {
				dVector tmp (0.0f, 0.0f, 0.0f, 0.0);
				p[2] = tmp;
				tmp[i] = size * NE_SCALE_UNIFORM_SCALE_FACTOR;
				p[1] = tmp;
				q[i] = tmp;
				p[2][(i + 1) % 3] = size * NE_SCALE_UNIFORM_SCALE_FACTOR;

				dFloat t1 = dPolygonRayCast (localL0, localL1, 3, &p[0].m_x, sizeof (dVector), indices);
				if (t1 < t) {
					t = t1;
					handle = m_xyzHandle;
				}
			}

			if (t > 1.0f) {
				dFloat t1 = dPolygonRayCast (localL0, localL1, 3, &q[0].m_x, sizeof (dVector), indices);
				if (t1 < t) {
					t = t1;
					handle = m_xyzHandle;
				}
			}

			dFloat planeSize = size * NE_TRANSLATE_LINEAR_FACTOR;
			p[0] = dVector (planeSize * 0.0f, planeSize * 0.0f, 0.0f, 0.0f);
			p[1] = dVector (planeSize * 1.0f, planeSize * 0.0f, 0.0f, 0.0f);
			p[2] = dVector (planeSize * 1.0f, planeSize * 1.0f, 0.0f, 0.0f);
			p[3] = dVector (planeSize * 0.0f, planeSize * 1.0f, 0.0f, 0.0f);

			localL0 = matrix.UntransformVector(l0); 
			localL1 = matrix.UntransformVector(l1); 
			dFloat t1 = dPolygonRayCast (localL0, localL1, 4, &p[0].m_x, sizeof (dVector), indices);
			if (t1 < t) {
				t = t1;
				handle = m_xyHandle;
			}

			dMatrix matrix1 (dPitchMatrix(90.0f * 3.1416f/180.0f) * matrix);
			localL0 = matrix1.UntransformVector(l0); 
			localL1 = matrix1.UntransformVector(l1); 
			t1 = dPolygonRayCast (localL0, localL1, 4, &p[0].m_x, sizeof (dVector), indices);
			if (t1 < t) {
				t = t1;
				handle = m_zxHandle;
			}

			matrix1 = dYawMatrix(-90.0f * 3.1416f/180.0f) * matrix;
			localL0 = matrix1.UntransformVector(l0); 
			localL1 = matrix1.UntransformVector(l1); 
			t1 = dPolygonRayCast (localL0, localL1, 4, &p[0].m_x, sizeof (dVector), indices);
			if (t1 < t) {
				t = t1;
				handle = m_yzHandle;
			}


			dFloat radius = NE_TRANSLATE_ARROW_RADIUS_FACTOR * size;
			dFloat height = NE_TRANSLATE_ARROW_HEIGHT_FACTOR * size;
			dVector points[128 * 3];  
			dVector normals[128];  
			int count = CreateCone (points, normals, 8, radius, height, 128);
			matrix1 = GetIdentityMatrix();
			matrix1.m_posit.m_x = size;
			matrix1 = matrix1 * matrix;
			localL0 = matrix1.UntransformVector(l0); 
			localL1 = matrix1.UntransformVector(l1); 
			for (int i = 0; i < count; i ++) {
				t1 = dPolygonRayCast (localL0, localL1, 3, &points[i * 3].m_x, sizeof (dVector), indices);
				if (t1 < t) {
					t = t1;
					handle = m_xHandle;
					break;
				}
			}

			matrix1 = dRollMatrix(90.0f * 3.141592f / 180.0f);
			matrix1.m_posit.m_y = size;
			matrix1 = matrix1 * matrix;
			localL0 = matrix1.UntransformVector(l0); 
			localL1 = matrix1.UntransformVector(l1); 
			for (int i = 0; i < count; i ++) {
				t1 = dPolygonRayCast (localL0, localL1, 3, &points[i * 3].m_x, sizeof (dVector), indices);
				if (t1 < t) {
					t = t1;
					handle = m_yHandle;
					break;
				}
			}

			matrix1 = dYawMatrix(-90.0f * 3.141592f / 180.0f);
			matrix1.m_posit.m_z = size;
			matrix1 = matrix1 * matrix;
			localL0 = matrix1.UntransformVector(l0); 
			localL1 = matrix1.UntransformVector(l1); 
			for (int i = 0; i < count; i ++) {
				t1 = dPolygonRayCast (localL0, localL1, 3, &points[i * 3].m_x, sizeof (dVector), indices);
				if (t1 < t) {
					t = t1;
					handle = m_zHandle;
					break;
				}
			}

			break;
		}
	}

	return handle;
*/
}


void dCollisionNodeInfo::DrawGizmoHandle(dScene* world, const dMatrix& coordinaSystem, dGizmoMode mode, dGizmoHandle handle, const dVector& color, dFloat size) const
{
_ASSERTE (0);

/*
	glEnable (GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glDisable(GL_CULL_FACE);
	glDisable (GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_BLEND);
	glDisable (GL_DEPTH_TEST);

	dVector p[4];
	dVector q[4];

	dVector euler (GetEulers ());
	dMatrix matrix (euler.m_x, euler.m_y, euler.m_z, GetPosition());
	matrix = matrix * coordinaSystem;
	switch (mode)
	{
		case dNodeInfo::m_translation:
		{
			dFloat planeSize = size * NE_TRANSLATE_LINEAR_FACTOR;
			p[0] = dVector (planeSize * 0.0f, planeSize * 0.0f, 0.0f, 0.0f);
			p[1] = dVector (planeSize * 1.0f, planeSize * 0.0f, 0.0f, 0.0f);
			p[2] = dVector (planeSize * 1.0f, planeSize * 1.0f, 0.0f, 0.0f);
			p[3] = dVector (planeSize * 0.0f, planeSize * 1.0f, 0.0f, 0.0f);

			switch (handle) 
			{
				case m_xHandle:
				{
					dFloat radius = NE_TRANSLATE_ARROW_RADIUS_FACTOR * size;
					dFloat height = NE_TRANSLATE_ARROW_HEIGHT_FACTOR * size;

					dMatrix matrix1 = GetIdentityMatrix();
					matrix1.m_posit.m_x = size;
					matrix1 = matrix1 * matrix;

					Draw3DArrow (matrix1, 16, radius, height, color);
					break;
				}

				case m_yHandle:
				{
					dFloat radius = NE_TRANSLATE_ARROW_RADIUS_FACTOR * size;
					dFloat height = NE_TRANSLATE_ARROW_HEIGHT_FACTOR * size;

					dMatrix matrix1 = dRollMatrix(90.0f * 3.141592f / 180.0f);
					matrix1.m_posit.m_y = size;
					matrix1 = matrix1 * matrix;
					Draw3DArrow (matrix1, 16, radius, height, color);
					break;
				}

				case m_zHandle:
				{
					dFloat radius = NE_TRANSLATE_ARROW_RADIUS_FACTOR * size;
					dFloat height = NE_TRANSLATE_ARROW_HEIGHT_FACTOR * size;

					dMatrix matrix1 = dYawMatrix(-90.0f * 3.141592f / 180.0f);
					matrix1.m_posit.m_z = size;
					matrix1 = matrix1 * matrix;
					Draw3DArrow (matrix1, 16, radius, height, color);
					break;
				}


				case m_xyHandle:
				{
					matrix.TransformTriplex(q, sizeof (dVector), p, sizeof (dVector), 4);

					glBegin(GL_TRIANGLES);
					for (int i = 2; i < 4; i ++) {
						glColor3f (color[0], color[1], color[2]);
						glVertex3f(q[0].m_x, q[0].m_y, q[0].m_z); // origin of the line
						glVertex3f(q[i].m_x, q[i].m_y, q[i].m_z); // origin of the line
						glVertex3f(q[i-1].m_x, q[i-1].m_y, q[i-1].m_z); // origin of the line
					}
					glEnd();
					break;
				}

				case m_zxHandle:
				{
					dMatrix matrix1 (dPitchMatrix(90.0f * 3.1416f/180.0f) * matrix);
					matrix1.TransformTriplex(q, sizeof (dVector), p, sizeof (dVector), 4);
					
					glBegin(GL_TRIANGLES);
					for (int i = 2; i < 4; i ++) {
						glColor3f (color[0], color[1], color[2]);
						glVertex3f(q[0].m_x, q[0].m_y, q[0].m_z); // origin of the line
						glVertex3f(q[i].m_x, q[i].m_y, q[i].m_z); // origin of the line
						glVertex3f(q[i-1].m_x, q[i-1].m_y, q[i-1].m_z); // origin of the line
					}
					glEnd();
					break;
				}

				case m_yzHandle:
				{
					dMatrix matrix1 (dYawMatrix(-90.0f * 3.1416f/180.0f) * matrix);
					matrix1.TransformTriplex(q, sizeof (dVector), p, sizeof (dVector), 4);

					glBegin(GL_TRIANGLES);
					for (int i = 2; i < 4; i ++) {
						glColor3f (color[0], color[1], color[2]);
						glVertex3f(q[0].m_x, q[0].m_y, q[0].m_z); // origin of the line
						glVertex3f(q[i].m_x, q[i].m_y, q[i].m_z); // origin of the line
						glVertex3f(q[i-1].m_x, q[i-1].m_y, q[i-1].m_z); // origin of the line
					}
					glEnd();
					break;
				}
			}
			break;
		}


		case dNodeInfo::m_rotation:
		{
			switch (handle) 
			{
				case m_xHandle:
				{
					dFloat radius = NE_ROTATE_RADIUS_FACTOR * size;
					dFloat height = NE_ROTATE_HEIGHT_FACTOR * size;

					dMatrix matrix1 = GetIdentityMatrix();
					matrix1.m_posit.m_x = size * NE_ROTATE_GIZMO_LOCATION_FACTOR;
					matrix1 = matrix1 * matrix;
					Draw3DCylinder(matrix1, 16, radius, height, color);
					break;
				}

				case m_yHandle:
				{
					dFloat radius = NE_ROTATE_RADIUS_FACTOR * size;
					dFloat height = NE_ROTATE_HEIGHT_FACTOR * size;
					dMatrix matrix1 (dRollMatrix(90.0f * 3.141592f / 180.0f));
					matrix1.m_posit.m_y = size * NE_ROTATE_GIZMO_LOCATION_FACTOR;
					matrix1 = matrix1 * matrix;
					Draw3DCylinder(matrix1, 16, radius, height, color);
					break;
				}

				case m_zHandle:
				{
					dFloat radius = NE_ROTATE_RADIUS_FACTOR * size;
					dFloat height = NE_ROTATE_HEIGHT_FACTOR * size;

					dMatrix matrix1 (dYawMatrix(-90.0f * 3.141592f / 180.0f));
					matrix1.m_posit.m_z = size * NE_ROTATE_GIZMO_LOCATION_FACTOR;
					matrix1 = matrix1 * matrix;
					Draw3DCylinder(matrix1, 16, radius, height, color);
					break;
				}
			}

			break;
		}

		case dNodeInfo::m_scale:
		{
			dFloat planeSize = size * NE_TRANSLATE_LINEAR_FACTOR;
			p[0] = dVector (planeSize * 0.0f, planeSize * 0.0f, 0.0f, 0.0f);
			p[1] = dVector (planeSize * 1.0f, planeSize * 0.0f, 0.0f, 0.0f);
			p[2] = dVector (planeSize * 1.0f, planeSize * 1.0f, 0.0f, 0.0f);
			p[3] = dVector (planeSize * 0.0f, planeSize * 1.0f, 0.0f, 0.0f);

			switch (handle) 
			{
				case m_xyzHandle:
				{
					glPushMatrix();
					glMultMatrix(&matrix[0][0]);
					glBegin(GL_TRIANGLES);
					for (int i = 0; i < 3; i ++) {
						p[0] = dVector (0.0f, 0.0f, 0.0f, 0.0f);
						dVector tmp (0.0f, 0.0f, 0.0f, 0.0);
						p[2] = tmp;
						tmp[i] = size * NE_SCALE_UNIFORM_SCALE_FACTOR;
						p[1] = tmp;
						q[i] = tmp;
						p[2][(i + 1) % 3] = size * NE_SCALE_UNIFORM_SCALE_FACTOR;

						glColor3f (color[0], color[1], color[2]);
						glVertex3f(p[0].m_x, p[0].m_y, p[0].m_z); // origin of the line
						glVertex3f(p[1].m_x, p[1].m_y, p[1].m_z); // origin of the line
						glVertex3f(p[2].m_x, p[2].m_y, p[2].m_z); // origin of the line
					}

					glColor3f (color[0], color[1], color[2]);
					glVertex3f(q[0].m_x, q[0].m_y, q[0].m_z); // origin of the line
					glVertex3f(q[1].m_x, q[1].m_y, q[1].m_z); // origin of the line
					glVertex3f(q[2].m_x, q[2].m_y, q[2].m_z); // origin of the line

					glEnd();
					glPopMatrix();
					break;
				}


				case m_xHandle:
				{
					dFloat radius = NE_TRANSLATE_ARROW_RADIUS_FACTOR * size;
					dFloat height = NE_TRANSLATE_ARROW_HEIGHT_FACTOR * size;

					dMatrix matrix1 = GetIdentityMatrix();
					matrix1.m_posit.m_x = size;
					matrix1 = matrix1 * matrix;

					Draw3DArrow (matrix1, 16, radius, height, color);
					break;
				}

				case m_yHandle:
				{
					dFloat radius = NE_TRANSLATE_ARROW_RADIUS_FACTOR * size;
					dFloat height = NE_TRANSLATE_ARROW_HEIGHT_FACTOR * size;

					dMatrix matrix1 = dRollMatrix(90.0f * 3.141592f / 180.0f);
					matrix1.m_posit.m_y = size;
					matrix1 = matrix1 * matrix;
					Draw3DArrow (matrix1, 16, radius, height, color);
					break;
				}

				case m_zHandle:
				{
					dFloat radius = NE_TRANSLATE_ARROW_RADIUS_FACTOR * size;
					dFloat height = NE_TRANSLATE_ARROW_HEIGHT_FACTOR * size;

					dMatrix matrix1 = dYawMatrix(-90.0f * 3.141592f / 180.0f);
					matrix1.m_posit.m_z = size;
					matrix1 = matrix1 * matrix;
					Draw3DArrow (matrix1, 16, radius, height, color);
					break;
				}

				case m_xyHandle:
				{
					matrix.TransformTriplex(q, sizeof (dVector), p, sizeof (dVector), 4);

					glBegin(GL_TRIANGLES);
					for (int i = 2; i < 4; i ++) {
						glColor3f (color[0], color[1], color[2]);
						glVertex3f(q[0].m_x, q[0].m_y, q[0].m_z); // origin of the line
						glVertex3f(q[i].m_x, q[i].m_y, q[i].m_z); // origin of the line
						glVertex3f(q[i-1].m_x, q[i-1].m_y, q[i-1].m_z); // origin of the line
					}
					glEnd();
					break;
				}

				case m_zxHandle:
				{
					dMatrix matrix1 (dPitchMatrix(90.0f * 3.1416f/180.0f) * matrix);
					matrix1.TransformTriplex(q, sizeof (dVector), p, sizeof (dVector), 4);

					glBegin(GL_TRIANGLES);
					for (int i = 2; i < 4; i ++) {
						glColor3f (color[0], color[1], color[2]);
						glVertex3f(q[0].m_x, q[0].m_y, q[0].m_z); // origin of the line
						glVertex3f(q[i].m_x, q[i].m_y, q[i].m_z); // origin of the line
						glVertex3f(q[i-1].m_x, q[i-1].m_y, q[i-1].m_z); // origin of the line
					}
					glEnd();
					break;
				}

				case m_yzHandle:
				{
					dMatrix matrix1 (dYawMatrix(-90.0f * 3.1416f/180.0f) * matrix);
					matrix1.TransformTriplex(q, sizeof (dVector), p, sizeof (dVector), 4);

					glBegin(GL_TRIANGLES);
					for (int i = 2; i < 4; i ++) {
						glColor3f (color[0], color[1], color[2]);
						glVertex3f(q[0].m_x, q[0].m_y, q[0].m_z); // origin of the line
						glVertex3f(q[i].m_x, q[i].m_y, q[i].m_z); // origin of the line
						glVertex3f(q[i-1].m_x, q[i-1].m_y, q[i-1].m_z); // origin of the line
					}
					glEnd();
					break;
				}
			}
			break;
		}
	}
*/
}


// Draw selection gizmo
void dCollisionNodeInfo::DrawGizmo(
	dScene* world, 
	dScene::dTreeNode* myNode, 
	const dMatrix& coordinaSystem,
	const dVector& color, 
	dGizmoMode mode, 
	dFloat size) const
{
_ASSERTE (0);

/*
	_ASSERTE (world->GetInfoFromNode(myNode) == this);
	glEnable (GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glDisable(GL_CULL_FACE);
	glDisable (GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_BLEND);

	DrawWireFrame(world, myNode, dVector(1.0f, 1.0f, 1.0f, 0.0f), workBuffer, worlfBufferInBytes);
//	DrawGizmoHandle(world, coordinaSystem, mode, handle, color, size);

	dVector euler (GetEulers ());
	dMatrix matrix (euler.m_x, euler.m_y, euler.m_z, GetPosition());
	matrix = matrix * coordinaSystem;

	glPushMatrix();
	glLoadMatrix(&matrix[0][0]);

	glDisable (GL_DEPTH_TEST);
	switch (mode) 
	{
		case dNodeInfo::m_selection:
		{
			glBegin(GL_LINES);
			for (int i = 0; i < 3; i ++) {
				dVector p1 (0.0f, 0.0f, 0.0f, 1.0f);
				p1[i] = size;
				dVector color (0.0f, 0.0f, 0.0f, 0.0f) ;
				color[i] = 1.0f;
				glColor3f (color[0], color[1], color[2]);
				glVertex3f(0.0f, 0.0f, 0.0f); // origin of the line
				glVertex3f(p1.m_x, p1.m_y, p1.m_z); // origin of the line
			}
			glEnd();
			break;
		}

		case dNodeInfo::m_translation:
		{
			dFloat planeSize = size * NE_TRANSLATE_LINEAR_FACTOR;
			glBegin(GL_LINES);
			for (int i = 0; i < 3; i ++) {
				dVector p1 (0.0f, 0.0f, 0.0f, 1.0f);
				dVector color (0.0f, 0.0f, 0.0f, 0.0f) ;
				p1[i] = size;
				color[i] = 1.0f;
				glColor3f (color[0], color[1], color[2]);
				glVertex3f(0.0f, 0.0f, 0.0f); // origin of the line
				glVertex3f(p1.m_x, p1.m_y, p1.m_z); // origin of the line

				// draw 2d selection plane
				color = dVector (0.0f, 0.0f, 0.0f, 0.0f);
				dVector q0 (0.0f, 0.0f, 0.0f);
				dVector q1 (0.0f, 0.0f, 0.0f);
				color[i] = 1.0f;
				int index = (i + 1) % 3;	
				q0[index] = planeSize;
				q1[index] = planeSize;
				q1[i] = planeSize;
				glColor3f (color[0], color[1], color[2]);
				glVertex3f(q0.m_x, q0.m_y, q0.m_z); 
				glVertex3f(q1.m_x, q1.m_y, q1.m_z); 


				color = dVector (0.0f, 0.0f, 0.0f, 0.0f);
				q0 = dVector (0.0f, 0.0f, 0.0f);
				q1 = dVector (0.0f, 0.0f, 0.0f);
				color[index] = 1.0f;
				q0[i] = planeSize;
				q1[i] = planeSize;
				q1[index] = planeSize;
				glColor3f (color[0], color[1], color[2]);
				glVertex3f(q0.m_x, q0.m_y, q0.m_z); 
				glVertex3f(q1.m_x, q1.m_y, q1.m_z); 
			}
			glEnd();

			dFloat radius = NE_TRANSLATE_ARROW_RADIUS_FACTOR * size;
			dFloat height = NE_TRANSLATE_ARROW_HEIGHT_FACTOR * size;
			int segments = 16;

			// draw dragging handles
			dMatrix location (GetIdentityMatrix());
			location.m_posit.m_x = size;
			Draw3DArrow (location, segments, radius, height, dVector (1.0f, 0.0f, 0.0f, 0.0f));

			location = dRollMatrix(90.0f * 3.141592f / 180.0f);
			location.m_posit.m_y = size;
			Draw3DArrow (location, segments, radius, height, dVector (0.0f, 1.0f, 0.0f, 0.0f));

			location = dYawMatrix(-90.0f * 3.141592f / 180.0f);
			location.m_posit.m_z = size;
			Draw3DArrow (location, segments, radius, height, dVector (0.0f, 0.0f, 1.0f, 0.0f));

			break;
		}

		case dNodeInfo::m_rotation:
		{
			glBegin(GL_LINES);
			for (int i = 0; i < 3; i ++) {
				dVector p1 (0.0f, 0.0f, 0.0f, 1.0f);
				p1[i] = size;
				dVector color (0.0f, 0.0f, 0.0f, 0.0f) ;
				color[i] = 1.0f;
				glColor3f (color[0], color[1], color[2]);
				glVertex3f(0.0f, 0.0f, 0.0f); // origin of the line
				glVertex3f(p1.m_x, p1.m_y, p1.m_z); // origin of the line
			}
			glEnd();

			dFloat radius = NE_ROTATE_RADIUS_FACTOR * size;
			dFloat height = NE_ROTATE_HEIGHT_FACTOR * size;
			int segments = 16;

			dMatrix location (GetIdentityMatrix());
			location.m_posit.m_x = size * NE_ROTATE_GIZMO_LOCATION_FACTOR;
			Draw3DCylinder (location, segments, radius, height, dVector (1.0f, 0.0f, 0.0f, 0.0f));

			location = dRollMatrix(90.0f * 3.141592f / 180.0f);
			location.m_posit.m_y = size * NE_ROTATE_GIZMO_LOCATION_FACTOR;
			Draw3DCylinder (location, segments, radius, height, dVector (0.0f, 1.0f, 0.0f, 0.0f));

			location = dYawMatrix(-90.0f * 3.141592f / 180.0f);
			location.m_posit.m_z = size * NE_ROTATE_GIZMO_LOCATION_FACTOR;
			Draw3DCylinder (location, segments, radius, height, dVector (0.0f, 0.0f, 1.0f, 0.0f));
			break;
		}

		case dNodeInfo::m_scale:
		{
			glBegin(GL_LINES);
			for (int i = 0; i < 3; i ++) {
				dVector p1 (0.0f, 0.0f, 0.0f, 1.0f);
				p1[i] = size;
				dVector color (0.0f, 0.0f, 0.0f, 0.0f) ;
				color[i] = 1.0f;
				glColor3f (color[0], color[1], color[2]);
				glVertex3f(0.0f, 0.0f, 0.0f); // origin of the line
				glVertex3f(p1.m_x, p1.m_y, p1.m_z); // origin of the line

				// draw the uniform scale diamond
				dFloat planeSize = size * NE_SCALE_UNIFORM_SCALE_FACTOR;
				int index = (i + 1) % 3;
				color = dVector (0.0f, 0.0f, 0.0f, 0.0f);
				dVector q0 (0.0f, 0.0f, 0.0f);
				dVector q1 (0.0f, 0.0f, 0.0f);
				color[i] = 1.0f;
				color[index] = 1.0f;
				q0[index] = planeSize;
				q1[i] = planeSize;
				glColor3f (1.0, 1.0, 0.0f);
				glVertex3f(q0.m_x, q0.m_y, q0.m_z); 
				glVertex3f(q1.m_x, q1.m_y, q1.m_z); 

				// draw 2d selection plane
				planeSize = size * NE_TRANSLATE_LINEAR_FACTOR;
				color = dVector (0.0f, 0.0f, 0.0f, 0.0f);
				q0 = dVector (0.0f, 0.0f, 0.0f);
				q1 = dVector (0.0f, 0.0f, 0.0f);
				color[i] = 1.0f;
				index = (i + 1) % 3;	
				q0[index] = planeSize;
				q1[index] = planeSize;
				q1[i] = planeSize;
				glColor3f (color[0], color[1], color[2]);
				glVertex3f(q0.m_x, q0.m_y, q0.m_z); 
				glVertex3f(q1.m_x, q1.m_y, q1.m_z); 

				color = dVector (0.0f, 0.0f, 0.0f, 0.0f);
				q0 = dVector (0.0f, 0.0f, 0.0f);
				q1 = dVector (0.0f, 0.0f, 0.0f);
				color[index] = 1.0f;
				q0[i] = planeSize;
				q1[i] = planeSize;
				q1[index] = planeSize;
				glColor3f (color[0], color[1], color[2]);
				glVertex3f(q0.m_x, q0.m_y, q0.m_z); 
				glVertex3f(q1.m_x, q1.m_y, q1.m_z); 
			}
			glEnd();

			dFloat radius = NE_TRANSLATE_ARROW_RADIUS_FACTOR * size;
			dFloat height = NE_TRANSLATE_ARROW_HEIGHT_FACTOR * size;
			int segments = 16;

			// draw one dimension scale handles
			dMatrix location (GetIdentityMatrix());
			location.m_posit.m_x = size;
			Draw3DArrow (location, segments, radius, height, dVector (1.0f, 0.0f, 0.0f, 0.0f));

			location = dRollMatrix(90.0f * 3.141592f / 180.0f);
			location.m_posit.m_y = size;
			Draw3DArrow (location, segments, radius, height, dVector (0.0f, 1.0f, 0.0f, 0.0f));

			location = dYawMatrix(-90.0f * 3.141592f / 180.0f);
			location.m_posit.m_z = size;
			Draw3DArrow (location, segments, radius, height, dVector (0.0f, 0.0f, 1.0f, 0.0f));
			break;
		}
	}
	glPopMatrix();
*/
}


NewtonCollision* dCollisionNodeInfo::CreateNewtonCollision (NewtonWorld* const world, dScene* const scene, dScene::dTreeNode* const myNode) const
{
	_ASSERTE (IsType (dCollisionNodeInfo::GetRttiType()));

	_ASSERTE (0);
	return NULL;
/*
	// get the collision node	
	dScene::dTreeNode* shapeNode = scene->FindChildByType( bodyNode, dCollisionNodeInfo::GetRttiType());
	_ASSERTE (shapeNode);
	dCollisionNodeInfo* collInfo = (dCollisionNodeInfo*) scene->GetInfoFromNode(shapeNode);

	dMatrix offsetMatrix (collInfo->GetTransform ());
	int collisionID = collInfo->GetShapeId ();

	// crate a newton collision shape from the node.
	NewtonCollision* collision = NULL;
	if (collInfo->GetTypeId() == dCollisionSphereNodeInfo::GetRttiType()) {
		_ASSERTE (0);
	} else if (collInfo->GetTypeId() == dCollisionBoxNodeInfo::GetRttiType()) { 
		dCollisionBoxNodeInfo* box = (dCollisionBoxNodeInfo*) collInfo;
		dVector size (box->GetSize());
		collision = NewtonCreateBox(m_world, size.m_x, size.m_y, size.m_z, collisionID, &offsetMatrix[0][0]);
	} else if (collInfo->GetTypeId() == dCollisionConvexHullNodeInfo::GetRttiType()) { 
		dCollisionConvexHullNodeInfo* convex = (dCollisionConvexHullNodeInfo*) collInfo;
		collision = NewtonCreateConvexHull(m_world, convex->GetVertexCount(), &convex->GetVertexCloud()[0].m_x, sizeof(dVector), 0.01f, collisionID, &offsetMatrix[0][0]);
	} else {
		_ASSERTE (0);
	}
	return collision;
*/
}

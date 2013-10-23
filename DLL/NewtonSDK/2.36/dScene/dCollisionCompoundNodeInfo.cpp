/////////////////////////////////////////////////////////////////////////////
// Name:        dCollisionCompoundNodeInfo.h
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
#include "dCollisionCompoundNodeInfo.h"


D_IMPLEMENT_CLASS_NODE(dCollisionCompoundNodeInfo);

dCollisionCompoundNodeInfo::dCollisionCompoundNodeInfo(dScene* const world) 
	:dCollisionNodeInfo ()
{
	SetName ("compound collision");
}

dCollisionCompoundNodeInfo::dCollisionCompoundNodeInfo()
	:dCollisionNodeInfo ()
{
	SetName ("compound collision");
}

dCollisionCompoundNodeInfo::~dCollisionCompoundNodeInfo(void)
{
}

dCollisionCompoundNodeInfo::dCollisionCompoundNodeInfo(NewtonCollision* compound)
	:dCollisionNodeInfo () 
{
	_ASSERTE (0);
/*
	NewtonCollisionInfoRecord record;
	NewtonCollisionGetInfo(box, &record);
	_ASSERTE (record.m_collisionType == SERIALIZE_ID_BOX);

	dMatrix& offsetMatrix = *((dMatrix*) record.m_offsetMatrix);
	SetName ("compound collision");
	m_size = dVector (record.m_box.m_x, record.m_box.m_z, record.m_box.m_y); 
	SetTransform (offsetMatrix);
	SetShapeId (record.m_collisionUserID);

	CalculateGeometryProperies (box, m_geometricInertia, m_geometricCenterAndVolume); 
*/
}


void dCollisionCompoundNodeInfo::BakeTransform (const dMatrix& transform)
{
	dCollisionNodeInfo::BakeTransform (transform);
	_ASSERTE (0);
//	m_size = transform.RotateVector(m_size);
}


void dCollisionCompoundNodeInfo::CalculateInertiaGeometry (dScene* world, dVector& inertia, dVector& centerOfMass) const
{
	_ASSERTE (0);
/*
	NewtonWorld* newton = world->GetNewtonWorld();
	NewtonCollision* box = NewtonCreateBox(newton, m_size.m_x, m_size.m_y, m_size.m_z, 0, &m_matrix[0][0]);
	CalculateGeometryProperies (box, inertia, centerOfMass);
	NewtonReleaseCollision (newton, box);
*/
}


void dCollisionCompoundNodeInfo::Serialize (TiXmlElement* const rootNode) const
{
	SerialiseBase(dCollisionNodeInfo, rootNode);

//	char tmp[1024];
//	TiXmlElement* dataNode = new TiXmlElement ("size");
//	rootNode->LinkEndChild(dataNode);
//	dFloatArrayToString (&m_size[0], 4, tmp, sizeof (tmp));
//	dataNode->SetAttribute("float4", tmp);
}

bool dCollisionCompoundNodeInfo::Deserialize (TiXmlElement* const rootNode, int revisionNumber) 
{
	DeserialiseBase(dCollisionNodeInfo, rootNode, revisionNumber);

//	TiXmlElement* dataNode = (TiXmlElement*) rootNode->FirstChild ("size");
//	dStringToFloatArray (dataNode->Attribute("float4"), &m_size[0], 4);
	return true;
}


void dCollisionCompoundNodeInfo::SerializeBinary (FILE* const file) 
{
_ASSERTE (0);
	fprintf (file, "%s\n%s\n", GetClassName(), GetName());
	_ASSERTE (0);
//	fwrite (&m_matrix[0][0], 1, sizeof (dMatrix), file);
}


void dCollisionCompoundNodeInfo::DrawWireFrame(dScene* const world, dScene::dTreeNode* const myNode, const dVector& color) const
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
void dCollisionCompoundNodeInfo::DrawSolidWireFrame(dScene* const world, dScene::dTreeNode* const myNode, const dVector& color) const
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
void dCollisionCompoundNodeInfo::DrawGouraudShaded(dScene* const world, dScene::dTreeNode* const myNode, const dVector& color) const
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




dCollisionCompoundNodeInfo::dGizmoHandle dCollisionCompoundNodeInfo::GetHighlightedGizmoHandle(
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
		case dCollisionNodeInfo::m_translation:
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

		case dCollisionNodeInfo::m_rotation:
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


		case dCollisionNodeInfo::m_scale:
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


void dCollisionCompoundNodeInfo::DrawGizmoHandle(dScene* world, const dMatrix& coordinaSystem, dGizmoMode mode, dGizmoHandle handle, const dVector& color, dFloat size) const
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
		case dCollisionNodeInfo::m_translation:
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


		case dCollisionNodeInfo::m_rotation:
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

		case dCollisionNodeInfo::m_scale:
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
void dCollisionCompoundNodeInfo::DrawGizmo(
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
		case dCollisionNodeInfo::m_selection:
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

		case dCollisionNodeInfo::m_translation:
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

		case dCollisionNodeInfo::m_rotation:
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

		case dCollisionNodeInfo::m_scale:
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



NewtonCollision* dCollisionCompoundNodeInfo::CreateNewtonCollision (NewtonWorld* const world, dScene* const scene, dScene::dTreeNode* const myNode) const
{
	_ASSERTE (IsType (dCollisionCompoundNodeInfo::GetRttiType()));

	// get the collision node	
	int collisionID = GetShapeId ();
//	const dMatrix& offsetMatrix = GetTransform ();
	
	// count all collision nodes
	int count = 0;
	for (void* link = scene->GetFirstChild(myNode); link; link = scene->GetNextChild (myNode, link)) {
		dScene::dTreeNode* childNode = scene->GetNodeFromLink(link);
		dNodeInfo* info = scene->GetInfoFromNode(childNode);
		count += (info->IsType (dCollisionNodeInfo::GetRttiType()));
	}

	// create space to load all sub shapes
	NewtonCollision** collisionArray = new NewtonCollision*[count];
	int index = 0;
	for (void* link = scene->GetFirstChild(myNode); link; link = scene->GetNextChild (myNode, link)) {
		dScene::dTreeNode* childNode = scene->GetNodeFromLink(link);
		dNodeInfo* info = scene->GetInfoFromNode(childNode);
		if (info->IsType (dCollisionNodeInfo::GetRttiType())) {
			dCollisionNodeInfo* collInfo = (dCollisionNodeInfo*) scene->GetInfoFromNode(childNode);
			collisionArray[index] = collInfo->CreateNewtonCollision (world, scene, childNode);
			index ++;
		}
	}
	
	// create a compound collision 
	NewtonCollision* collision = NewtonCreateCompoundCollision (world, count, collisionArray, collisionID);

	// release all temporary memory
	for (int i = 0; i < count; i ++) {
		NewtonReleaseCollision(world, collisionArray [i]);
	}
	delete[] collisionArray;
	
	return collision;
}
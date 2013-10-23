#include <cstddef>
#include <stdafx.h>
#include "newton.h"
#include "dll.h"
#include "recast.h"


export double GmnMaterialGetDefaultGroupID(double dWorld);
export double GmnMaterialCreateGroupId(double dWorld);
export double GmnMaterialDestroyAllGroupID(double dWorld);


export double GmnMaterialSetDefaultCollidable(double dWorld, double id0, double id1, double state);
export double GmnMaterialSetContinuousCollisionMode(double dWorld, double id0, double id1, double state);
export double GmnMaterialSetDefaultFriction(double dWorld, double id0, double id1, double coesta, double coedyn);
export double GmnMaterialSetDefaultElasticity(double dWorld, double id0, double id1, double elasticCoef);
export double GmnMaterialSetDefaultSoftness(double dWorld, double id0, double id1, double softnessCoef);
export double GmnMaterialSetCollisionCallback(double dWorld, double mat0, double mat1);


int MaterialBeginCollisionCallback (const NewtonMaterial* material, const NewtonBody* body0, const NewtonBody* body1);
int MaterialContactCollisionCallback (const NewtonMaterial* material, const NewtonContact* contact);
void MaterialEndCollisionCallback (const NewtonMaterial* material);

int ClearCollisionRecord();

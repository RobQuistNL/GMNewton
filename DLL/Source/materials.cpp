#include "materials.h"


/*********************************
GroupID setup interface
*********************************/

export double GmnMaterialGetDefaultGroupID(double dWorld){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   return (double)NewtonMaterialGetDefaultGroupID(nWorld);
}

export double GmnMaterialCreateGroupId(double dWorld){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   return (double)NewtonMaterialCreateGroupID(nWorld);
}

export double GmnMaterialDestroyAllGroupID(double dWorld){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   NewtonMaterialDestroyAllGroupID(nWorld);
   return (1);
}


/*********************************
Material setup interface
*********************************/

export double GmnMaterialSetDefaultCollidable(double dWorld, double id0, double id1, double state){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   NewtonMaterialSetDefaultCollidable(nWorld, (int)id0, (int)id1, (int)state);
   return (1);
}

export double GmnMaterialSetContinuousCollisionMode(double dWorld, double id0, double id1, double state){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   NewtonMaterialSetContinuousCollisionMode(nWorld, (int)id0, (int)id1, (int)state);
   return (1);
}

export double GmnMaterialSetDefaultFriction(double dWorld, double id0, double id1, double coesta, double coedyn){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   NewtonMaterialSetDefaultFriction(nWorld, (int)id0, (int)id1, coesta, coedyn);
   return (1);
}

export double GmnMaterialSetDefaultElasticity(double dWorld, double id0, double id1, double elasticCoef){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   NewtonMaterialSetDefaultElasticity(nWorld, (int)id0, (int)id1, elasticCoef);
   return (1);
}

export double GmnMaterialSetDefaultSoftness(double dWorld, double id0, double id1, double softnessCoef){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   NewtonMaterialSetDefaultSoftness(nWorld, (int)id0, (int)id1, softnessCoef);
   return (1);
}

export double GmnMaterialSetCollisionCallback(double dWorld, double mat0, double mat1){//,new CollisionEvent() , MaterialBeginCollisionCallback, MaterialContactCollisionCallback, MaterialEndCollisionCallback){
    NewtonWorld* nWorld = recastDoubleWorld(dWorld);
    NewtonMaterialSetCollisionCallback(nWorld, (int)mat0, (int)mat1, NULL, MaterialBeginCollisionCallback, MaterialContactCollisionCallback, MaterialEndCollisionCallback);
   return(1);
}

/***********************************
Collision Callbacks
***********************************/

struct COLLISION{
    COLLISION();
    const NewtonBody *colBody0, *colBody1;
    int maximumImpactSpeed;
    int responseType;
    int contactCount;
    COLLISION* prev;
} g_col;
COLLISION::COLLISION(){
    prev=NULL;
    maximumImpactSpeed=0;
    contactCount=0;
}
COLLISION* last_col = &g_col;

export double GmnMaterialSetResponseType(double dWorld, double mat0, double mat1, double action){
    //Sets response type for collisions between two material types
    //0  Normal
    //1  Ghost
    //2  Ignore
    //3  Physics Only
    int usrDat = (int)action;
    NewtonWorld* nWorld = recastDoubleWorld(dWorld);
    NewtonMaterialSetCollisionCallback(nWorld, (int)mat0, (int)mat1, &usrDat, MaterialBeginCollisionCallback, MaterialContactCollisionCallback, MaterialEndCollisionCallback);
    return(1);
}

int MaterialBeginCollisionCallback (const NewtonMaterial* material, const NewtonBody* body0, const NewtonBody* body1){
   //Do collision stuff here

   COLLISION *temp;

   temp = new COLLISION();
   temp->prev=last_col;
   last_col=temp;
   temp->colBody0 = body0;
   temp->colBody1 = body1;

   int action = (int)(NewtonMaterialGetMaterialPairUserData(material));
   temp->responseType = action;
   switch(action){
       case 0://Normal
          //Process as normal
          break;
       case 1://Ghost

          break;
       case 2://Ignore
          delete temp;
          return(0);
          break;
       case 3://Physics only

          break;
   }


   return(1);
}

int MaterialContactCollisionCallback (const NewtonMaterial* material, const NewtonContact* contact){
   //Do collision stuff here
   last_col->contactCount+=1;//Count the contact
   int contact_speed = (int)NewtonMaterialGetContactNormalSpeed(material, contact);
   if(contact_speed>last_col->maximumImpactSpeed){//Check if it is hardest contact
      last_col->maximumImpactSpeed=contact_speed;//If so, save impact speed
   }
   return(1);
}

void MaterialEndCollisionCallback (const NewtonMaterial* material){
   //Do collision stuff here

   if(last_col->contactCount<=0){//Forget collision if no contacts
       COLLISION* temp = last_col->prev;
       delete last_col;
       last_col=temp;
       delete temp;
   }



   //GCurrentEvent = (CollisionEvent*) NewtonMaterialGetMaterialPairUserData (material);
   //if(GCurrentEvent->contacts>0){
   //   collisions[0].push_back(*(int*)NewtonBodyGetUserData(GCurrentEvent->body[0]));
   //   collisions[1].push_back(*(int*)NewtonBodyGetUserData(GCurrentEvent->body[1]));
   //   collision_contacts.push_back(GCurrentEvent->contacts);
   //   collision_contact_points.push_back(GCurrentEvent->contact_points);
   //}
}

int ClearCollisionRecord(){
    COLLISION *temp;
    while(last_col->prev!=NULL){
       temp=last_col;
       last_col=temp->prev;
       delete temp;
    }
   return(1);
}


//May 8 09 1:31
export double GmnCollisionGet(){
    if(last_col->prev!=NULL){
        return(double)(int)last_col;
    }else{
        return(0);
    }
}

export double GmnCollisionGetNext(double current){
    COLLISION *temp;
    temp = (COLLISION*)(int)current;
    if(temp->prev!=NULL){
        return(double)(int)temp;
    }else{
        return(0);
    }
}

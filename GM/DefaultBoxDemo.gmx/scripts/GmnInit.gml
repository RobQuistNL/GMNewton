//global.__GmnSetCompatabilityMode__ = external_define("GMNewton.dll","GmnSetCompatabilityMode",dll_cdecl,ty_real,1,ty_real);
//global.__GmnGetCompatabilityMode__ = external_define("GMNewton.dll","GmnGetCompatabilityMode",dll_cdecl,ty_real,0);
global.__GmnSetGravity__ = external_define("GMNewton.dll","GmnSetGravity",dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.__GmnCreateBody__ = external_define("GMNewton.dll","GmnCreateBody",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnDestroyBody__ = external_define("GMNewton.dll","GmnDestroyBody",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnBodyLinkObject__ = external_define("GMNewton.dll","GmnBodyLinkObject",dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.__GmnBodyUnlinkObject__ = external_define("GMNewton.dll","GmnBodyUnlinkObject",dll_cdecl,ty_real,1,ty_real);
global.__GmnBodyGetLinked__ = external_define("GMNewton.dll","GmnBodyGetLinked",dll_cdecl,ty_real,1,ty_real);
global.__GmnBodyGetLinkedObject__ = external_define("GMNewton.dll","GmnBodyGetLinkedObject",dll_cdecl,ty_real,1,ty_real);
global.__GmnBodyGetWorld__ = external_define("GMNewton.dll","GmnBodyGetWorld",dll_cdecl,ty_real,1,ty_real);
global.__GmnBodyGetCollision__ = external_define("GMNewton.dll","GmnBodyGetCollision",dll_cdecl,ty_real,1,ty_real);
global.__GmnBodySetMassMatrix__ = external_define("GMNewton.dll","GmnBodySetMassMatrix",dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnBodySetForce__ = external_define("GMNewton.dll","GmnBodySetForce",dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.__GmnBodyAddForce__ = external_define("GMNewton.dll","GmnBodyAddForce",dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.__GmnBodyGetForce__ = external_define("GMNewton.dll","GmnBodyGetForce",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnBodySetTorque__ = external_define("GMNewton.dll","GmnBodySetTorque",dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.__GmnBodyAddTorque__ = external_define("GMNewton.dll","GmnBodyAddTorque",dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.__GmnBodyGetTorque__ = external_define("GMNewton.dll","GmnBodyGetTorque",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnBodySetAutoFreeze__ = external_define("GMNewton.dll","GmnBodySetAutoFreeze",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnBodySetAutoSleep__ = external_define("GMNewton.dll","GmnBodySetAutoSleep",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnBodySetMaterialGroupID__ = external_define("GMNewton.dll","GmnBodySetMaterialGroupID",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnBodySetContinuousCollisionMode__ = external_define("GMNewton.dll","GmnBodySetContinuousCollisionMode",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnBodySetJointRecursiveCollision__ = external_define("GMNewton.dll","GmnBodySetJointRecursiveCollision",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnBodySetVelocity__ = external_define("GMNewton.dll","GmnBodySetVelocity",dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.__GmnBodySetVelocityAxis__ = external_define("GMNewton.dll","GmnBodySetVelocityAxis",dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.__GmnBodyGetVelocity__ = external_define("GMNewton.dll","GmnBodyGetVelocity",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnBodySetOmega__ = external_define("GMNewton.dll","GmnBodySetOmega",dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.__GmnBodySetOmegaAxis__ = external_define("GMNewton.dll","GmnBodySetOmegaAxis",dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.__GmnBodyGetOmega__ = external_define("GMNewton.dll","GmnBodyGetOmega",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnBodySetLinearDamping__ = external_define("GMNewton.dll","GmnBodySetLinearDamping",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnBodyGetLinearDamping__ = external_define("GMNewton.dll","GmnBodyGetLinearDamping",dll_cdecl,ty_real,1,ty_real);
global.__GmnBodySetAngularDamping__ = external_define("GMNewton.dll","GmnBodySetAngularDamping",dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.__GmnBodyGetAngularDamping__ = external_define("GMNewton.dll","GmnBodyGetAngularDamping",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnBodyAddImpulse__ = external_define("GMNewton.dll","GmnBodyAddImpulse",dll_cdecl,ty_real,7,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnBodySetCentreOfMass__ = external_define("GMNewton.dll","GmnBodySetCentreOfMass",dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.__GmnBodySetAutoMassMatrix__ = external_define("GMNewton.dll","GmnBodySetAutoMassMatrix",dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.__GmnBodyAutoGetPosRot__ = external_define("GMNewton.dll","GmnBodyAutoGetPosRot",dll_cdecl,ty_real,1,ty_real);
global.__GmnBodyGetPosition__ = external_define("GMNewton.dll","GmnBodyGetPosition",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnBodyGetRotation__ = external_define("GMNewton.dll","GmnBodyGetRotation",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnBodySetPosition__ = external_define("GMNewton.dll","GmnBodySetPosition",dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.__GmnBodySetRotation__ = external_define("GMNewton.dll","GmnBodySetRotation",dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.__GmnBodyGetPointGlobalVelocity__ = external_define("GMNewton.dll","GmnBodyGetPointGlobalVelocity",dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnBodyGetPointLocalVelocity__ = external_define("GMNewton.dll","GmnBodyGetPointLocalVelocity",dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnBodyAddPointGlobalForce__ = external_define("GMNewton.dll","GmnBodyAddPointGlobalForce",dll_cdecl,ty_real,7,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnBodyAddPointLocalForce__ = external_define("GMNewton.dll","GmnBodyAddPointLocalForce",dll_cdecl,ty_real,7,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnBodyGetLocalOmega__ = external_define("GMNewton.dll","GmnBodyGetLocalOmega",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__createBuoyancyDefine__ = external_define("GMNewton.dll","createBuoyancyDefine",dll_cdecl,ty_real,6,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnMaterialSetBuoyancyCallback__ = external_define("GMNewton.dll","GmnMaterialSetBuoyancyCallback",dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnCreateNull__ = external_define("GMNewton.dll","GmnCreateNull",dll_cdecl,ty_real,1,ty_real);
global.__GmnCreateBox__ = external_define("GMNewton.dll","GmnCreateBox",dll_cdecl,ty_real,7,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnCreateSphere__ = external_define("GMNewton.dll","GmnCreateSphere",dll_cdecl,ty_real,7,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnCreateCone__ = external_define("GMNewton.dll","GmnCreateCone",dll_cdecl,ty_real,6,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnCreateCapsule__ = external_define("GMNewton.dll","GmnCreateCapsule",dll_cdecl,ty_real,6,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnCreateCylinder__ = external_define("GMNewton.dll","GmnCreateCylinder",dll_cdecl,ty_real,6,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnCreateChamferCylinder__ = external_define("GMNewton.dll","GmnCreateChamferCylinder",dll_cdecl,ty_real,6,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnConvexCollisionCalculateVolume__ = external_define("GMNewton.dll","GmnConvexCollisionCalculateVolume",dll_cdecl,ty_real,1,ty_real);
global.__GmnModelBufferClear__ = external_define("GMNewton.dll","GmnModelBufferClear",dll_cdecl,ty_real,0);
global.__GmnModelBufferAdd__ = external_define("GMNewton.dll","GmnModelBufferAdd",dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.__GmnModelBufferCount__ = external_define("GMNewton.dll","GmnModelBufferCount",dll_cdecl,ty_real,0);
global.__GmnCreateConvexHull__ = external_define("GMNewton.dll","GmnCreateConvexHull",dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnBeginConstructCompoundCollision__ = external_define("GMNewton.dll","GmnBeginConstructCompoundCollision",dll_cdecl,ty_real,0);
global.__GmnConstructCompoundCollisionAdd__ = external_define("GMNewton.dll","GmnConstructCompoundCollisionAdd",dll_cdecl,ty_real,1,ty_real);
global.__GmnEndConstructCompoundCollision__ = external_define("GMNewton.dll","GmnEndConstructCompoundCollision",dll_cdecl,ty_real,1,ty_real);
global.__GmnReleaseCollision__ = external_define("GMNewton.dll","GmnReleaseCollision",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnCollisionSetAsTriggerVolume__ = external_define("GMNewton.dll","GmnCollisionSetAsTriggerVolume",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnCustomConstraintCreateRigid__ = external_define("GMNewton.dll","GmnCustomConstraintCreateRigid",dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnCustomConstraintCreateDryRollingFriction__ = external_define("GMNewton.dll","GmnCustomConstraintCreateDryRollingFriction",dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.__GmnCreateCustomKinematicController__ = external_define("GMNewton.dll","GmnCreateCustomKinematicController",dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.__GmnCustomKinematicControllerSetMaxAngularFriction__ = external_define("GMNewton.dll","GmnCustomKinematicControllerSetMaxAngularFriction",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnCustomKinematicControllerSetMaxLinearFriction__ = external_define("GMNewton.dll","GmnCustomKinematicControllerSetMaxLinearFriction",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnCustomKinematicControllerSetPickMode__ = external_define("GMNewton.dll","GmnCustomKinematicControllerSetPickMode",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnCustomKinematicControllerSetTargetPosit__ = external_define("GMNewton.dll","GmnCustomKinematicControllerSetTargetPosit",dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.__GmnCustomKinematicControllerSetTargetRotation__ = external_define("GMNewton.dll","GmnCustomKinematicControllerSetTargetRotation",dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.__GmnConstraintCreateBall__ = external_define("GMNewton.dll","GmnConstraintCreateBall",dll_cdecl,ty_real,6,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnBallGetJointForce__ = external_define("GMNewton.dll","GmnBallGetJointForce",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnBallGetJointForceMag__ = external_define("GMNewton.dll","GmnBallGetJointForceMag",dll_cdecl,ty_real,1,ty_real);
global.__GmnConstraintCreateUpVector__ = external_define("GMNewton.dll","GmnConstraintCreateUpVector",dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnConstraintCreateHinge__ = external_define("GMNewton.dll","GmnConstraintCreateHinge",dll_cdecl,ty_real,9,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnCreateCustomSlider__ = external_define("GMNewton.dll","GmnCreateCustomSlider",dll_cdecl,ty_real,8,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnSliderEnableLimits__ = external_define("GMNewton.dll","GmnSliderEnableLimits",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnSliderSetLimits__ = external_define("GMNewton.dll","GmnSliderSetLimits",dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.__GmnDestroyJoint__ = external_define("GMNewton.dll","GmnDestroyJoint",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnCustomDestroyJoint__ = external_define("GMNewton.dll","GmnCustomDestroyJoint",dll_cdecl,ty_real,1,ty_real);
global.__GmnJointSetCollisionState__ = external_define("GMNewton.dll","GmnJointSetCollisionState",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnCustomJointSetCollisionState__ = external_define("GMNewton.dll","GmnCustomJointSetCollisionState",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnCreateCustomHinge__ = external_define("GMNewton.dll","GmnCreateCustomHinge",dll_cdecl,ty_real,8,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnCustomHingeSetLimits__ = external_define("GMNewton.dll","GmnCustomHingeSetLimits",dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.__GmnCustomHingeGetForceMagnitude__ = external_define("GMNewton.dll","GmnCustomHingeGetForceMagnitude",dll_cdecl,ty_real,1,ty_real);
global.__GmnCustomHingeGetTorqueMagnitude__ = external_define("GMNewton.dll","GmnCustomHingeGetTorqueMagnitude",dll_cdecl,ty_real,1,ty_real);
global.__GmnCustomHingeSetSeverCallscript__ = external_define("GMNewton.dll","GmnCustomHingeSetSeverCallscript",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnCustomHingeSetBreakCallscript__ = external_define("GMNewton.dll","GmnCustomHingeSetBreakCallscript",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnCustomHingeSetSeverTolerance__ = external_define("GMNewton.dll","GmnCustomHingeSetSeverTolerance",dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.__GmnCustomHingeSetSeverable__ = external_define("GMNewton.dll","GmnCustomHingeSetSeverable",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnCustomHingeSetLimitsBreakTolerance__ = external_define("GMNewton.dll","GmnCustomHingeSetLimitsBreakTolerance",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnCustomHingeSetLimitsBreakable__ = external_define("GMNewton.dll","GmnCustomHingeSetLimitsBreakable",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnCustomHingeGetLimitsBreakTolerance__ = external_define("GMNewton.dll","GmnCustomHingeGetLimitsBreakTolerance",dll_cdecl,ty_real,1,ty_real);
global.__GmnCreateCustomRigid__ = external_define("GMNewton.dll","GmnCreateCustomRigid",dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnCustomRigidGetForceMagnitude__ = external_define("GMNewton.dll","GmnCustomRigidGetForceMagnitude",dll_cdecl,ty_real,1,ty_real);
global.__GmnCustomRigidGetTorqueMagnitude__ = external_define("GMNewton.dll","GmnCustomRigidGetTorqueMagnitude",dll_cdecl,ty_real,1,ty_real);
global.__GmnCustomRigidSetSeverCallscript__ = external_define("GMNewton.dll","GmnCustomRigidSetSeverCallscript",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnCustomRigidSetSeverTolerance__ = external_define("GMNewton.dll","GmnCustomRigidSetSeverTolerance",dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.__GmnCustomRigidSetSeverable__ = external_define("GMNewton.dll","GmnCustomRigidSetSeverable",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnCreateCustomRigid__ = external_define("GMNewton.dll","GmnCreateCustomRigid",dll_cdecl,ty_real,9,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnCreateCustomPlayerController__ = external_define("GMNewton.dll","GmnCreateCustomPlayerController",dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.__GmnCustomPlayerControllerSetMaxSlope__ = external_define("GMNewton.dll","GmnCustomPlayerControllerSetMaxSlope",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnCustomPlayerControllerSetVelocity__ = external_define("GMNewton.dll","GmnCustomPlayerControllerSetVelocity",dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.__GmnMaterialGetDefaultGroupID__ = external_define("GMNewton.dll","GmnMaterialGetDefaultGroupID",dll_cdecl,ty_real,1,ty_real);
global.__GmnMaterialCreateGroupId__ = external_define("GMNewton.dll","GmnMaterialCreateGroupId",dll_cdecl,ty_real,1,ty_real);
global.__GmnMaterialDestroyAllGroupID__ = external_define("GMNewton.dll","GmnMaterialDestroyAllGroupID",dll_cdecl,ty_real,1,ty_real);
global.__GmnMaterialSetDefaultCollidable__ = external_define("GMNewton.dll","GmnMaterialSetDefaultCollidable",dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.__GmnMaterialSetContinuousCollisionMode__ = external_define("GMNewton.dll","GmnMaterialSetContinuousCollisionMode",dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.__GmnMaterialSetDefaultFriction__ = external_define("GMNewton.dll","GmnMaterialSetDefaultFriction",dll_cdecl,ty_real,5,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnMaterialSetDefaultElasticity__ = external_define("GMNewton.dll","GmnMaterialSetDefaultElasticity",dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.__GmnMaterialSetDefaultSoftness__ = external_define("GMNewton.dll","GmnMaterialSetDefaultSoftness",dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.__GmnMaterialSetCollisionCallback__ = external_define("GMNewton.dll","GmnMaterialSetCollisionCallback",dll_cdecl,ty_real,3,ty_real,ty_real,ty_real);
global.__GmnMaterialSetResponseType__ = external_define("GMNewton.dll","GmnMaterialSetResponseType",dll_cdecl,ty_real,4,ty_real,ty_real,ty_real,ty_real);
global.__GmnCollisionGet__ = external_define("GMNewton.dll","GmnCollisionGet",dll_cdecl,ty_real,0);
global.__GmnCollisionGetNext__ = external_define("GMNewton.dll","GmnCollisionGetNext",dll_cdecl,ty_real,1,ty_real);
global.__GmnCollisionGetObject__ = external_define("GMNewton.dll","GmnCollisionGetObject",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnCollisionGetContactCount__ = external_define("GMNewton.dll","GmnCollisionGetContactCount",dll_cdecl,ty_real,1,ty_real);
global.__GmnCollisionGetMaxImpactSpeed__ = external_define("GMNewton.dll","GmnCollisionGetMaxImpactSpeed",dll_cdecl,ty_real,1,ty_real);
global.__GmnCreateTreeCollision__ = external_define("GMNewton.dll","GmnCreateTreeCollision",dll_cdecl,ty_real,1,ty_real);
global.__GmnTreeCollisionBeginBuild__ = external_define("GMNewton.dll","GmnTreeCollisionBeginBuild",dll_cdecl,ty_real,1,ty_real);
global.__GmnTreeCollisionAddFace__ = external_define("GMNewton.dll","GmnTreeCollisionAddFace",dll_cdecl,ty_real,11,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnTreeCollisionEndBuild__ = external_define("GMNewton.dll","GmnTreeCollisionEndBuild",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnCreate__ = external_define("GMNewton.dll","GmnCreate",dll_cdecl,ty_real,0);
global.__GmnDestroy__ = external_define("GMNewton.dll","GmnDestroy",dll_cdecl,ty_real,1,ty_real);
global.__GmnSetPlatformArchitecture__ = external_define("GMNewton.dll","GmnSetPlatformArchitecture",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnSetSolverModel__ = external_define("GMNewton.dll","GmnSetSolverModel",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnSetFrictionModel__ = external_define("GMNewton.dll","GmnSetFrictionModel",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnUpdateFPS__ = external_define("GMNewton.dll","GmnUpdateFPS",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnUpdate__ = external_define("GMNewton.dll","GmnUpdate",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnSetMinimumFrameRate__ = external_define("GMNewton.dll","GmnSetMinimumFrameRate",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnGetTimeStep__ = external_define("GMNewton.dll","GmnGetTimeStep",dll_cdecl,ty_real,1,ty_real);
global.__GmnDestroyAllBodies__ = external_define("GMNewton.dll","GmnDestroyAllBodies",dll_cdecl,ty_real,1,ty_real);
global.__GmnSetWorldSize__ = external_define("GMNewton.dll","GmnSetWorldSize",dll_cdecl,ty_real,7,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnWorldFreezeBody__ = external_define("GMNewton.dll","GmnWorldFreezeBody",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnWorldUnfreezeBody__ = external_define("GMNewton.dll","GmnWorldUnfreezeBody",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnBodySetFreezeState__ = external_define("GMNewton.dll","GmnBodySetFreezeState",dll_cdecl,ty_real,2,ty_real,ty_real);
global.__GmnWorldGetBodyCount__ = external_define("GMNewton.dll","GmnWorldGetBodyCount",dll_cdecl,ty_real,1,ty_real);
global.__GmnWorldRayCastDist__ = external_define("GMNewton.dll","GmnWorldRayCastDist",dll_cdecl,ty_real,7,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);
global.__GmnWorldRayCastObject__ = external_define("GMNewton.dll","GmnWorldRayCastObject",dll_cdecl,ty_real,7,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real,ty_real);

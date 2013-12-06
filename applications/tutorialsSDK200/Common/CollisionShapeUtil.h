/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __COLLISION_SHAPE__H
#define __COLLISION_SHAPE__H


class Entity;

// Utility function to calculate the Bounding Box of a collision shape 
void CalculateBoxdingBox (const NewtonCollision* shape, dVector& boxMin, dVector& boxMax);

// display a collision Shape in wire frame
void ShowCollisionShape (const NewtonCollision* shape, const dMatrix& matrix);

// Create a centered Newton Collision From the Bounding Box of the Entity  
NewtonCollision* CreateNewtonBox (NewtonWorld* world, Entity *ent, int shapeId);

// Create a centered and oriented Newton Collision From the specified parameters 
NewtonCollision* CreateNewtonCapsule (NewtonWorld* world, Entity *ent, dFloat height, dFloat radius, int shapeId, const dMatrix& orientation);

// Create a centered and oriented Newton Collision From the specified parameters 
NewtonCollision* CreateNewtonCylinder (NewtonWorld* world, Entity *ent, dFloat height, dFloat radius, int shapeId, const dMatrix& orientation);


// create a convex collision Shape from the vertices of an entity
NewtonCollision* CreateNewtonConvex (NewtonWorld* world, Entity *ent, int shapeId);

// Create an optimized Mesh Collision form the entity Mesh
NewtonCollision* CreateMeshCollision (NewtonWorld* world, Entity* ent, int* shapeIdArray);

// Create a compound collision from each of the sub meshes of and entity
NewtonCollision* CreateNewtonCompoundFromEntitySubmesh (NewtonWorld* world, Entity* ent, int* shapeIdArray);

// Create a height field collision form a elevation file
NewtonCollision* CreateHeightFieldCollision (NewtonWorld* world, char* fileName, int* shapeIdArray);



#endif


/*
#include "stdafx.h"
#include <iostream>
#include <dVector.h>
#include <dMatrix.h>
#include <dNewton.h>
#include <dNewtonCollision.h>
#include <dNewtonDynamicBody.h>

// make a custom rigid body for the application to use as the base class of all it dynamics bodies
class MyDynamicBody: public dNewtonDynamicBody
{
	public:
	MyDynamicBody(dNewton* const world, dFloat mass, const dNewtonCollision* const collision, void* const userData, const dMatrix& matrix)
		:dNewtonDynamicBody (world, mass, collision, userData, &matrix[0][0], NULL)
	{
	}

	// the end application need to overload this function from dNetwonBody
	void OnBodyTransform (const dFloat* const matrix, int threadIndex)
	{
		Update (matrix);
	}

	// the end application need to overload this function from dNetwonDynamicBody
	void OnForceAndTorque (dFloat timestep, int threadIndex)
	{
		// apply gravity force to the body
		dFloat mass;
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;

		GetMassAndInertia (mass, Ixx, Iyy, Izz);
		dVector gravityForce (0.0f, -9.8f * mass, 0.0f, 0.0f);
		SetForce (&gravityForce[0]);
	}
};

dNewtonDynamicBody* CreateBackgroundBody(dNewton* const world)
{
	// make a flat quad 
	dFloat points[4][3] = 
	{
		{-100.0f, 0.0f,  100.0f}, 
		{ 100.0f, 0.0f,  100.0f}, 
		{ 100.0f, 0.0f, -100.0f}, 
		{-100.0f, 0.0f, -100.0f}, 
	};

	// create a collision tree instance with collision mask 1 
	dNewtonCollisionMesh collision (world, 1);

	// start building the collision mesh
	collision.BeginFace();

	// add the face one at a time
	collision.AddFace (4, &points[0][0], 3 * sizeof (dFloat), 0);

	// finish building the collision
	collision.EndFace();

	// create a body with a collision and locate at the identity matrix position 
	return new MyDynamicBody (world, 0, &collision, NULL, dGetIdentityMatrix());
}


static dNewtonDynamicBody* CreateFreeFallBall(dNewton* const world)
{
	// crate a collision sphere instance, of radio 1.0 and a collision mask 1 
	dNewtonCollisionSphere collision (world, 1.0f, 1);

	// create a dynamic body with a sphere shape, mass of 1.0 kg, located 50 units above the ground 
	dMatrix matrix (dGetIdentityMatrix());
	matrix.m_posit.m_y = 50.0f;
	dNewtonDynamicBody* const body = new MyDynamicBody (world, 1.0, &collision, NULL, matrix);

	// set the linear damping to zero
	body->SetLinearDrag (0.0f);

	return body;
}


int _tmain(int argc, _TCHAR* argv[])
{
	// create a newton world
	dNewton world;

	// create a static body to serve as the floor.
	dNewtonDynamicBody* const background = CreateBackgroundBody(&world);

	// create a free fall body
	dNewtonDynamicBody* const freeFallBall = CreateFreeFallBall (&world);

	// run the simulation loop
	for (int i = 0; i < 300; i ++) {
		// this function advance the simulation by the timestep, if another call is made before the time step of the previews call is exhausted, the call is ignored
		//world.Update (1.0f/60.f);

		// this function will advance the simulation by the timestep, on each call
		world.UpdateOffLine (1.0f/60.f);

		dMatrix matrix;
		freeFallBall->InterplateMatrix (1.0f, &matrix[0][0]);
		std::cout << "height: " << matrix.m_posit.m_y << std::endl;
	}

	return 0;
}

*/



/* 
 * File:   main.cpp
 * Author: main
 *
 * Created on 24 de julio de 2014, 21:17
 */



//-----------------------------------------------------------------------------
//#include <cstdlib>
//#include <iostream>
//#include "newton/dNewton/dNewton.h"
//#include "newton/dNewton/dNewtonDynamicBody.h"
//#include "newton/dNewton/dNewtonCollision.h"
//#include "newton/dNewton/dNewtonPlayerManager.h"


#include "stdafx.h"
#include <iostream>
#include <dVector.h>
#include <dMatrix.h>
#include <dNewton.h>
#include <dNewtonCollision.h>
#include <dNewtonDynamicBody.h>
#include <dNewtonPlayerManager.h>


//-----------------------------------------------------------------------------
using namespace std;
//-----------------------------------------------------------------------------
class MyDynamicBody: public dNewtonDynamicBody
{
   public:
   MyDynamicBody(dNewton* const world, dFloat mass,
                const dNewtonCollision* const collision, void* const userData,
                const dMatrix& matrix)
      :dNewtonDynamicBody (world, mass, collision, userData, &matrix[0][0], NULL)
   {
   }

   // the end application need to overload this function from dNetwonBody
   void OnBodyTransform (const dFloat* const matrix, int threadIndex)
   {
      Update (matrix);
   }

   // the end application need to overload this function from dNetwonDynamicBody
   void OnForceAndTorque (dFloat timestep, int threadIndex)
   {
      // apply gravity force to the body
      dFloat mass;
      dFloat Ixx;
      dFloat Iyy;
      dFloat Izz;

      GetMassAndInertia (mass, Ixx, Iyy, Izz);
      dVector gravityForce (0.0f, -9.8f * mass, 0.0f);
      SetForce (&gravityForce[0]);
   }
};
//-----------------------------------------------------------------------------
class MyPlayer : public dNewtonPlayerManager::dNewtonPlayer
{
public:
    MyPlayer(dNewtonPlayerManager* const manager, void* const userData, dFloat mass,
            dFloat outerRadius, dFloat innerRadius, dFloat height, dFloat stairStep,
            const dFloat* const upDir, const dFloat* const frontDir, dLong collisionMask)
            : dNewtonPlayer(manager, userData, mass, outerRadius, innerRadius,
            height, stairStep, upDir, frontDir, collisionMask)
    {
    }
    
    ~MyPlayer()
    {
    }
    
    void OnPlayerMove (dFloat timestep)
    {
        dVector gravity(0.0f, -9.81f, 0.0f);
        SetPlayerVelocity (0.0f, 0.0f, 0.0f, 0.0f, &gravity[0], timestep);
    }
};
//-----------------------------------------------------------------------------
int main(int argc, char** argv)
{
    dNewton* world = new dNewton();

    // create floor
    dNewtonCollisionBox floor_collision (world, 100, 1, 100, 1);
    
    floor_collision.SetScale(1, 2, 1); // If I uncomment this line dNewtonPlayer->CustomPlayerController->m_isJumping never change.
    
    MyDynamicBody* floor = new MyDynamicBody (world, 0, &floor_collision, NULL, dGetIdentityMatrix());
    
    // create player
    dFloat mass = 100.0f;
    dFloat outerRadius = 1;
    dFloat innerRadius = 0.25;
    dFloat height = 2.0f;
    dFloat stairStep = 0.5;    
    
    dVector upDir(0, 1, 0);
    dVector frontDir(0, 0, 1);
    
    dNewtonPlayerManager* manager = new dNewtonPlayerManager(world);
    MyPlayer* player = new MyPlayer(manager, NULL, 
            mass, outerRadius, innerRadius, height, stairStep, &upDir[0], &frontDir[0], 1);
    
    dMatrix matrix;
    player->GetMatrix(&matrix[0][0]);
    matrix.m_posit.m_y = 5.0f;
    player->SetMatrix(&matrix[0][0]);
    
    std::cout << "height: " << std::endl;
    
    while(true)
    {
        world->UpdateOffLine(1.0f/60.f);
        dMatrix matrix;
		player->InterplateMatrix(1.0f, &matrix[0][0]);
		std::cout << matrix.m_posit.m_y << " ";
    }

    return 0;
}


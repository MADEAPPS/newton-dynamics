/* 
 * File:   main.cpp
 * Author: main
 *
 * Created on 24 de julio de 2014, 21:17
 */

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
      dVector gravityForce (0.0f, -9.8f * mass, 0.0f);
      SetForce (&gravityForce[0]);
   }
};

//-----------------------------------------------------------------------------
class MyPlayer : public dNewtonPlayerManager::dNewtonPlayer
{
	public:
    MyPlayer(dNewtonPlayerManager* const manager, void* const userData, dFloat mass, dFloat outerRadius, dFloat innerRadius, dFloat height, dFloat stairStep, const dFloat* const upDir, const dFloat* const frontDir, dLong collisionMask)
            : dNewtonPlayer(manager, userData, mass, outerRadius, innerRadius, height, stairStep, upDir, frontDir, collisionMask)
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
    dNewton world;

    // create floor
    dNewtonCollisionBox floor_collision (&world, 100, 1, 100, 1);
    
    //floor_collision.SetScale(1, 2, 1); 
    MyDynamicBody* const floor = new MyDynamicBody (&world, 0, &floor_collision, NULL, dGetIdentityMatrix());
    
    // create player
    dFloat mass = 100.0f;
    dFloat outerRadius = 1;
    dFloat innerRadius = 0.25;
    dFloat height = 2.0f;
    dFloat stairStep = 0.5;    
    
    dVector upDir(0, 1, 0);
    dVector frontDir(0, 0, 1);
    
    dNewtonPlayerManager* const manager = new dNewtonPlayerManager(&world);
    MyPlayer* const player = new MyPlayer(manager, NULL, mass, outerRadius, innerRadius, height, stairStep, &upDir[0], &frontDir[0], 1);
    
    dMatrix matrix;
    player->GetMatrix(&matrix[0][0]);
    matrix.m_posit.m_y = 5.0f;
    player->SetMatrix(&matrix[0][0]);
    
    std::cout << "height: " << std::endl;
    
    for (int i = 0; i < 1500; i ++)
    {
        world.UpdateOffLine(1.0f/60.f);
        dMatrix matrix;
		player->InterpolateMatrix(1.0f, &matrix[0][0]);
		std::cout << matrix.m_posit.m_y << " ";
    }

    return 0;
}


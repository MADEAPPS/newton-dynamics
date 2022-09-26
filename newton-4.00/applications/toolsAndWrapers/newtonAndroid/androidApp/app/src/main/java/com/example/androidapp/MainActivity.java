package com.example.androidapp;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;

import com.javaNewton.nWorld;
import com.javaNewton.nRigidBody;
import com.javaNewton.nBodyNotify;
import com.javaNewton.nShapeInstance;

import com.newton.nVector;
import com.newton.nMatrix;
import com.newton.nShapeBox;
import com.newton.nRigidBodyType;

public class MainActivity extends AppCompatActivity
{
    // make sure you load the engine dynamic library
    static
    {
        System.loadLibrary("ndNewton");
    }

   nWorld world;

    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);

		// create an instance of the newton engine
		world = new nWorld();
		world.SetSubSteps(2);
		
		TestEngine();
    }

	protected void AddFloor()
	{
		nMatrix location = new nMatrix();
		location.SetIdentity();
		location.Set(3, new nVector(0.0f, -0.5f, 0.0f, 1.0f));

		nRigidBody floor = new nRigidBody(nRigidBodyType.m_dynamic);
		nShapeInstance box = new nShapeInstance(new nShapeBox(200.0f, 1.0f, 200.f));

		floor.SetNotifyCallback(new nBodyNotify());
		floor.SetMatrix(location);
		floor.SetCollisionShape(box);
		//body->SetMassMatrix(mass, box);
		world.AddBody(floor);
	}

	protected void TestEngine()
	{
		world.Sync();

		AddFloor();

		//ndFloat32 totalTime = 0;
		for (int i = 0; i < 100; i++)
		{
			//if (i == 0) world.AddJoint(&joint);
			//if (i == 2) world.RemoveJoint(&joint);
			world.Update(1.0f / 60.0f);
			//totalTime += world.GetUpdateTime();
			world.Sync();
		}
	}
}
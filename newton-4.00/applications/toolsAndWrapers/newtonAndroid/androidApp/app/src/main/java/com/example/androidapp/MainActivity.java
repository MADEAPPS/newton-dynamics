package com.example.androidapp;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;

import com.javaNewton.nMatrix;
import com.javaNewton.nWorld;
import com.javaNewton.nRigidBody;
import com.javaNewton.nBodyNotify;
import com.javaNewton.nShapeInstance;

import com.newton.nVector;
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
		nShapeInstance boxShape = new nShapeInstance(new nShapeBox(200.0f, 1.0f, 200.0f));

		floor.SetNotifyCallback(new nBodyNotify());
		floor.SetMatrix(location);
		floor.SetCollisionShape(boxShape);
		world.AddBody(floor);
	}

	protected void AddBox()
	{
		nMatrix location = new nMatrix();
		location.SetIdentity();
		location.Set(3, new nVector(0.0f, 5.0f, 0.0f, 1.0f));

		nRigidBody box = new nRigidBody(nRigidBodyType.m_dynamic);
		nShapeInstance boxShape = new nShapeInstance(new nShapeBox(0.5f, 0.5f, 0.5f));

		box.SetNotifyCallback(new nBodyNotify());
		box.SetMatrix(location);
		box.SetCollisionShape(boxShape);
		box.SetMassMatrix(1.0f, boxShape);
		world.AddBody(box);
	}

	protected void TestEngine()
	{
		world.Sync();

		AddFloor();
		AddBox();
		for (int i = 0; i < 100; i++)
		{
			world.Update(1.0f / 60.0f);
			world.Sync();
		}
	}
}
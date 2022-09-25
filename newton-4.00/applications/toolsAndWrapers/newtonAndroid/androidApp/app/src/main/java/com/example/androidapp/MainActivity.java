package com.example.androidapp;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;

import com.newton.nWorld;
import com.newton.nVector;
import com.newton.nMatrix;
import com.newton.nShapeBox;
import com.newton.nRigidBody;
import com.newton.nBodyNotify;
import com.newton.nShapeInstance;

import com.javaNewton.RigidBody;
import com.javaNewton.NewtonWorld;

public class MainActivity extends AppCompatActivity
{
    // make sure you load the engine dynamic library
    static
    {
        System.loadLibrary("ndNewton");
    }

   NewtonWorld world;

    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);

		// create an instance of the newton engine
		world = new NewtonWorld();
		world.SetSubSteps(2);
		

		TestEngine();
    }

	protected void TestEngine()
    {
	    world.Sync();

		nMatrix location = new nMatrix();
		location.SetIdentity();
		location.Set(3, new nVector(0.0f, -0.5f, 0.0f, 1.0f));
		
		RigidBody floor = new RigidBody(nRigidBody.Type.m_dynamic);
		nShapeInstance box = new nShapeInstance(new nShapeBox(200.0f, 1.0f, 200.f));

		floor.SetNotifyCallback(new nBodyNotify());
		floor.SetMatrix(location);
		floor.SetCollisionShape(box);
		//body->SetMassMatrix(mass, box);
		world.AddBody(floor);
    }
}
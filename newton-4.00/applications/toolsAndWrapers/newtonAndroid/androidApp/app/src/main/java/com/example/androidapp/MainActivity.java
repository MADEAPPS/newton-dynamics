package com.example.androidapp;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;

import com.newton.World;
import com.newton.Vector4;
import com.newton.Matrix4;
import com.newton.ShapeBox;
import com.newton.ShapeInstance;

public class MainActivity extends AppCompatActivity
{
    // make sure you load the engine dynamic library
    static
    {
        System.loadLibrary("ndNewton");
    }

    World world;

    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);

		// create an instance of the newton engine
		world = new World();
		world.Sync();

		TestEngine();
    }

	protected void TestEngine()
    {
	    world.SetSubSteps(2);

		ShapeInstance box = new ShapeInstance(new ShapeBox(200.0f, 1.0f, 200.f));

	    Matrix4 xxx = new Matrix4();
	    Vector4 xxx1 = new Vector4(1.0f, 2.0f, 4.0f, 0.0f);

		xxx.SetIdentity();
        //ndVector size(0.5f, 0.25f, 0.8f, 0.0f);
        //ndVector origin(0.0f, 0.0f, 0.0f, 0.0f);

        //ndBodyDynamic* bodyFloor = BuildFloorBox(world);
    }
}
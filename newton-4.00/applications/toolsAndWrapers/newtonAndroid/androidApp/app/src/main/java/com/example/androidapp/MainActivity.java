package com.example.androidapp;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;

import com.newton.ndWorld;
import com.newton.ndVector4;
import com.newton.ndMatrix;

public class MainActivity extends AppCompatActivity
{
    // make sure you load the engine dynamic library
    static
    {
        System.loadLibrary("ndNewton");
    }

    ndWorld world;

    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);

		// create an instance of the newton engine
		world = new ndWorld();
		world.Sync();

		TestEngine();
    }

	protected void TestEngine()
    {
	//world.SetSubSteps(1);

	//ndMatrix xxx = new ndMatrix();
	ndVector4 xxx1 = new ndVector4(1.0f, 2.0f, 4.0f, 0.0f);
        //ndVector size(0.5f, 0.25f, 0.8f, 0.0f);
        //ndVector origin(0.0f, 0.0f, 0.0f, 0.0f);

        //ndBodyDynamic* bodyFloor = BuildFloorBox(world);
    }
}
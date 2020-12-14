package com.eventapp.openquad2;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.util.Log;

public class MainActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        JoystickView joystick = new JoystickView(this);
        setContentView(R.layout.activity_main);
    }

    public void onJoystickMoved(float xPercent, float yPercent, int id) {
        switch (id) {
            case R.id.joystickRight:
                Log.d("Right Joystick", "X Percent in main: " + xPercent + " Y Percent in main: " + yPercent);
                break;
            case R.id.joystickLeft:
                Log.d("Left Joystick", "X Percent in main: " + xPercent + " Y Percent in main: " + yPercent);
                break;
        }

    }
}

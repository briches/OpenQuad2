package com.eventapp.openquad2;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PorterDuff;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;

public class JoystickView extends SurfaceView implements SurfaceHolder.Callback, View.OnTouchListener {
    private float centerX;
    private float centerY;
    private float baseRadius;
    private float hatRadius;
    private JoystickListener joystickCallback;
    private final int ratio = 5;

    private void setupDimensions() {
        centerX = getWidth() / 4 ; //screen size / 4 (centered left, and centered right for 2 joysticks
        centerY = getHeight() / 8; // screen size / 8 (bottom of the screen)
        baseRadius = Math.min(getWidth(), getHeight()) / 3;
        hatRadius = Math.min(getWidth(), getHeight()) / 5; //hat needs to be smaller than the base.
    }

    public JoystickView(Context context) {
        super(context);
        getHolder().addCallback(this);
        setOnTouchListener(this);
    }

    public JoystickView(Context context, AttributeSet attributes, int style) {
        super(context, attributes, style);
        getHolder().addCallback(this);
    }

    public JoystickView(Context context, AttributeSet attributes) {
        super(context, attributes);
    }

    public void drawJoystick(float newX, float newY) {
        if (getHolder().getSurface().isValid()) {
            //I need a canvas to draw on
            Canvas myCanvas = this.getHolder().lockCanvas();
            Paint colors = new Paint();
            myCanvas.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR);

            //Calculating triangles wherever the hat is moved to.
            float hypotenuse = (float) Math.sqrt(Math.pow(newX- centerX, 2) + Math.pow(newY - centerY, 2));
            float sin = (newY - centerY) / hypotenuse;
            float cos = (newX - centerX) / hypotenuse;

            colors.setARGB(255, 50, 50 , 50);
            myCanvas.drawCircle(centerX, centerY, baseRadius, colors);

            for(int i = 1; i <= (int) (baseRadius / ratio); i++) {
                colors.setARGB(150, 255, 0, 0);
                myCanvas.drawCircle(newX - cos * hypotenuse * (ratio/baseRadius) * i,
                                    newY - sin * hypotenuse * (ratio/baseRadius) * i,
                                    i * (hatRadius * ratio / baseRadius), colors);
            }

            for(int i = 1; i < (int) (hatRadius / ratio); i++){
                colors.setARGB(255, 50, 50, 50);
                myCanvas.drawCircle(newX, newY, hatRadius - (float) i * (ratio) / 2, colors);
            }
            myCanvas.drawCircle(newX, newY, hatRadius, colors);

            getHolder().unlockCanvasAndPost(myCanvas);
        }
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        setupDimensions();
        drawJoystick(centerX, centerY);
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {

    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {

    }

    public boolean onTouch(View view, MotionEvent motionEvent) {
        if(view.equals(this)) {
            if(motionEvent.getAction() != motionEvent.ACTION_UP) {
                float displacement = (float) Math.sqrt((Math.pow(motionEvent.getX() - centerX, 2) + Math.pow(motionEvent.getY() - centerY, 2)));
                if (displacement < baseRadius) {
                    drawJoystick(motionEvent.getX(), motionEvent.getY());
                    joystickCallback.onJoystickMoved((motionEvent.getX() - centerX)/baseRadius, (motionEvent.getY() - centerY)/baseRadius, getId());
                }
                else {
                    float ratio = baseRadius / displacement;
                    float constrainedX = centerX + (motionEvent.getX() - centerX) * ratio;
                    float constrainedY = centerY + (motionEvent.getY() - centerY) * ratio;
                    drawJoystick(constrainedX, constrainedY);
                    joystickCallback.onJoystickMoved((constrainedX - centerX)/baseRadius, (constrainedY - centerY)/baseRadius, getId());
                }
            }
            else {
                drawJoystick(centerX, centerY);
                joystickCallback.onJoystickMoved(0,0, getId());
            }
        }
        return true;
    }

    public interface JoystickListener {
        void onJoystickMoved(float xPercent, float yPercent, int id);
    }
}

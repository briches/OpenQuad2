import peasy.*;
import processing.serial.*;
import java.io.*; 
import java.util.*; 

static PShape quad;
static Serial myPort;

static final float PITCH_INCREMENT_RAD = (PI/64);
static final float YAW_INCREMENT_RAD   = (PI/64);
static final float ROLL_INCREMENT_RAD  = (PI/64);

static final float PITCH_DEFAULT = (PI/2);
static final float YAW_DEFAULT = (PI/4);
static final float ROLL_DEFAULT = (0);

static float g_pitch = PITCH_DEFAULT;
static float g_yaw = YAW_DEFAULT;
static float g_roll = ROLL_DEFAULT;

void setup() {

    //Set up the serial port
    println(Serial.list());
    myPort = new Serial(this, Serial.list()[0], 1000000);
    print("Connected to serial port: ");
    println(Serial.list()[0]);

    size(640,480,P3D);
    println("Test");
    quad = loadShape("Frame v9.obj");
    lights();
}

void draw() {


    background(0);
    
    pushMatrix();
    translate(width/2, height/2, 0);
    rotateX(g_pitch);
    rotateY(g_roll);
    rotateZ(g_yaw);
    shape(quad, 0, 0);
    popMatrix();
}

void keyPressed() {

    println("pitch: ", g_pitch, " roll: ", g_roll, " yaw: ", g_yaw);

    switch(key)
    {
        case 'w':
            g_pitch += PITCH_INCREMENT_RAD;
        break;

        case 's':
            g_pitch -= PITCH_INCREMENT_RAD;
        break;

        case 'a':
            g_roll -= ROLL_INCREMENT_RAD;
            break;

        case 'd':
            g_roll += ROLL_INCREMENT_RAD;
            break;

        case 'q':
            g_yaw -= YAW_INCREMENT_RAD;
            break;

        case 'e':
            g_yaw += YAW_INCREMENT_RAD;
            break;

        case 'h':
            println("Home position");
            g_pitch = PITCH_DEFAULT;
            g_yaw = YAW_DEFAULT;
            g_roll = ROLL_DEFAULT;
            break;

        default:
            break;
    }
}

/*================================================================================
     Serial Event: Used to RX data
     -----------------------------------------------------------------------------*/
void serialEvent(Serial port)
{
    if(port.available() > 0)
    {
        // connection = true;
        // connectTime = millis();
        
        String readline = port.readStringUntil('\n');
        if(readline != null)
        {
            if(readline.contains("[kinematics]"))
            {
                String[] values = readline.split(",");
                if(values.length == 3)
                {
                    print(values[0], values[1], values[2]);
                    g_pitch = (PI/180) * Float.parseFloat(values[1]) + PITCH_DEFAULT;
                    g_roll = (PI/180) * Float.parseFloat(values[2]) + ROLL_DEFAULT;
                }
            }
        }
    }
}
import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import processing.serial.*; 
import processing.net.*; 
import java.io.*; 
import java.nio.*; 
import java.util.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class oq2_ui extends PApplet {



 

 

private static final char[] HEX_ARRAY = "0123456789ABCDEF".toCharArray();

Server myServer;
byte[] byteBuffer = new byte[4096];

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

public void setup() {

    //Set up the serial port
    // println(Serial.list());
    // myPort = new Serial(this, Serial.list()[0], 1000000);
    // print("Connected to serial port: ");
    // println(Serial.list()[0]);

    // size(640,480,P3D);
    println("Test");
    quad = loadShape("Frame v9.obj");
    lights();

    myServer = new Server(this, 1337, "192.168.1.65");
}


public static String bytesToHexString(byte[] bytes, int numbytes) 
{
    char[] hexChars = new char[numbytes * 3];

    for (int j = 0; j < numbytes; j++) 
    {
        int v = bytes[j] & 0xFF;
        hexChars[j * 3] = HEX_ARRAY[v >>> 4];
        hexChars[j * 3 + 1] = HEX_ARRAY[v & 0x0F];
        hexChars[j * 3 + 2] = ' ';
    }
    return new String(hexChars);
}

boolean firstConnect = false;

public void draw() 
{
    // background(0);
    
    // pushMatrix();
    // translate(width/2, height/2, 0);
    // rotateX(g_pitch);
    // rotateY(g_roll);
    // rotateZ(g_yaw);
    // shape(quad, 0, 0);
    // popMatrix();

    Client thisClient = myServer.available();
    // If the client is not null, and says something, display what it said
    if (thisClient != null) 
    {
        println(thisClient);

        int rxByteCount = thisClient.readBytes(byteBuffer);

        if (rxByteCount > 0) 
        {
            println(thisClient.ip() + "\t" + bytesToHexString(byteBuffer, rxByteCount));

            if(firstConnect == false)
            {
                firstConnect = true;
                oq2p_exhaustive_test(thisClient, 500);
            }
        }
    }
}

public void keyPressed() 
{

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
     Server Event: Created when a new client connects to the server
     -----------------------------------------------------------------------------*/
public void serverEvent(Server someServer, Client someClient) 
{
  println("We have a new client: " + someClient.ip());
}

/*================================================================================
     Serial Event: Used to RX data
     -----------------------------------------------------------------------------*/
public void serialEvent(Serial port)
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
public void oq2p_exhaustive_test(Client _socket, int inter_test_period)
{
    // Arm motors 1 by 1
    _socket.write( oq2p_arm_message(OQ2P_MOTOR_INDEX_1, OQ2P_MOTOR_ARM) );
    delay(inter_test_period);
    _socket.write( oq2p_arm_message(OQ2P_MOTOR_INDEX_2, OQ2P_MOTOR_ARM) );
    delay(inter_test_period);
    _socket.write( oq2p_arm_message(OQ2P_MOTOR_INDEX_3, OQ2P_MOTOR_ARM) );
    delay(inter_test_period);
    _socket.write( oq2p_arm_message(OQ2P_MOTOR_INDEX_4, OQ2P_MOTOR_ARM) );
    delay(inter_test_period);

    // Disarm motors 1 by 1
    _socket.write( oq2p_arm_message(OQ2P_MOTOR_INDEX_1, OQ2P_MOTOR_DISARM) );
    delay(inter_test_period);
    _socket.write( oq2p_arm_message(OQ2P_MOTOR_INDEX_2, OQ2P_MOTOR_DISARM) );
    delay(inter_test_period);
    _socket.write( oq2p_arm_message(OQ2P_MOTOR_INDEX_3, OQ2P_MOTOR_DISARM) );
    delay(inter_test_period);
    _socket.write( oq2p_arm_message(OQ2P_MOTOR_INDEX_4, OQ2P_MOTOR_DISARM) );
    delay(inter_test_period);

    // Arm all
    _socket.write( oq2p_arm_message(OQ2P_MOTOR_INDEX_ALL, OQ2P_MOTOR_ARM) );
    delay(inter_test_period);

    // Disarm all
    // _socket.write( oq2p_arm_message(OQ2P_MOTOR_INDEX_ALL, OQ2P_MOTOR_DISARM) );
    // delay(inter_test_period);

    // Query armed state
    _socket.write( oq2p_arm_state_request());
    delay(inter_test_period);
}
static final byte OQ2P_START = (byte)0xCA;
static final byte OQ2P_END = (byte)0xFE;

static final byte OQ2P_CLASS_CONTROL = (byte)0x02;

static final byte OQ2P_TYPE_COMMAND = (byte)0x00;
static final byte OQ2P_TYPE_REQUEST = (byte)0x01;
static final byte OQ2P_TYPE_RESPONSE = (byte)0x02;


static final byte OQ2P_MOTOR_INDEX_ALL  = (byte)0xFF;
static final byte OQ2P_MOTOR_INDEX_1  = (byte)0x01;
static final byte OQ2P_MOTOR_INDEX_2  = (byte)0x02;
static final byte OQ2P_MOTOR_INDEX_3  = (byte)0x03;
static final byte OQ2P_MOTOR_INDEX_4  = (byte)0x04;

static final byte OQ2P_MOTOR_ARM        = (byte)0x01;
static final byte OQ2P_MOTOR_DISARM     = (byte)0x00;


/**
 * 
 */
public byte[] oq2p_arm_message(byte index, byte arm)
{
    byte buffer[] = new byte[9];
    
    // Header
    buffer[0] = OQ2P_START;
    buffer[1] = OQ2P_CLASS_CONTROL;
    buffer[2] = (byte)0x00;    // Message ARM
    buffer[3] = OQ2P_TYPE_COMMAND;
    buffer[4] = (byte)0x00;    // Length MSB
    buffer[5] = (byte)0x02;    // Length LSB
    
    // Data
    buffer[6] = arm;
    buffer[7] = index;

    // Footer
    buffer[8] = OQ2P_END;

    return buffer;
}

public byte[] oq2p_arm_state_request()
{
    byte buffer[] = new byte[7];
    
    // Header
    buffer[0] = OQ2P_START;
    buffer[1] = OQ2P_CLASS_CONTROL;
    buffer[2] = (byte)0x00;    // Message ARM
    buffer[3] = OQ2P_TYPE_REQUEST;
    buffer[4] = (byte)0x00;    // Length MSB
    buffer[5] = (byte)0x00;    // Length LSB

    // Footer
    buffer[6] = OQ2P_END;

    return buffer;
}
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "oq2_ui" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}

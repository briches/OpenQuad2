import processing.serial.*;
import processing.net.*;
import java.io.*; 
import java.nio.*;
import java.util.*; 

static final float PITCH_INCREMENT_RAD = (PI/180);
static final float YAW_INCREMENT_RAD   = (PI/180);
static final float ROLL_INCREMENT_RAD  = (PI/180);

static final float PITCH_DEFAULT = 0;
static final float YAW_DEFAULT = 0;
static final float ROLL_DEFAULT = 0;

static float g_pitch = PITCH_DEFAULT;
static float g_yaw = YAW_DEFAULT;
static float g_roll = ROLL_DEFAULT;

static PShape quad; 
static Serial myPort;
static Server mServer;
static Client quadSocket = null;
static boolean connectedStatus = false;

// Used to set up a simple fifo buffer of messages to send to the quad
static ArrayList<byte[]> messageList;

static byte armed1;
static byte armed2;
static byte armed3;
static byte armed4;

// Graphical locations and definitions
static final int quadModelX = 466;
static final int quadModelY = 466;
static final int quadBoxX = 520;
static final int quadBoxY = 520;
static final int quadBoxZ = -200;
static final int quadBoxSize = 400;


public boolean read_from_socket(Client read_client)
{
    byte msg[] = read_client.readBytes();

    if (msg != null) 
    {
        println(read_client.ip() + "\t" + bytesToHexString(msg, msg.length));
        return true;
    }

    return false;
}

void setup() {

    armed1 = OQ2P_MOTOR_DISARM;
    armed2 = OQ2P_MOTOR_DISARM;
    armed3 = OQ2P_MOTOR_DISARM;
    armed4 = OQ2P_MOTOR_DISARM;

    messageList = new ArrayList<byte[]>();

    size(640,640,P3D);
    println("Test");
    quad = loadShape("Frame v9.obj");

    mServer = new Server(this, 1337, "192.168.1.65");
}


public void draw() 
{
    background(200);
    lights();
    
    // Add quad
    pushMatrix();
    translate( quadModelX, quadModelY, 0);
    rotateX(g_pitch);
    rotateY(g_roll);
    rotateZ(g_yaw);
    shape(quad, 0, 0);
    popMatrix();

    // Add Box around quad
    translate( quadBoxX, quadBoxY, quadBoxZ);
    fill(255);
    rectMode(CENTER);
    rect(0, 0, quadBoxSize, quadBoxSize);



    // If the client is not null, and says something, display what it said
    if (connectedStatus == true) 
    {
        try
        {
            if(messageList.size() > 0)
            {
                byte[] message = messageList.get(0);

                println("Sending\t\t" + bytesToHexString(message, message.length));

                quadSocket.write(message);

                messageList.remove(0);
            }

            if(quadSocket.available() > 0)
            {
                read_from_socket(quadSocket);
            }

            if(quadSocket.active() == false)
            {
                println("Disconnecting from quad");
                quadSocket.stop();
                quadSocket = null;
                connectedStatus = false;
            }
        }
        catch (Exception e) 
        {
            println("Exception: " + e);

            connectedStatus = false;
        }
    }
}

/*================================================================================
    Keyboard event: Created when a new client connects to the Server
    -----------------------------------------------------------------------------*/
public void keyPressed() 
{

    // println("pitch: ", g_pitch, " roll: ", g_roll, " yaw: ", g_yaw);

    switch(key)
    {
        case '1':
            if(armed1 == OQ2P_MOTOR_DISARM)
            {
                armed1 = OQ2P_MOTOR_ARM;
                println("ARM 1");
                messageList.add(oq2p_arm_message(OQ2P_MOTOR_INDEX_1, OQ2P_MOTOR_ARM) );
            }
            else
            {
                armed1 = OQ2P_MOTOR_DISARM;
                println("DISARM 1");
                messageList.add(oq2p_arm_message(OQ2P_MOTOR_INDEX_1, OQ2P_MOTOR_DISARM) );
            }
        break;

        case '2':
            if(armed2 == OQ2P_MOTOR_DISARM)
            {
                armed2 = OQ2P_MOTOR_ARM;
                println("ARM 2");
                messageList.add(oq2p_arm_message(OQ2P_MOTOR_INDEX_2, OQ2P_MOTOR_ARM) );
            }
            else
            {
                armed2 = OQ2P_MOTOR_DISARM;
                println("DISARM 2");
                messageList.add(oq2p_arm_message(OQ2P_MOTOR_INDEX_2, OQ2P_MOTOR_DISARM) );
            }
        break;

        case '3':
            if(armed3 == OQ2P_MOTOR_DISARM)
            {
                armed3 = OQ2P_MOTOR_ARM;
                println("ARM 3");
                messageList.add(oq2p_arm_message(OQ2P_MOTOR_INDEX_3, OQ2P_MOTOR_ARM) );
            }
            else
            {
                armed3 = OQ2P_MOTOR_DISARM;
                println("DISARM 3");
                messageList.add(oq2p_arm_message(OQ2P_MOTOR_INDEX_3, OQ2P_MOTOR_DISARM) );
            }
        break;

        case '4':
            if(armed4 == OQ2P_MOTOR_DISARM)
            {
                armed4 = OQ2P_MOTOR_ARM;
                println("ARM 4");
                messageList.add(oq2p_arm_message(OQ2P_MOTOR_INDEX_4, OQ2P_MOTOR_ARM) );
            }
            else
            {
                armed4 = OQ2P_MOTOR_DISARM;
                println("DISARM 4");
                messageList.add(oq2p_arm_message(OQ2P_MOTOR_INDEX_4, OQ2P_MOTOR_DISARM) );
            }
        break;

        case 'w':
        {
            g_pitch += PITCH_INCREMENT_RAD;
            int degress100 = (int)((180 / PI) * g_pitch * 100);
            messageList.add(oq2p_setpoint_command(OQ2P_MID_PITCH_SET, degress100));
        }
        break;

        case 's':
        {
            g_pitch -= PITCH_INCREMENT_RAD;
            int degress100 = (int)((180 / PI) * g_pitch * 100);
            messageList.add(oq2p_setpoint_command(OQ2P_MID_PITCH_SET, degress100));
        }
        break;

        case 'a':
        {
            g_roll -= ROLL_INCREMENT_RAD;
            int degress100 = (int)((180 / PI) * g_roll * 100);
            messageList.add(oq2p_setpoint_command(OQ2P_MID_ROLL_SET, degress100));
        } 
        break;

        case 'd':
        {
            g_roll += ROLL_INCREMENT_RAD;
            int degress100 = (int)((180 / PI) * g_roll * 100);
            messageList.add(oq2p_setpoint_command(OQ2P_MID_ROLL_SET, degress100));
        }  
        break;

        case 'q':
        {
            g_yaw -= YAW_INCREMENT_RAD;
            int degress100 = (int)((180 / PI) * g_yaw * 100);
            messageList.add(oq2p_setpoint_command(OQ2P_MID_YAW_SET, degress100));
        } 
        break;

        case 'e':
        {
            g_yaw += YAW_INCREMENT_RAD;
            int degress100 = (int)((180 / PI) * g_yaw * 100);
            messageList.add(oq2p_setpoint_command(OQ2P_MID_YAW_SET, degress100));
        }  break;

        case 'h':
            println("Home position");
            g_pitch = PITCH_DEFAULT;
            g_yaw = YAW_DEFAULT;
            g_roll = ROLL_DEFAULT;
            break;

        default:
            break;
    }

    if(key == CODED)
    {
        switch(keyCode)
        {
            case SHIFT:
            {
                println("Shift");
            } break;

            case CONTROL:
            {
                println("CTRL");
            } break;
        }
    }
}

/*================================================================================
    Server Event: Created when a new client connects to the mServer
    -----------------------------------------------------------------------------*/
void serverEvent(Server someServer, Client someClient) 
{
    if(connectedStatus == false)
    {
        quadSocket = someClient;

        connectedStatus = true;
    
        println("Connected to quad at IP " + quadSocket.ip());

        read_from_socket(quadSocket);
    }
}

void disconnectEvent(Client someClient) 
{
    print("Disconnect event for client ", someClient.ip());
    if(quadSocket == someClient)
    {
        quadSocket = null;
        connectedStatus = true;
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
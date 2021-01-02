static final byte OQ2P_START = (byte)0xCA;
static final byte OQ2P_END = (byte)0xFE;

static final byte OQ2P_CLASS_CONTROL = (byte)0x02;

static final byte OQ2P_TYPE_COMMAND = (byte)0x00;
static final byte OQ2P_TYPE_REQUEST = (byte)0x01;
static final byte OQ2P_TYPE_RESPONSE = (byte)0x02;

// Message IDS
static final byte OQ2P_MID_ARM =        (byte)0x00;
static final byte OQ2P_MID_PITCH_SET =  (byte)0x01;
static final byte OQ2P_MID_ROLL_SET =   (byte)0x02;
static final byte OQ2P_MID_YAW_SET =    (byte)0x03;
static final byte OQ2P_MID_ELEVATION_SET =  (byte)0x04;
static final byte OQ2P_MID_SPEED_SET =  (byte)0x05;
static final byte OQ2P_MID_THRUST =     (byte)0x06;
static final byte OQ2P_MID_KEEP_ALIVE = (byte)0xAA;

// Motor indexes
static final byte OQ2P_MOTOR_INDEX_ALL  = (byte)0xFF;
static final byte OQ2P_MOTOR_INDEX_1  = (byte)0x01;
static final byte OQ2P_MOTOR_INDEX_2  = (byte)0x02;
static final byte OQ2P_MOTOR_INDEX_3  = (byte)0x03;
static final byte OQ2P_MOTOR_INDEX_4  = (byte)0x04;

// Motor arm or no
static final byte OQ2P_MOTOR_ARM        = (byte)0x01;
static final byte OQ2P_MOTOR_DISARM     = (byte)0x00;


byte[] oq2p_arm_message(byte index, byte arm) 
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

byte[] oq2p_arm_state_request()
{
    byte buffer[] = new byte[7];
    
    // Header
    buffer[0] = OQ2P_START;
    buffer[1] = OQ2P_CLASS_CONTROL;
    buffer[2] = OQ2P_MID_ARM;
    buffer[3] = OQ2P_TYPE_REQUEST;
    buffer[4] = (byte)0x00;    // Length MSB
    buffer[5] = (byte)0x00;    // Length LSB

    // Footer
    buffer[6] = OQ2P_END;

    return buffer;
}

byte[] oq2p_setpoint_command(byte index, int value) 
{
    byte buffer[] = new byte[9];

    buffer[0] = OQ2P_START;
    buffer[1] = OQ2P_CLASS_CONTROL;
    buffer[2] = index;
    buffer[3] = OQ2P_TYPE_COMMAND;
    buffer[4] = (byte)0x00;    // Length MSB
    buffer[5] = (byte)0x02;    // Length LSB

    // Data
    buffer[6] = (byte)((value & (int)0xFF00) >> 8);
    buffer[7] = (byte)((value & (int)0x00FF) >> 0);

    buffer[8] = OQ2P_END;

    return buffer;
}

byte[] oq2p_setpoint_request(byte index) 
{

    byte buffer[] = new byte[7];

    buffer[0] = OQ2P_START;
    buffer[1] = OQ2P_CLASS_CONTROL;
    buffer[2] = index;
    buffer[3] = OQ2P_TYPE_REQUEST;
    buffer[4] = (byte)0x00;    // Length MSB
    buffer[5] = (byte)0x00;    // Length LSB

    buffer[6] = OQ2P_END;

    return buffer;
}

byte[] oq2p_thrust_command(int percent) 
{
    byte buffer[] = new byte[8];

    buffer[0] = OQ2P_START;
    buffer[1] = OQ2P_CLASS_CONTROL;
    buffer[2] = OQ2P_MID_THRUST;
    buffer[3] = OQ2P_TYPE_COMMAND;
    buffer[4] = (byte)0x00;    // Length MSB
    buffer[5] = (byte)0x01;    // Length LSB

    buffer[6] = (byte)percent;

    buffer[7] = OQ2P_END;

    return buffer;
}

byte[] oq2p_thrust_request() 
{

    byte buffer[] = new byte[7];

    buffer[0] = OQ2P_START;
    buffer[1] = OQ2P_CLASS_CONTROL;
    buffer[2] = OQ2P_MID_THRUST;
    buffer[3] = OQ2P_TYPE_REQUEST;
    buffer[4] = (byte)0x00;    // Length MSB
    buffer[5] = (byte)0x00;    // Length LSB

    buffer[6] = OQ2P_END;

    return buffer;
}
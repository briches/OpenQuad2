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
public void oq2p_exhaustive_test(Client _socket, int inter_test_period)
{
    // Arm motors 1 by 1
    _socket.write( oq2p_arm_message(OQ2P_MOTOR_INDEX_1, OQ2P_MOTOR_ARM) );
    delay(inter_test_period);
    if(!read_from_socket(_socket))
        println("error no response");


    _socket.write( oq2p_arm_message(OQ2P_MOTOR_INDEX_2, OQ2P_MOTOR_ARM) );
    delay(inter_test_period);
    if(!read_from_socket(_socket))
        println("error no response");


    _socket.write( oq2p_arm_message(OQ2P_MOTOR_INDEX_3, OQ2P_MOTOR_ARM) );
    delay(inter_test_period);
    if(!read_from_socket(_socket))
        println("error no response");


    _socket.write( oq2p_arm_message(OQ2P_MOTOR_INDEX_4, OQ2P_MOTOR_ARM) );
    delay(inter_test_period);
    if(!read_from_socket(_socket))
        println("error no response");


    // Disarm motors 1 by 1
    _socket.write( oq2p_arm_message(OQ2P_MOTOR_INDEX_1, OQ2P_MOTOR_DISARM) );
    delay(inter_test_period);
    if(!read_from_socket(_socket))
        println("error no response");

    _socket.write( oq2p_arm_message(OQ2P_MOTOR_INDEX_2, OQ2P_MOTOR_DISARM) );
    delay(inter_test_period);
    if(!read_from_socket(_socket))
        println("error no response");

    _socket.write( oq2p_arm_message(OQ2P_MOTOR_INDEX_3, OQ2P_MOTOR_DISARM) );
    delay(inter_test_period);
    if(!read_from_socket(_socket))
        println("error no response");

    _socket.write( oq2p_arm_message(OQ2P_MOTOR_INDEX_4, OQ2P_MOTOR_DISARM) );
    delay(inter_test_period);
    if(!read_from_socket(_socket))
        println("error no response");

    // Arm all
    _socket.write( oq2p_arm_message(OQ2P_MOTOR_INDEX_ALL, OQ2P_MOTOR_ARM) );
    delay(inter_test_period);
    if(!read_from_socket(_socket))
        println("error no response");

    // Disarm all
    _socket.write( oq2p_arm_message(OQ2P_MOTOR_INDEX_ALL, OQ2P_MOTOR_DISARM) );
    delay(inter_test_period);
    if(!read_from_socket(_socket))
        println("error no response");

    // Query armed state
    _socket.write( oq2p_arm_state_request());
    delay(inter_test_period);
    if(!read_from_socket(_socket))
        println("error no response");

    // Test set pitch
    // Units are 100 * degrees
    _socket.write( oq2p_setpoint_command(OQ2P_MID_PITCH_SET, 256));
    delay(inter_test_period);
    if(!read_from_socket(_socket))
        println("error no response");

    // Test set roll
    // Units are 100 * degrees
    _socket.write( oq2p_setpoint_command(OQ2P_MID_ROLL_SET, 512));
    delay(inter_test_period);
    if(!read_from_socket(_socket))
        println("error no response");

    // Test set yaw
    // Units are 100 * degrees
    _socket.write( oq2p_setpoint_command(OQ2P_MID_YAW_SET, 768));
    delay(inter_test_period);
    if(!read_from_socket(_socket))
        println("error no response");

    // Test get yaw
    // Units are 100 * degrees
    _socket.write( oq2p_setpoint_request(OQ2P_MID_YAW_SET));
    delay(inter_test_period);
    if(!read_from_socket(_socket))
        println("error no response");

    // Test set roll
    // Units are 100 * degrees
    _socket.write( oq2p_setpoint_request(OQ2P_MID_ROLL_SET));
    delay(inter_test_period);
    if(!read_from_socket(_socket))
        println("error no response");

    // Test set pitch
    // Units are 100 * degrees
    _socket.write( oq2p_setpoint_request(OQ2P_MID_PITCH_SET));
    delay(inter_test_period);
    if(!read_from_socket(_socket))
        println("error no response");

    // Test get thrust base
    // Units are percent. 0 - 100
    _socket.write( oq2p_thrust_request());
    delay(inter_test_period);
    if(!read_from_socket(_socket))
        println("error no response");

    // Test set thrust base
    // Units are percent. 0 - 100
    _socket.write( oq2p_thrust_command(50));
    delay(inter_test_period);
    if(!read_from_socket(_socket))
        println("error no response");

    // Test get thrust base
    // Units are percent. 0 - 100
    _socket.write( oq2p_thrust_request());
    delay(inter_test_period);
    if(!read_from_socket(_socket))
        println("error no response");
}
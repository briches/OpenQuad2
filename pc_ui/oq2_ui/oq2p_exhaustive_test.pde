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
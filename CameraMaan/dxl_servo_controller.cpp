#include "dxl_servo_controller.h"
#include <exception.h>

void DxlController::clean_up()
{
    int dxl_comm_result = COMM_TX_FAIL; // Communication result
    uint8_t dxl_error = 0;              // Dynamixel error

    cout << "Servo ID: " << DXL_ID_PAN << " -- [Disabling Torque!]" << endl;

    // Disable Dynamixel Torque for Pan servo
    dxl_comm_result = packet_handler->write1ByteTxRx(port_handler, DXL_ID_PAN, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packet_handler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packet_handler->getRxPacketError(dxl_error));
    }

    cout << "Servo ID: " << DXL_ID_TILT << " -- [Disabling Torque!]" << endl;

    // Disable Dynamixel Torque for Tilt servo
    dxl_comm_result = packet_handler->write1ByteTxRx(port_handler, DXL_ID_TILT, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packet_handler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packet_handler->getRxPacketError(dxl_error));
    }

    // Close ports
    port_handler->closePort();
    return;
}

DxlController::DxlController()
{
    port_handler = dynamixel::PortHandler::getPortHandler(PORT_PATH);
    packet_handler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    int dxl_comm_result = COMM_TX_FAIL; // Communication result

    uint8_t dxl_error = 0; // Dynamixel error

    // Open port for BOTH servos
    if (port_handler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        throw std::runtime_error("Failed to open the port!");
    }

    // Set port baudrate for servos
    if (port_handler->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        throw std::runtime_error("Failed to change the baudrate");
    }

    // Enable Torque for PAN servo (port1)
    dxl_comm_result = packet_handler->write1ByteTxRx(port_handler, DXL_ID_PAN, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        throw std::runtime_error(packet_handler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        throw std::runtime_error(packet_handler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("PAN servo has been successfully connected \n");
    }

    // Enable Torque for TILT servo (port2)
    dxl_comm_result = packet_handler->write1ByteTxRx(port_handler, DXL_ID_PAN, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        throw std::runtime_error(packet_handler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        throw std::runtime_error(packet_handler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("Dynamixel has been successfully connected \n");
    }

    //Change PAN moving speed
    dxl_comm_result = packet_handler->write2ByteTxRx(port_handler, DXL_ID_PAN, ADDR_MX_MOVEMENT_SPEED, MOVE_SPEED, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        throw std::runtime_error(packet_handler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        throw std::runtime_error(packet_handler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("PAN Dynamixel speed has been changed \n");
    }

    // Change TILT moving speed
    dxl_comm_result = packet_handler->write2ByteTxRx(port_handler, DXL_ID_TILT, ADDR_MX_MOVEMENT_SPEED, MOVE_SPEED, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        throw std::runtime_error(packet_handler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        throw std::runtime_error(packet_handler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("TILT Dynamixel speed has been changed \n");
    }
}

DxlController::~DxlController()
{
    clean_up();
}

int DxlController::relative_PAN(int PAN_degrees)
{
    // Error checking variables
    uint8_t dxl_error = 0;              // Dynamixel error
    int dxl_comm_result = COMM_TX_FAIL; // Communication result

    // Get the current position of the servo
    int current_pos = getPosition(DXL_ID_PAN);

    // Convert from degrees to servo position unit
    int pos_diff = PAN_degrees / .29296875;

    int goal_position = current_pos - pos_diff;

    // Write goal position for PAN servo
    if (goal_position <= DXL_PAN_MAXIMUM_POSITION_VALUE && goal_position >= DXL_PAN_MINIMUM_POSITION_VALUE)
    {
        dxl_comm_result = packet_handler->write2ByteTxRx(port_handler, DXL_ID_PAN, ADDR_MX_GOAL_POSITION, goal_position, &dxl_error);
    }
    else
    {
        printf("Target position %i is out of bounds\n", goal_position);
        return -1;
    }

    if (dxl_comm_result != COMM_SUCCESS)
    {
        cout << "FAILED to write goal position for PAN servo. ID:" << DXL_ID_PAN << endl;
        printf("%s\n", packet_handler->getTxRxResult(dxl_comm_result));
        return -1;
    }
    else if (dxl_error != 0)
    {
        cout << "FAILED to write goal position for PAN servo. ID:" << DXL_ID_PAN << endl;
        printf("%s\n", packet_handler->getRxPacketError(dxl_error));
        return -1;
    }
    return goal_position;
}

int DxlController::relative_TILT(int TILT_degrees)
{
    // Error checking variables
    uint8_t dxl_error = 0;              // Dynamixel error
    int dxl_comm_result = COMM_TX_FAIL; // Communication result

    // Get the current position of the servo
    int current_pos = getPosition(DXL_ID_TILT);

    // Convert from degrees to servo position unit
    int pos_diff = TILT_degrees / .29296875;

    int goal_position = current_pos - pos_diff;

    // Write goal position for PAN servo
    if (goal_position <= DXL_TILT_MAXIMUM_POSITION_VALUE && goal_position >= DXL_TILT_MINIMUM_POSITION_VALUE)
    {
        dxl_comm_result = packet_handler->write2ByteTxRx(port_handler, DXL_ID_TILT, ADDR_MX_GOAL_POSITION, goal_position, &dxl_error);
    }
    else
    {
        printf("Target position %i is out of bounds\n", goal_position);
        return -1;
    }

    if (dxl_comm_result != COMM_SUCCESS)
    {
        cout << "FAILED to write goal position for PAN servo. ID:" << DXL_ID_TILT << endl;
        printf("%s\n", packet_handler->getTxRxResult(dxl_comm_result));
        return -1;
    }
    else if (dxl_error != 0)
    {
        cout << "FAILED to write goal position for PAN servo. ID:" << DXL_ID_TILT << endl;
        printf("%s\n", packet_handler->getRxPacketError(dxl_error));
        return -1;
    }
    return goal_position;
}

int DxlController::getPosition(int servo_id)
{
    // Error checking variables
    uint8_t dxl_error = 0;              // Dynamixel error
    int dxl_comm_result = COMM_TX_FAIL; // Communication result
    uint16_t dxl_present_position = 0;  // Present position

    // Read present position for PAN servo
    dxl_comm_result = packet_handler->read2ByteTxRx(port_handler, servo_id, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packet_handler->getTxRxResult(dxl_comm_result));
        return -1;
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packet_handler->getRxPacketError(dxl_error));
        return -1;
    }

    return int(dxl_present_position);
}

void DxlController::WAIT_for_goal(int servo_ID, int goal_position)
{
    if (goal_position < 0)
    {
        cout << "Relative pan or tilt failed!" << endl;
        return;
    }
    int current_position;
    do
    {
        current_position = getPosition(servo_ID);
        if (current_position > 0)
        {
            cout << "ID: " << servo_ID << " Current position: " << current_position << " Goal position: " << goal_position << endl;
        }
        else
        {
            cout << "Error in WAIT_for_goal. getPosition(int) failed" << endl;
        }

    } while ((abs(goal_position - current_position) > DXL_MOVING_STATUS_THRESHOLD));
}

bool DxlController::return_home()
{
    cout << "Returning home..." << endl;
    // Error checking variables
    uint8_t dxl_error = 0;              // Dynamixel error
    int dxl_comm_result = COMM_TX_FAIL; // Communication result

    dxl_comm_result = packet_handler->write2ByteTxRx(port_handler, DXL_ID_PAN, ADDR_MX_GOAL_POSITION, 511, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        cout << "FAILED to write goal position for PAN servo. ID:" << DXL_ID_PAN << endl;
        printf("%s\n", packet_handler->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        cout << "FAILED to write goal position for PAN servo. ID:" << DXL_ID_PAN << endl;
        printf("%s\n", packet_handler->getRxPacketError(dxl_error));
        return false;
    }

    WAIT_for_goal(DXL_ID_PAN, 511);

    dxl_comm_result = packet_handler->write2ByteTxRx(port_handler, DXL_ID_TILT, ADDR_MX_GOAL_POSITION, 511, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        cout << "FAILED to write goal position for TILT servo. ID:" << DXL_ID_TILT << endl;
        printf("%s\n", packet_handler->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        cout << "FAILED to write goal position for TILT servo. ID:" << DXL_ID_TILT << endl;
        printf("%s\n", packet_handler->getRxPacketError(dxl_error));
        return false;
    }

    WAIT_for_goal(DXL_ID_TILT, 511);

    return true;
}

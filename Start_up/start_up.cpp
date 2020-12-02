
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0

#include <stdlib.h>
#include <signal.h>
#include <string>
#include <iostream>

#include "dynamixel_sdk.h" // Uses Dynamixel SDK library

// Control table address
#define ADDR_MX_TORQUE_ENABLE 24 // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION 30
#define ADDR_MX_PRESENT_POSITION 36

// Protocol version
#define PROTOCOL_VERSION 1.0 // See which protocol version is used in the Dynamixel

// Dynamixel Settings
#define DXL_ID_PAN 5   // Dynamixel ID: 5
#define DXL_ID_TILT 10 // Dynamixel ID: 10
#define BAUDRATE 57600
#define PORT_PATH "/dev/ttyUSB0"

#define TORQUE_ENABLE 1  // Value for enabling the torque
#define TORQUE_DISABLE 0 // Value for disabling the torque
#define DXL_PAN_MINIMUM_POSITION_VALUE 0
#define DXL_PAN_MAXIMUM_POSITION_VALUE 1023
#define DXL_TILT_MINIMUM_POSITION_VALUE 200
#define DXL_TILT_MAXIMUM_POSITION_VALUE 720
#define DXL_MOVING_STATUS_THRESHOLD 3 // Dynamixel moving status threshold

#define ESC_ASCII_VALUE 0x1b

using namespace std;

// Initialize PortHandler instance
// Gives access and methods for handling port: /dev/ttyUSB0
// Both devices use /dev/ttyUSB0 so we only need 1 port handler
dynamixel::PortHandler *PORT_HANDLER = dynamixel::PortHandler::getPortHandler(PORT_PATH);

// Initialize PacketHandler instance
// Set the protocol version
// We are using Dynamixel AX-12's and they use PROTOCOL 1.0
dynamixel::PacketHandler *PACKET_HANDLER = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

// This is a signal handler that disables the servos and closes ports before exiting the program
void clean_up(int)
{
    int dxl_comm_result = COMM_TX_FAIL; // Communication result
    uint8_t dxl_error = 0;              // Dynamixel error

    cout << "Servo ID: " << DXL_ID_PAN << " -- [Disabling Torque!]" << endl;

    // Disable Dynamixel Torque for Pan servo
    dxl_comm_result = PACKET_HANDLER->write1ByteTxRx(PORT_HANDLER, DXL_ID_PAN, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", PACKET_HANDLER->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", PACKET_HANDLER->getRxPacketError(dxl_error));
    }

    cout << "Servo ID: " << DXL_ID_TILT << " -- [Disabling Torque!]" << endl;

    // Disable Dynamixel Torque for Tilt servo
    dxl_comm_result = PACKET_HANDLER->write1ByteTxRx(PORT_HANDLER, DXL_ID_TILT, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", PACKET_HANDLER->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", PACKET_HANDLER->getRxPacketError(dxl_error));
    }

    // Close ports
    PORT_HANDLER->closePort();
    exit(1);
}

// Disables the PAN and TILT servos and closes ports
int clean_up()
{
    int dxl_comm_result = COMM_TX_FAIL; // Communication result
    uint8_t dxl_error = 0;              // Dynamixel error

    cout << "Servo ID: " << DXL_ID_PAN << " -- [Disabling Torque!]" << endl;

    // Disable Dynamixel Torque for Pan servo
    dxl_comm_result = PACKET_HANDLER->write1ByteTxRx(PORT_HANDLER, DXL_ID_PAN, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", PACKET_HANDLER->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", PACKET_HANDLER->getRxPacketError(dxl_error));
    }

    cout << "Servo ID: " << DXL_ID_TILT << " -- [Disabling Torque!]" << endl;

    // Disable Dynamixel Torque for Tilt servo
    dxl_comm_result = PACKET_HANDLER->write1ByteTxRx(PORT_HANDLER, DXL_ID_TILT, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", PACKET_HANDLER->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", PACKET_HANDLER->getRxPacketError(dxl_error));
    }

    // Close ports
    PORT_HANDLER->closePort();
    return 0;
}

int getch()
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

// Returns the current position of a servo or -1 upon failure
//  positions will be from 0 to 1023
int getPosition(int servo_id)
{
    // Error checking variables
    uint8_t dxl_error = 0;              // Dynamixel error
    int dxl_comm_result = COMM_TX_FAIL; // Communication result
    uint16_t dxl_present_position = 0;  // Present position

    // Read present position for PAN servo
    dxl_comm_result = PACKET_HANDLER->read2ByteTxRx(PORT_HANDLER, servo_id, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", PACKET_HANDLER->getTxRxResult(dxl_comm_result));
        return -1;
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", PACKET_HANDLER->getRxPacketError(dxl_error));
        return -1;
    }

    return int(dxl_present_position);
}

/* +30 would rotate clockwise 30 degrees, while -30 will rotate counter-clockwise 30 degrees.
 *
 * @param an integer representing the desired change in orientation relative to the servos current position. 
 * @return returns goal position upon success, and -1 on failure
 */
int relative_PAN(int PAN_degrees)
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
        dxl_comm_result = PACKET_HANDLER->write2ByteTxRx(PORT_HANDLER, DXL_ID_PAN, ADDR_MX_GOAL_POSITION, goal_position, &dxl_error);
    }
    else
    {
        printf("Target position %i is out of bounds\n", goal_position);
        return -1;
    }

    if (dxl_comm_result != COMM_SUCCESS)
    {
        cout << "FAILED to write goal position for PAN servo. ID:" << DXL_ID_PAN << endl;
        printf("%s\n", PACKET_HANDLER->getTxRxResult(dxl_comm_result));
        return -1;
    }
    else if (dxl_error != 0)
    {
        cout << "FAILED to write goal position for PAN servo. ID:" << DXL_ID_PAN << endl;
        printf("%s\n", PACKET_HANDLER->getRxPacketError(dxl_error));
        return -1;
    }
    return goal_position;
}

/* +30 would rotate clockwise 30 degrees, while -30 will rotate counter-clockwise 30 degrees.
 *
 * @param an integer representing the desired change in orientation relative to the servos current position. 
 * @return returns goal position upon success, and -1 on failure
 */
int relative_TILT(int TILT_degrees)
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
        dxl_comm_result = PACKET_HANDLER->write2ByteTxRx(PORT_HANDLER, DXL_ID_TILT, ADDR_MX_GOAL_POSITION, goal_position, &dxl_error);
    }
    else
    {
        printf("Target position %i is out of bounds\n", goal_position);
        return -1;
    }

    if (dxl_comm_result != COMM_SUCCESS)
    {
        cout << "FAILED to write goal position for PAN servo. ID:" << DXL_ID_TILT << endl;
        printf("%s\n", PACKET_HANDLER->getTxRxResult(dxl_comm_result));
        return -1;
    }
    else if (dxl_error != 0)
    {
        cout << "FAILED to write goal position for PAN servo. ID:" << DXL_ID_TILT << endl;
        printf("%s\n", PACKET_HANDLER->getRxPacketError(dxl_error));
        return -1;
    }
    return goal_position;
}

void WAIT_for_goal(int servo_ID, int goal_position)
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

bool return_home()
{
    cout << "Returning home..." << endl;
    // Error checking variables
    uint8_t dxl_error = 0;              // Dynamixel error
    int dxl_comm_result = COMM_TX_FAIL; // Communication result

    dxl_comm_result = PACKET_HANDLER->write2ByteTxRx(PORT_HANDLER, DXL_ID_PAN, ADDR_MX_GOAL_POSITION, 511, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        cout << "FAILED to write goal position for PAN servo. ID:" << DXL_ID_PAN << endl;
        printf("%s\n", PACKET_HANDLER->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        cout << "FAILED to write goal position for PAN servo. ID:" << DXL_ID_PAN << endl;
        printf("%s\n", PACKET_HANDLER->getRxPacketError(dxl_error));
        return false;
    }

    WAIT_for_goal(DXL_ID_PAN, 511);

    dxl_comm_result = PACKET_HANDLER->write2ByteTxRx(PORT_HANDLER, DXL_ID_TILT, ADDR_MX_GOAL_POSITION, 511, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        cout << "FAILED to write goal position for TILT servo. ID:" << DXL_ID_TILT << endl;
        printf("%s\n", PACKET_HANDLER->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        cout << "FAILED to write goal position for TILT servo. ID:" << DXL_ID_TILT << endl;
        printf("%s\n", PACKET_HANDLER->getRxPacketError(dxl_error));
        return false;
    }
    
    WAIT_for_goal(DXL_ID_TILT, 511);

    return true;
}

int main()
{
    // We need a signal handler for SIGINT, SIGHUP, and SIGTERM that disables the servos.
    struct sigaction exit_action;
    exit_action.sa_handler = clean_up;
    sigemptyset(&exit_action.sa_mask);
    exit_action.sa_flags = 0;
    sigaction(SIGINT, &exit_action, nullptr);
    sigaction(SIGHUP, &exit_action, nullptr);
    sigaction(SIGTERM, &exit_action, nullptr);

    int dxl_comm_result = COMM_TX_FAIL; // Communication result

    uint8_t dxl_error = 0; // Dynamixel error

    // Open port for BOTH servos
    if (PORT_HANDLER->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    // Set port baudrate for PAN servo
    if (PORT_HANDLER->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    // Enable Torque for PAN servo (port1)
    dxl_comm_result = PACKET_HANDLER->write1ByteTxRx(PORT_HANDLER, DXL_ID_PAN, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", PACKET_HANDLER->getTxRxResult(dxl_comm_result));
        return clean_up();
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", PACKET_HANDLER->getRxPacketError(dxl_error));
        return clean_up();
    }
    else
    {
        printf("PAN servo has been successfully connected \n");
    }

    // Enable Torque for TILT servo (port2)
    dxl_comm_result = PACKET_HANDLER->write1ByteTxRx(PORT_HANDLER, DXL_ID_PAN, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", PACKET_HANDLER->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", PACKET_HANDLER->getRxPacketError(dxl_error));
    }
    else
    {
        printf("Dynamixel has been successfully connected \n");
    }

    if (!return_home())
    {
        return clean_up();
    }

    WAIT_for_goal(DXL_ID_PAN, relative_PAN(-90));
    WAIT_for_goal(DXL_ID_TILT, relative_TILT(30));
    WAIT_for_goal(DXL_ID_TILT, relative_TILT(-60));

    return_home();

    WAIT_for_goal(DXL_ID_PAN, relative_PAN(90));
    WAIT_for_goal(DXL_ID_TILT, relative_TILT(30));
    WAIT_for_goal(DXL_ID_TILT, relative_TILT(-60));

    return_home();

    return clean_up();
}

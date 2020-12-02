
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0

#include <stdlib.h>
#include <stdio.h>
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

#define TORQUE_ENABLE 1                // Value for enabling the torque
#define TORQUE_DISABLE 0               // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE 300 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE 511 // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD 3  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE 0x1b

// Initialize PortHandler instance
// Gives access and methods for handling port: /dev/ttyUSB0
// Both devices use /dev/ttyUSB0 so we only need 1 port handler
dynamixel::PortHandler *PORT_HANDLER = dynamixel::PortHandler::getPortHandler(PORT_PATH);

// Initialize PacketHandler instance
// Set the protocol version
// We are using Dynamixel AX-12's and they use PROTOCOL 1.0
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

// This is a signal handler that disables the servos and closes ports before exiting the program
void clean_up(int)
{
    int dxl_comm_result = COMM_TX_FAIL; // Communication result
    uint8_t dxl_error = 0;              // Dynamixel error

    printf("Servo ID: %s -- [Disabling Torque!]\n", DXL_ID_PAN);

    // Disable Dynamixel Torque for Pan servo
    dxl_comm_result = packetHandler->write1ByteTxRx(PORT_HANDLER, DXL_ID_PAN, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    printf("Servo ID: %s -- [Disabling Torque!]\n", DXL_ID_TILT);

    // Disable Dynamixel Torque for Tilt servo
    dxl_comm_result = packetHandler->write1ByteTxRx(PORT_HANDLER, DXL_ID_TILT, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
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

    printf("Servo ID: %s -- [Disabling Torque!]\n", DXL_ID_PAN);

    // Disable Dynamixel Torque for Pan servo
    dxl_comm_result = packetHandler->write1ByteTxRx(PORT_HANDLER, DXL_ID_PAN, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    printf("Servo ID: %s -- [Disabling Torque!]\n", DXL_ID_TILT);

    // Disable Dynamixel Torque for Tilt servo
    dxl_comm_result = packetHandler->write1ByteTxRx(PORT_HANDLER, DXL_ID_TILT, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
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



/* +30 would rotate clockwise 30 degrees, while -30 will rotate counter-clockwise 30 degrees.
 *
 * @param an integer representing the desired change in orientation relative to the servos current position. 
 * @return true on success, fail otherwise
 */
bool relative_PAN(int degrees)
{
    // Error checking variables
    uint8_t dxl_error = 0;                                                               // Dynamixel error
    int dxl_comm_result = COMM_TX_FAIL;                                                  // Communication result

    //Function converts relative degrees to a servo position


    int goal_position;

    

    // Write goal position for PAN servo
    dxl_comm_result = packetHandler->write2ByteTxRx(PORT_HANDLER, DXL_ID_PAN, ADDR_MX_GOAL_POSITION, goal_position, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("FAILED to write goal position for PAN servo. ID:%s\nERROR:\n", DXL_ID_PAN);
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        printf("FAILED to write goal position for PAN servo. ID:%s\nERROR:\n", DXL_ID_PAN);
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool relative_TILT(int degrees)
{

    return false;
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

    int index = 0;
    int dxl_comm_result = COMM_TX_FAIL;                                                  // Communication result
    int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE}; // Goal position

    uint8_t dxl_error = 0;             // Dynamixel error
    uint16_t dxl_present_position = 0; // Present position

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
    dxl_comm_result = packetHandler->write1ByteTxRx(PORT_HANDLER, DXL_ID_PAN, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return clean_up();
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        return clean_up();
    }
    else
    {
        printf("PAN servo has been successfully connected \n");
    }

    // Enable Torque for TILT servo (port2)
    dxl_comm_result = packetHandler->write1ByteTxRx(PORT_HANDLER, DXL_ID_PAN, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("Dynamixel has been successfully connected \n");
    }

    while (1)
    {
        printf("Press any key to continue! (or press ESC to quit!)\n");
        if (getch() == ESC_ASCII_VALUE)
            break;

        // Write goal position for PAN servo
        dxl_comm_result = packetHandler->write2ByteTxRx(PORT_HANDLER, DXL_ID_PAN, ADDR_MX_GOAL_POSITION, dxl_goal_position[index], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("FAILED to write goal position for PAN servo. ID:%s\nERROR:\n", DXL_ID_PAN);
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            // Exit program gracefully
            return clean_up();
        }
        else if (dxl_error != 0)
        {
            printf("FAILED to write goal position for PAN servo. ID:%s\nERROR:\n", DXL_ID_PAN);
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            // Exit program gracefully
            return clean_up();
        }

        // Print present position for PAN servo while the servo is rotating towards its target position
        do
        {
            // Read present position for PAN servo
            dxl_comm_result = packetHandler->read2ByteTxRx(PORT_HANDLER, DXL_ID_PAN, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
                break;
            }
            else if (dxl_error != 0)
            {
                printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            }

            printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID_PAN, dxl_goal_position[index], dxl_present_position);

            // While not at target position
        } while ((abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

        // Write goal position for Tilt servo
        dxl_comm_result = packetHandler->write2ByteTxRx(PORT_HANDLER, DXL_ID_TILT, ADDR_MX_GOAL_POSITION, dxl_goal_position[index], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            break;
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }

        // Print present position for TILT servo while the servo is rotating towards its target position
        do
        {
            // Read present position for TILT servo
            dxl_comm_result = packetHandler->read2ByteTxRx(PORT_HANDLER, DXL_ID_TILT, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
                break;
            }
            else if (dxl_error != 0)
            {
                printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            }

            printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID_TILT, dxl_goal_position[index], dxl_present_position);

        } while ((abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

        // Change goal position
        if (index == 0)
        {
            index = 1;
        }
        else
        {
            index = 0;
        }
    }

    return clean_up();
}

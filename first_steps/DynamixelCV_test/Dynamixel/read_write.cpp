
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0

#include <stdlib.h>
#include <stdio.h>
#include <signal.h>

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
#define DEVICENAME1 "/dev/ttyUSB0"
#define DEVICENAME2 "/dev/ttyUSB0"

#define TORQUE_ENABLE 1                // Value for enabling the torque
#define TORQUE_DISABLE 0               // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE 300 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE 511 // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD 3 // Dynamixel moving status threshold

#define ESC_ASCII_VALUE 0x1b

// Initialize PortHandler instance
// Set the port path
// Get methods and members of PortHandlerLinux or PortHandlerWindows
dynamixel::PortHandler *portHandler1 = dynamixel::PortHandler::getPortHandler(DEVICENAME1);
dynamixel::PortHandler *portHandler2 = dynamixel::PortHandler::getPortHandler(DEVICENAME2);

// Initialize PacketHandler instance
// Set the protocol version
// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

void clean_up(int)
{
    printf("Disabling Torque\n");
    int dxl_comm_result = COMM_TX_FAIL; // Communication result
    uint8_t dxl_error = 0;              // Dynamixel error

    // Disable Dynamixel Torque for Pan servo
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler1, DXL_ID_PAN, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    // Disable Dynamixel Torque for Tilt servo
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, DXL_ID_TILT, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    // Close ports
    portHandler1->closePort();
    portHandler2->closePort();
    exit(1);
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

int main()
{
    struct sigaction exit_action;
    exit_action.sa_handler = clean_up;
    sigemptyset(&exit_action.sa_mask);
    exit_action.sa_flags = 0;
    sigaction(SIGINT, &exit_action, nullptr);
    sigaction(SIGHUP, &exit_action, nullptr);
    sigaction(SIGTERM, &exit_action, nullptr);

    int index = 1;
    int dxl_comm_result = COMM_TX_FAIL;                                                  // Communication result
    int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE}; // Goal position

    uint8_t dxl_error = 0;             // Dynamixel error
    uint16_t dxl_present_position = 0; // Present position

    // Open port for pan servo
    if (portHandler1->openPort())
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

    // Open port for tilt servo
    if (portHandler2->openPort())
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

    // Set port1 baudrate for pan servo
    if (portHandler1->setBaudRate(BAUDRATE))
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

    // Set port2 baudrate for tilt servo
    if (portHandler2->setBaudRate(BAUDRATE))
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

    // Enable Torque for pan servo (port1)
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler1, DXL_ID_PAN, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
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

    // Enable Torque for tilt servo (port2)
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, DXL_ID_PAN, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
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

        // Write goal position for pan servo
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler1, DXL_ID_PAN, ADDR_MX_GOAL_POSITION, dxl_goal_position[index], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            break;
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }

        do
        {
            // Read present position for pan servo
            dxl_comm_result = packetHandler->read2ByteTxRx(portHandler1, DXL_ID_PAN, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
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

        } while ((abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

        // Write goal position for Tilt servo
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler2, DXL_ID_TILT, ADDR_MX_GOAL_POSITION, dxl_goal_position[index], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            break;
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }

        do
        {
            // Read present position for tilt servo
            dxl_comm_result = packetHandler->read2ByteTxRx(portHandler2, DXL_ID_TILT, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
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

    // Disable Dynamixel Torque for Pan servo
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler1, DXL_ID_PAN, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    // Disable Dynamixel Torque for Tilt servo
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, DXL_ID_TILT, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    // Close ports
    portHandler1->closePort();
    portHandler2->closePort();
    return 0;
}

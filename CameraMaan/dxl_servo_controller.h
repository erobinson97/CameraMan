/* AUTHOR: Ethan Robinson 
 * Created on December 14th, 2020
 * 
 * This is a class that implements my general purpose functions implemented
 * from creating start_up.cpp.
 *
 */
#ifndef DXL_SERVO_CONTROLLER_H
#define DXL_SERVO_CONTROLLER_H
#include "dynamixel_sdk.h"
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#include <stdlib.h>
#include <signal.h>
#include <string>
#include <iostream>
#include <exception>

// Control table address
#define ADDR_MX_TORQUE_ENABLE 24 // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION 30
#define ADDR_MX_PRESENT_POSITION 36
#define ADDR_MX_MOVEMENT_SPEED 32

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

#define DXL_MOVING_STATUS_THRESHOLD 30 // Dynamixel moving status threshold

#define MOVE_SPEED 30 //0 to 1023

#define ESC_ASCII_VALUE 0x1b

using namespace std;
class DxlController
{
private:
    // Initialize PortHandler instance
    // Gives access and methods for handling port: /dev/ttyUSB0
    //Both devices use /dev/ttyUSB0 so we only need 1 port handler
    dynamixel::PortHandler *port_handler;

    // Initialize PacketHandler instance
    // Set the protocol version
    // We are using Dynamixel AX-12's and they use PROTOCOL 1.0
    dynamixel::PacketHandler *packet_handler;



public:
    DxlController();
    ~DxlController();
    void clean_up(); // Disables servo torque and closes ports.

    /*
     * Getter for the current position of a servo; define by its ID number.
     * 
     * @param The servo ID.
     * @return The current position of the servo (0 to 1023), or -1 upon failure.
     */
    int getPosition(int servo_id);

    /* +30 would rotate clockwise 30 degrees, while -30 will rotate counter-clockwise 30 degrees.
     *
     * @param an integer representing the desired change in orientation relative to the servos current position. 
     * @return returns goal position upon success, and -1 on failure
     */
    int relative_PAN(int PAN_degrees);

    int relative_TILT(int TILT_degrees);

    void WAIT_for_goal(int servo_ID, int goal_position);

    bool return_home();
};

#endif

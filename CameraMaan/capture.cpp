#include <pthread.h>
#include <iostream>
#include <cstdlib>
#include <mqueue.h>
#include <fcntl.h>

//Dynamixel includes
#include "dynamixel_sdk.h"

#include "dxl_servo_controller.h"

//OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

using namespace std;
using namespace cv;

//Define message queue attributes
#define CAPTURE_QUEUE_NAME "/capture_queue"
#define SERVO_QUEUE_NAME "/servo_queue"

// Servo controller thread
void *ControllServos(void *threadid)
{
    DxlController controller;
    try
    {
        controller = DxlController();
    }
    catch (std::exception e)
    {
        controller.clean_up();

        cerr << "Failed to create DxlController object." << endl;
        cerr << "ErrOut: " << e.what() << "." << endl;
        cout << "Exiting thread" << endl;
        pthread_exit(NULL);
    }

    controller.WAIT_for_goal(DXL_ID_PAN, controller.relative_PAN(-90));
    controller.WAIT_for_goal(DXL_ID_TILT, controller.relative_TILT(30));
    controller.WAIT_for_goal(DXL_ID_TILT, controller.relative_TILT(-60));

    controller.return_home();

    controller.WAIT_for_goal(DXL_ID_PAN, controller.relative_PAN(90));
    controller.WAIT_for_goal(DXL_ID_TILT, controller.relative_TILT(30));
    controller.WAIT_for_goal(DXL_ID_TILT, controller.relative_TILT(-60));

    controller.return_home();

    pthread_exit(NULL);
}

// Tracking thread
void *Track(void *threadid)
{
    //Just run imshow for now

    mqd_t mq = mq_open(CAPTURE_QUEUE_NAME, O_RDONLY);
    if (mq < 0)
    {
        fprintf(stderr, "[CAPTURE]: Error, cannot open the queue: %s.\n", strerror(errno));
        pthread_exit(NULL);
    }

    char buff[sizeof(Mat) + 1];
    memset(buff, 0x00, sizeof(buff));
    unsigned int prio;
    ssize_t bytes_read;

    do
    {
        bytes_read = mq_receive(mq, buff, sizeof(Mat), &prio);

        if (bytes_read == sizeof(Mat))
        {
            Mat frame = (Mat)buff;
            imshow("TrackerFrames", frame);
        }
    }while(bytes_read >= 0);

    pthread_exit(NULL);
}

// Capture thread
void *Capture(void *threadid)
{
    VideoCapture capture(0);
    if (!capture.isOpened())
    {
        cerr << "Error opening video!" << endl;
        pthread_exit(NULL);
    }

    struct mq_attr attr;
    attr.mq_flags = 0;
    attr.mq_maxmsg = 16;
    attr.mq_msgsize = sizeof(Mat);
    attr.mq_curmsgs = 0;

    unsigned int prio = 0;

    mqd_t mq = mq_open(CAPTURE_QUEUE_NAME, O_WRONLY | O_CREAT, ((int)(0644)), &attr);
    if (mq < 0)
    {
        fprintf(stderr, "[CAPTURE]: Error, cannot open the queue: %s.\n", strerror(errno));
        pthread_exit(NULL);
    }

    char buff[sizeof(Mat)];
    Mat frame;
    capture >> frame;
    buff = &frame;
    mq_send(mq, buff, sizeof(Mat), prio);


    pthread_exit(NULL);
}

int main(int argc, char **argv[])
{
    // Setup signal handlers

    // Set up message queue between capture and tracking threads

    // Set up threads
    int errorCheck;
    int tempID = 0; // Temporary variable for thread id
    pthread_t thread_Controller, thread_Tracker, thread_Capture;

    // Create controller thread
    cout << "Creating controller thread" << endl;
    errorCheck = pthread_create(&thread_Controller, NULL, ControllServos, (void *)tempID);
    if (errorCheck)
    {
        cerr << "Unable to create thread[" << tempID << "], " << errorCheck << endl;
        cout << "Exiting..." << endl;
        exit(-1);
    }
    tempID++;

    // Create tracker thread
    cout << "Creating tracker thread" << endl;
    errorCheck = pthread_create(&thread_Tracker, NULL, Track, (void *)tempID);
    if (errorCheck)
    {
        cerr << "Unable to create thread[" << tempID << "], " << errorCheck << endl;
        cout << "Exiting..." << endl;
        exit(-1);
    }
    tempID++;

    // Create capture thread
    cout << "Creating capture thread" << endl;
    errorCheck = pthread_create(&thread_Capture, NULL, Capture, (void *)tempID);
    if (errorCheck)
    {
        cerr << "Unable to create thread[" << tempID << "], " << errorCheck << endl;
        cout << "Exiting..." << endl;
        exit(-1);
    }

    return 0;
}
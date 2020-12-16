#include <pthread.h>
#include <iostream>
#include <cstdlib>
#include <mqueue.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

//Dynamixel includes
#include "dynamixel_sdk.h"

#include "dxl_servo_controller.h"

//OpenCV includes
#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/tracking/kalman_filters.hpp>
#include <opencv2/tracking/tracking.hpp>
#include <opencv2/objdetect/objdetect.hpp>

using namespace std;
using namespace cv;

//Define message queue attributes
#define CAPTURE_QUEUE_NAME "/capture_queue"
#define SERVO_QUEUE_NAME "/servo_queue"

bool CAPTURE_RUNNING = false;
bool TRACKER_RUNNING = false;

// Servo controller thread
void *ControllServos(void *threadid)
{
    optional<DxlController> controller;
    try
    {
        controller.emplace();
    }
    catch (std::exception e)
    {
        cerr << "Failed to create DxlController object." << endl;
        cerr << "ErrOut: " << e.what() << "." << endl;
        cout << "Exiting DxlController thread" << endl;

        controller->clean_up();
        pthread_exit(NULL);
    }

    controller->WAIT_for_goal(DXL_ID_PAN, controller->relative_PAN(-90));
    controller->WAIT_for_goal(DXL_ID_TILT, controller->relative_TILT(30));
    controller->WAIT_for_goal(DXL_ID_TILT, controller->relative_TILT(-60));

    controller->return_home();

    controller->WAIT_for_goal(DXL_ID_PAN, controller->relative_PAN(90));
    controller->WAIT_for_goal(DXL_ID_TILT, controller->relative_TILT(30));
    controller->WAIT_for_goal(DXL_ID_TILT, controller->relative_TILT(-60));

    controller->return_home();

    printf("Exiting DxlController thread\n");
    pthread_exit(NULL);
}

// Tracking thread
void *Track(void *threadid)
{
    TRACKER_RUNNING = true;
    Ptr<Tracker> tracker = TrackerCSRT::create();
    bool object_defined = false;
    Rect2d obj_position;
    bool tracking = false;

    //Open the message queue
    mqd_t mq;
    do
    {
        mq = mq_open(CAPTURE_QUEUE_NAME, O_RDONLY);
        if (mq < 0)
        {
            fprintf(stderr, "[TRACKER]: Error, cannot open the queue: %s.\n", strerror(errno));
            sleep(1);
        }
    } while (mq == -1 || CAPTURE_RUNNING == false);

    printf("[TRACKER]: capture_queue opened\n");
    Mat *frame;
    ssize_t bytes_read;

    do
    {
        bytes_read = mq_receive(mq, (char *)&frame, sizeof(Mat *), NULL);

        printf("[TRACKER]: Recieved %d bytes.\n", bytes_read);

        if (bytes_read == sizeof(Mat *) && !(frame->empty()))
        {
            if (!object_defined)
            {
                imshow("CaptureFrames", *frame);
                if (waitKey(20) != -1)
                {
                    tracker->init(*frame, selectROI("CaptureFrames", *frame, true, false));
                    object_defined = true;
                }
            }
            else
            {
                tracking = tracker->update(*frame, obj_position);
                if (!tracking)
                {
                    putText(*frame, "Tracking failure detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);
                }
                cout << "Object x = " << obj_position.x << " y = " << obj_position.y << endl;

                //Draw rectangle on tracked object
                rectangle(*frame, obj_position, Scalar(255, 0, 0), 2, 1);

                imshow("CaptureFrames", *frame);
                waitKey(20);
            }
        }
        delete frame;

    } while (CAPTURE_RUNNING);

    //Delete message queue
    if (!CAPTURE_RUNNING)
    {
        mq_close(mq);
        mq_unlink(CAPTURE_QUEUE_NAME);
    }
    TRACKER_RUNNING = false;

    printf("Exiting tracker thread\n");
    pthread_exit(NULL);
}

// Capture thread
void *Capture(void *threadid)
{
    CAPTURE_RUNNING = true;
    VideoCapture capture(0);
    if (!capture.isOpened())
    {
        cerr << "Error opening video!" << endl;
        pthread_exit(NULL);
    }

    struct mq_attr attr;
    attr.mq_flags = 0;
    attr.mq_maxmsg = 8;
    attr.mq_msgsize = sizeof(Mat *);
    attr.mq_curmsgs = 0;

    unsigned int prio = 0;

    mq_unlink(CAPTURE_QUEUE_NAME);
    mqd_t mq = mq_open(CAPTURE_QUEUE_NAME, (O_WRONLY | O_CREAT), (S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH), &attr);
    if (mq < 0)
    {
        fprintf(stderr, "[CAPTURE]: Error, cannot open the queue: %s.\n", strerror(errno));
        printf("Exiting capture thread");
        pthread_exit(NULL);
    }

    printf("[CAPTURE]: Created message queue!\n");

    Mat frame;
    Mat *heap_frame;

    //Send rames while capture is
    while (capture.isOpened())
    {
        capture >> frame;
        resize(frame, frame, Size(1024, 800));
        heap_frame = new Mat(frame);
        mq_send(mq, (const char *)&heap_frame, sizeof(Mat *), prio);
    }

    if (!TRACKER_RUNNING)
    {
        mq_close(mq);
        mq_unlink(CAPTURE_QUEUE_NAME);
    }
    CAPTURE_RUNNING = false;
    printf("Exiting capture thread");
    pthread_exit(NULL);
}

int main(int argc, char *argv[])
{

    namedWindow("CaptureFrames", WINDOW_AUTOSIZE);

    // Set up threads
    int errorCheck;
    int tempID = 0; // Temporary variable for thread id
    pthread_t thread_Controller, thread_Tracker, thread_Capture;

    /*
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
*/

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

    pthread_join(thread_Capture, nullptr);
    pthread_join(thread_Tracker, nullptr);
    //pthread_join(thread_Controller, nullptr);
    return 0;
}

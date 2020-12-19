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
bool CONTROLLER_RUNNING = false;

// Servo controller thread
void *ControllServos(void *threadid)
{
    optional<DxlController> controller;
    CONTROLLER_RUNNING = true;
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
    Point *position;

    controller->return_home();

    mqd_t mq;
    do
    {
        mq = mq_open(SERVO_QUEUE_NAME, O_RDONLY);
        if (mq < 0)
        {
            fprintf(stderr, "[CONTROLLER]: Error, cannot open the capture queue: %s.\n", strerror(errno));
            sleep(1);
        }

    } while (mq == -1 || !TRACKER_RUNNING);
    printf("[CONTROLLER]: servo queue opened\n");
    ssize_t bytes_read;
    printf("[CONTROLLER]: Waiting for instructions...\n");
    int panDegrees;
    int tiltDegrees;
    do
    {
        bytes_read = mq_receive(mq, (char *)&position, sizeof(Point *), NULL);
        if (bytes_read == sizeof(Point *))
        {
            cout << "[CONTROLLER]: x = " << position->x << " y = " << position->y << endl;

            //Pan
            if (position->x > 640)
            {
                panDegrees = (position->x - 640) / 32.5;
                panDegrees = panDegrees / 2;
                cout << "position:x = " << position->x << ". panDegrees = " << panDegrees << endl;
                controller->WAIT_for_goal(DXL_ID_PAN, controller->relative_PAN(panDegrees));
            }
            else
            {
                panDegrees = (640 - position->x) / 32.5;
                panDegrees = panDegrees / 2;
                cout << "position:x = " << position->x << ". panDegrees = " << -panDegrees << endl;
                controller->WAIT_for_goal(DXL_ID_PAN, controller->relative_PAN(-panDegrees));
            }

            //Tilt
            if (position->y > 360)
            {
                tiltDegrees = (360 - position->y) / 20;
                tiltDegrees = tiltDegrees / 3;
                cout << "position:y = " << position->y << ". tiltDegrees = " << tiltDegrees << endl;
                controller->WAIT_for_goal(DXL_ID_TILT, controller->relative_TILT(tiltDegrees));
            }
            else
            {
                tiltDegrees = (position->y - 360) / 20;
                tiltDegrees = tiltDegrees / 3;
                cout << "position:y = " << position->y << ". tiltDegrees = " << -tiltDegrees << endl;
                controller->WAIT_for_goal(DXL_ID_TILT, controller->relative_TILT(-tiltDegrees));
            }
        }

        delete position;
    } while (TRACKER_RUNNING);

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
    Rect2d prev_position;
    Point *position;
    bool tracking = false;

    // Create message queue between tracker and controller
    struct mq_attr attr;
    attr.mq_flags = 0;
    attr.mq_maxmsg = 8;
    attr.mq_msgsize = sizeof(Point *);
    attr.mq_curmsgs = 0;

    mqd_t mq_controller;
    do
    {
        mq_controller = mq_open(SERVO_QUEUE_NAME, (O_WRONLY | O_CREAT), (S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH), &attr);
        if (mq_controller < 0)
        {
            fprintf(stderr, "[TRACKER]: Error, cannot open the controller queue: %s.\n", strerror(errno));
            sleep(1);
        }
    } while (mq_controller == -1 || !CONTROLLER_RUNNING);
    printf("[TRACKER]: servo queue opened\n");

    //Open the message queue between tracker and capture threads
    mqd_t mq;
    do
    {
        mq = mq_open(CAPTURE_QUEUE_NAME, O_RDONLY);
        if (mq < 0)
        {
            fprintf(stderr, "[TRACKER]: Error, cannot open the capture queue: %s.\n", strerror(errno));
            sleep(1);
        }
    } while (mq == -1 || CAPTURE_RUNNING == false);

    printf("[TRACKER]: capture_queue opened\n");
    Mat *frame;
    ssize_t bytes_read;

    do
    {
        bytes_read = mq_receive(mq, (char *)&frame, sizeof(Mat *), NULL);

        if (bytes_read == sizeof(Mat *) && !(frame->empty()))
        {
            if (!object_defined)
            {
                imshow("CaptureFrames", *frame);
                if (waitKey(20) != -1)
                {
                    tracker->init(*frame, selectROI("CaptureFrames", *frame, true, false));
                    object_defined = true;
                    destroyAllWindows();
                }
            }
            else
            {
                tracking = tracker->update(*frame, obj_position);
                if (!tracking)
                {
                    putText(*frame, "Tracking failure detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);
                    cout << "Tracking failure" << endl;
                }
                position = new Point(obj_position.x, obj_position.y);
                //rectangle(*frame, obj_position, Scalar(255, 0, 0), 2, 1);
                //imshow("CaptureFrames", *frame);
                //waitKey(10);

                //Send obj_position.x and obj_position.y to DxlController thread

                //Don't send if position isn't very different
                if (abs(obj_position.x - prev_position.x) > 10 || abs(obj_position.y - prev_position.y) > 10 || abs(obj_position.x - 640) > 10 || abs(obj_position.y - 360) > 10)
                {
                    mq_send(mq_controller, (const char *)&position, sizeof(Point *), NULL);
                }

                prev_position = obj_position;
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
    capture.set(cv::CAP_PROP_EXPOSURE, 4);
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
        resize(frame, frame, Size(1280, 720));
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

    pthread_join(thread_Capture, nullptr);
    pthread_join(thread_Tracker, nullptr);
    pthread_join(thread_Controller, nullptr);
    return 0;
}

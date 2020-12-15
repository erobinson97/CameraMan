#include <opencv2/dnn.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/tracking/kalman_filters.hpp>
#include <opencv2/tracking/tracking.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <iostream>

using namespace cv;
using namespace cv::ml;
using namespace std;

Ptr<Tracker> tracker;

struct initRoi
{
    //initial coordination based on EVENT_LBUTTONDOWN
    int initX;
    int initY;

    // actual coordination
    int actualX;
    int actualY;

    // final coordinates
    int finalX;
    int finalY;
    int finalWidth;
    int finalHeight;

    int displayRoi;
    int trackerReady;
    int initTracker;
} SelectedRoi;

static void CallBackF(int event, int x, int y, int flags, void *img)
{

    Mat &imge = *((Mat *)img);

    if (event == EVENT_RBUTTONDOWN)
    {
        cout << "right button " << endl;
        return;
    }

    if (event == EVENT_LBUTTONDOWN)
    {
        SelectedRoi.initX = x;
        SelectedRoi.initY = y;
        SelectedRoi.displayRoi = 1;
        cout << "left button DOWN" << endl;
        return;
    }

    if (event == EVENT_LBUTTONUP)
    {
        SelectedRoi.finalWidth = x - SelectedRoi.initX;
        SelectedRoi.finalHeight = y - SelectedRoi.initY;
        SelectedRoi.initTracker = 1;
        cout << "left button UP" << endl;
        return;
    }
    if (event == EVENT_MOUSEMOVE)
    {
        cout << "event mouse move" << endl;
        SelectedRoi.actualX = x;
        SelectedRoi.actualY = y;
        return;
    }
}

int main()
{

    //VideoCapture cap("tr.mov");
    VideoCapture cap(0);
    SelectedRoi.displayRoi = 0;
    SelectedRoi.trackerReady = 0;
    SelectedRoi.initTracker = 0;

    tracker = TrackerCSRT::create();
    // write output to file
    VideoWriter outputVideo;
    outputVideo.open("video.vmw", VideoWriter::fourcc('W', 'M', 'V', '2'),
                     cap.get(CAP_PROP_FPS), Size(1024, 800), true);

    for (;;)
    {
        if (!cap.isOpened())
        {
            cout << "Video Capture Fail" << endl;
            break;
        }

        else
        {
            Mat img;
            cap >> img;
            resize(img, img, Size(1024, 800));
            namedWindow("Video", WINDOW_AUTOSIZE);
            setMouseCallback("Video", CallBackF, 0);

            if (SelectedRoi.displayRoi != 0)
            {
                rectangle(img, Rect(SelectedRoi.initX, SelectedRoi.initY, SelectedRoi.actualX - SelectedRoi.initX, SelectedRoi.actualY - SelectedRoi.initY), Scalar(255, 255, 255), 4, 8, 0);
            }

            if (SelectedRoi.initTracker == 1)
            {
                tracker->init(img, Rect2d(SelectedRoi.initX, SelectedRoi.initY,
                                          SelectedRoi.finalWidth, SelectedRoi.finalHeight));
                SelectedRoi.trackerReady = 1;
            }
            if (SelectedRoi.trackerReady == 1)
            {
                Rect2d track;
                tracker->update(img, track);
                rectangle(img, track, Scalar(0, 0, 255), 4, 8, 0);
                Mat roi = img(track);
                if (roi.cols > 0)
                {
                    resize(roi, roi, Size(320, 460));
                    roi.copyTo(img(Rect(1, 1, 320, 460)));
                }
            }
            outputVideo << img;
            imshow("Video", img);
            int key2 = waitKey(20);
        }

    }
    return 0;
}
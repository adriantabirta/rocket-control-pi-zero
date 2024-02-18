#include <fstream>

#include <opencv2/opencv.hpp>
#include "opencv2/objdetect.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/core/types.hpp>

using namespace std;
using namespace cv;

#define FRAME_WIDTH 640 // 320
#define FRAME_HEIGHT 640 // 320

CascadeClassifier human_cascade = CascadeClassifier("haarcascade_fullbody.xml");

void loopLocallyUsingHaarAlgo(String path = "images/*.jpg") {
	Mat colorImage;
	Mat frame = Mat(FRAME_WIDTH, FRAME_HEIGHT, CV_8UC1);

	vector<String> fn;
        glob(path,fn,true); // recurse

 	std::vector<Rect> humans;

	// kill all proceses that prev used to display an image 
	system("sudo kill $(pgrep fbi)");

	for(size_t k=0; k<fn.size(); ++k) {
		frame = imread(fn[k], IMREAD_GRAYSCALE);

		if (frame.empty()) {
                        continue; //only proceed if sucsessful
                }

		// detect human
		human_cascade.detectMultiScale(frame, humans, 1.3, 1, 0, Size(20,20), Size());

		// convert back to color image
		cvtColor(frame, colorImage, cv::COLOR_GRAY2BGR);

		// add rectangles over humans
                for ( size_t i = 0; i < humans.size(); i++ ) {
                   rectangle(colorImage, humans[i], Scalar(rand() % 255 + 1, rand() % 255 + 1, rand() % 255 + 1), 1, LINE_8);

                   auto center = Point(humans[i].x + (humans[i].width / 2), humans[i].y + (humans[i].height / 2));

                   circle(colorImage, center, 2, Scalar(0, 0, 255), FILLED, LINE_8);

		   // putText(color, format("(%d,%d)", center.x + 15, center.y), center, FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0, 255, 0), 1, LINE_AA);
                }

		// select first object detected and calculate position
		if (!humans.empty()) {
                	auto firstHumanDetected = Point(humans[0].x + (humans[0].width / 2), humans[0].y + (humans[0].height / 2));

			// convert to fin steps & driver motors
			auto steps = convertImageCoordinatesToDriveSteps(Size(FRAME_WIDTH,FRAME_HEIGHT), firstHumanDetected);

			// add position steps in numers to the image
			circle(colorImage, Point(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), 3, Scalar(0, 255, 0), FILLED, LINE_8);

			putText(colorImage, format("%d", steps.x), Point(FRAME_WIDTH / 2 - 5 - 5, FRAME_HEIGHT / 2 - 25), FONT_HERSHEY_SIMPLEX, 0.35, Scalar(0, 255, 0), 1, LINE_AA);
			putText(colorImage, format("%d", steps.x), Point(FRAME_WIDTH / 2 - 5 - 5, FRAME_HEIGHT / 2 + 25), FONT_HERSHEY_SIMPLEX, 0.35, Scalar(0, 255, 0), 1, LINE_AA);

			putText(colorImage, format("%d", steps.y), Point(FRAME_WIDTH / 2 - 25 - 5, FRAME_HEIGHT / 2), FONT_HERSHEY_SIMPLEX, 0.35, Scalar(0, 255, 0), 1, LINE_AA);
        		putText(colorImage, format("%d", steps.y), Point(FRAME_WIDTH / 2 + 25 - 5 - 5, FRAME_HEIGHT / 2), FONT_HERSHEY_SIMPLEX, 0.35, Scalar(0, 255, 0), 1, LINE_AA);

			// save image
			imwrite("output.jpg", colorImage);

			// show image
			 system("sudo fbi -T 2 -d /dev/fb0 -noverbose output.jpg >/dev/null 2>&1");

			// delay
			// delay(1000);
        	}
	}
}



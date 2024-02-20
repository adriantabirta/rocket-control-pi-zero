#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>     /* srand, rand */
#include <unistd.h>

#include <sstream>
#include <string>
#include <vector>
#include <iostream>

#include "common.cpp"
#include "servo.cpp"
#include "camera.cpp"
#include "yolo-integration.cpp"

const int ledPin = 14; //25;

// Define GPIO pins connected to the stepper motor driver
#define ENABLE_PIN 27 // Example GPIO 27

using namespace std;
using namespace cv;

void processFrameFromCamera() {
	// skip 2/3 from camera frames
	// for 1/3 calculate fins position & update them
}

int main(int argc, char** argv) {

	// Setup pins
        wiringPiSetupGpio(); // Initialise WiringPi with Broadcom GPIO pins
        pinMode(ledPin, OUTPUT); // Set LED Pin as an output

        enablePins();

	auto net = createYoloNetwork();
	auto classList = load_class_list();

	Mat frame;
	bool debug = true;

	auto capture = initLocalVideoCapture("../resources/videos/sample2.mp4");

	int frameNumber = 1; // 0;

	while(true) {

		// kill all fbi processes to free resources
         	// system("sudo kill $(pgrep fbi) >/dev/null 2>&1");

		// read frame
        	capture.read(frame);
        	if (frame.empty()) {
            		std::cout << "End of stream\n";
            		break;
        	}

		++frameNumber;
		if (frameNumber % 30 != 0) {
			printf("skipped frame: %d \n", frameNumber);
			continue;
		}

		if (debug) {
			printf("Frame captured: w: %d h: %d \n", frame.size().width, frame.size().height);
        		// std::cout << "Frame captured\n";
			// std::cout << frame.size().width << " " << frame.size().height << "\n";
		}

		// crop image from center 640x640
		auto croppedImage = cropImageFromCenter(frame, frame.size(), Size(INPUT_WIDTH, INPUT_HEIGHT));

		// show image
		if (debug) {
        		// save
                	// imwrite("../resources/output.jpg", croppedImage);
			// send to video frame buffer
			// system("sudo fbi -T 2 -d /dev/fb0 -noverbose ../resources/output.jpg >/dev/null 2>&1");
		}

		// detect
		auto detection = detectFirstObjectWithBiggestPrecision(net, croppedImage, classList, debug);

		// draw rectangle
		if (debug) {
			// draw rectangle
			// std::cout << "detection: " << detection.box.width << " " << detection.box.height << "\n";

			// draw object center + rectangle + coord text
			auto center = Point(detection.box.x + (detection.box.width / 2), detection.box.y + (detection.box.height / 2));
                   	circle(croppedImage, center, 3, Scalar(0, 0, 255), FILLED, LINE_8);
			rectangle(croppedImage, detection.box, Scalar(0, 0, 255), 2);
			putText(croppedImage, format("(%d,%d)", center.x + 15, center.y), center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1, LINE_AA);

			// add center target lines + text
			circle(croppedImage, Point(croppedImage.size().width / 2, croppedImage.size().height / 2), 3, Scalar(255, 0, 0), FILLED, LINE_8);
			line(croppedImage,
				 Point(croppedImage.size().width / 4, croppedImage.size().height / 2),
				 Point(croppedImage.size().width * 0.75, croppedImage.size().height / 2),
				 Scalar(255, 0, 0), 1, LINE_8
			);

			line(croppedImage,
                                 Point(croppedImage.size().width / 2, croppedImage.size().height / 4),
                                 Point(croppedImage.size().width / 2, croppedImage.size().height - (croppedImage.size().height / 4)),
                                 Scalar(255, 0, 0), 1, LINE_8
			);

			// save image
			// imwrite("../resources/output.jpg", croppedImage);

                        // send to video frame buffer
                        // system("sudo fbi -T 2 -d /dev/fb0 -noverbose ../resources/output.jpg >/dev/null 2>&1");
		}

		// convert
		auto steps = convertImageCoordinatesToDriveSteps(croppedImage.size(), detection.box, debug);

		if (debug) {
			printf("steps x: %d y: %d \n", steps.x, steps.y);
			putText(croppedImage, format(" x:%d ", steps.x),
				Point(croppedImage.size().width - (croppedImage.size().width / 4), croppedImage.size().height / 2 + 5),
					FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1, LINE_AA);
			putText(croppedImage, format(" y:%d ", steps.y),
                                Point(croppedImage.size().width / 2 - 15, croppedImage.size().height / 4 - 10),
                                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1, LINE_AA);

			// save image
                        imwrite("../resources/output.jpg", croppedImage);

                        // send to video frame buffer
                        system("sudo fbi -T 2 -d /dev/fb0 -noverbose ../resources/output.jpg >/dev/null 2>&1");
		}

		// drive
                driveMotors(steps);
	}

	// detection.box

	// loopLocallyUsingHaarAlgo();

	return 0;

	// cout << cv::getBuildInformation() << std::endl;
	// cout << cv::checkHardwareSupport(CV_CPU_SSE2);

	ofstream logFile;
        logFile.open ("logs.csv", std::ios_base::app);

	ifstream logFile2;
	logFile2.open ("logs.csv");
	if(is_file_empty(logFile2)) {
		logFile << "rotate_x,rotate_y,elapsed_time_sec" << endl;
	}
	logFile2.close();

	int stop = 0;

	srand (time(NULL));

	while (stop < 10) {

		// Point detectedObjectCenterPoint = Point(1 + (rand() % frameWidth), 1 + (rand() % frameHeight));

		// long long start_time = cv::getTickCount();

		// convert 
		// auto steps = convertImageCoordinatesToDriveSteps(cv::Size(frameWidth, frameHeight), detectedObjectCenterPoint);

		// drive motors
		// driveMotors(steps);

      		// logFile << steps.x << "," << steps.y << "," << ((1/cv::getTickFrequency()) * (cv::getTickCount() - start_time)) << endl;

		// stop++;
	}

	logFile.close();

	return 0;

	// Turn LED On
        // printf("LED On\n");
        // digitalWrite(ledPin, HIGH);

        // delay(1000);

        // Turn LED off
        // printf("LED Off\n");
        // digitalWrite(ledPin, LOW);

        // delay(1000);

	// ========== End pin setup ============

	// === Haar ===
	CascadeClassifier human_cascade = CascadeClassifier("haarcascade_fullbody.xml");

	//Detector detector;
	// HOGDescriptor detector;
	//detector.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());

	// String human_cascade_name = samples::findFile("/home/at/processing/haarcascade_fullbody.xml");
	//cout << human_cascade_name;

	//if( !human_cascade.load( human_cascade_name ) ) {
        //	cout << "--(!)Error loading human cascade\n";
        //	return -1;
    	//};

	// === End Haar ===

	//int max_frames = 5;
	//std::vector<cv::Mat> frames(max_frames); // stores the video sequence for the demo

    	// Video capture parameters
    	//int width = 1280; // 640;
    	//int height = 800; // 240;
    	//int bytesPerFrame = width*height;
    	//int fps = 30; // 250; // setting to 250 will request the maximum framerate possible

    	// "raspividyuv" is the command that provides camera frames in YUV format
    	//  "--output -" specifies stdout as the output
    	//  "--timeout 0" specifies continuous video
    	//  "--luma" discards chroma channels, only luminance is sent through the pipeline
    	// see "raspividyuv --help" for more information on the parameters

	// std::stringstream ss;
    	// ss << "/bin/raspividyuv -w " << std::to_string(width) << " -h " << std::to_string(height) << " --output - --timeout 0 --framerate " << std::to_string(fps) << " --luma --nopreview";
    	// std::string videoCmd = ss.str();

    	// start the camera
    	// FILE *cameraProcess;

    	// if ((cameraProcess = popen(videoCmd.c_str(), "r")) == NULL) {
        //	printf("Error starting raspividyuv\n");
        //	return -1;
    	// }


    	// create buffer for camera data
    	// char* buffer = new char[bytesPerFrame];
    	// cv::Mat frame(height, width, CV_8UC1, (unsigned char*)buffer);

    	// wait for the first frame and discard it (only done to measure time more accurately)
   	// fread(buffer, bytesPerFrame, 1, cameraProcess);


    	// Mat frame2 = Mat (height, width, CV_8UC1, Scalar((rand() & 255), (rand() & 255), (rand() & 255)));
	// Mat frame2 = Mat (height, width, CV_8UC1);
	// Mat frame_gray;
	// cvtColor(frame2, frame_gray, COLOR_BGR2GRAY);

	// VideoCapture cap("djimini3activetrack720p.mp4"); 

	 // Check if camera opened successfully
  	//if(!cap.isOpened()){
    	//	cout << "Error opening video stream or file" << endl;
    	//	return -1;
  	//}

	//Mat color; // used to debug
 	//Mat frame = Mat(frameWidth, frameHeight, CV_8UC1);

	// equalizeHist(frame, frame);

	printf("Recording...\n");

    	long long start_time = cv::getTickCount();
	std::vector<Rect> humans;
        vector<Point> found;

 	String path("images/*.jpg");
	vector<cv::String> fn;
	cv::glob(path,fn,true); // recurse

	for (size_t k=0; k<fn.size(); ++k){

	// for (int frameNo = 0; frameNo < max_frames; frameNo++) {
	// while(1){

		digitalWrite(ledPin, HIGH);

		// kill all fbi processes to free resources
        	system("sudo kill $(pgrep fbi)");

		frame = imread(fn[k], IMREAD_GRAYSCALE);

		if (frame.empty()) {
			digitalWrite(ledPin, LOW);
			continue; //only proceed if sucsessful
		}

		// cap >> frame;

		// If the frame is empty, break immediately
    		//if (frame.empty())
		//	break;

		// printf("Frame in\n");

        	// Parse the raw stream into our buffer
        	// fread(buffer, bytesPerFrame, 1, cameraProcess);

        	// The frame can be processed here using any function in the OpenCV library.

		// ==== Process Image ====


		// human_cascade.detectMultiScale(frame, humans);
		human_cascade.detectMultiScale(frame, humans, 1.3, 1, 0, Size(20,20), Size());

		// cvtColor(frame, color, cv::COLOR_GRAY2BGR);

		for ( size_t i = 0; i < humans.size(); i++ ) {
		   // rectangle(color, humans[i], Scalar(rand() % 255 + 1, rand() % 255 + 1, rand() % 255 + 1), 1, LINE_8);

		   // auto center = Point(humans[i].x + (humans[i].width / 2), humans[i].y + (humans[i].height / 2));

                   // circle(color, center, 2, Scalar(0, 0, 255), FILLED, LINE_8);

                   // putText(color, format("(%d,%d)", center.x + 15, center.y), center, FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0, 255, 0), 1, LINE_AA);
		}


		// center.x > width / 2 => rotate right direction stepper motors on X axis with nr of steps (x >= (3/4*width) -> 2 else x<= (1/2*width) -> 1 else 0) 


		// detector.detectMultiScale(frame, humans, 0, Size(64,128), Size(), 1.2, 2, true);
		// detector.detect(frame, found, 0, Size(64,128));

		// imwrite("output.jpg", color);

		system("sudo fbi -T 2 -d /dev/fb0 -noverbose output.jpg >/dev/null 2>&1");

		digitalWrite(ledPin, LOW);

		// std::cout << "Found: " << humans.size() << '\n';
		// std::cout << "Found: " << found.size() << '\n';
		// frame.copyTo(frames[frameNo]); // save the frame (for the demo)
    	}

    	long long end_time = cv::getTickCount();

    	// pclose(cameraProcess);
    	// delete[] buffer;

    	// printf("Done! Result: %f fps\n", (cv::getTickFrequency() / (end_time - start_time))*max_frames);

	/*
    	printf("Writing frames to disk...\n");

    	cv::VideoWriter out("/home/at/processing/slow_motion.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(width, height));

    	cv::Mat rgbFrame;
    	for (int frameNo = 0; frameNo < max_frames; frameNo++) {
        	cvtColor(frames[frameNo], rgbFrame, cv::COLOR_GRAY2BGR); // video codec requires BGR image
        	out.write(rgbFrame);
   	}
	*/

	return 0;
}

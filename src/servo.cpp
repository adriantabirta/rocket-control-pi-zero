#include <stdio.h>
#include <wiringPi.h>
#include <cassert>

#include <opencv2/opencv.hpp>
#include "opencv2/objdetect.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/core/types.hpp>

using namespace cv;

// Define constants for stepper motor control

// Bipolar stepper motor (8mm, 20 steps)
// Step angle: 18 deg
// Total steps: 360 / 18 = 20 steps
// Resistance: 40 ohms°
// Check: https://aliexpress.ru/item/1005006345742170.html?spm=a2g2w.orderdetail.0.0.64394aa62mSmZd&sku_id=12000036836049440
#define STEPS_PER_REVOLUTION 20 // bipolar stepper motor with 20 steps per revolution

// Adjust the motor speed
//100000 // 100_000 = 0.1 seconds 
// was 2000 // Microseconds delay between steps (adjust as needed for your motor)
#define STEP_DELAY_US 2000

//====== X axis motors pins ======

#define MOTOR_X_STEP_PIN 4
#define MOTOR_X_DIRECTION_PIN 14

//====== Y axis motors pins ======

#define MOTOR_Y_STEP_PIN 22
#define MOTOR_Y_DIRECTION_PIN 23

// Maximum allowed steps in one direction
const int maximumStepsAllowed = (STEPS_PER_REVOLUTION * 0.5) - 1 - 1;

// Number of steps per quadrand. Used to move fins from -4...0...4.
const float numberOfStepsPerQuadrant = 4.0; // was 3;

void enablePins() {
    wiringPiSetupGpio(); // Initialize WiringPi

    // Set GPIO pins as outputs
    pinMode(MOTOR_X_STEP_PIN, OUTPUT);
    pinMode(MOTOR_X_DIRECTION_PIN, OUTPUT);

    pinMode(MOTOR_Y_STEP_PIN, OUTPUT);
    pinMode(MOTOR_Y_DIRECTION_PIN, OUTPUT);

    // todo: add othe settings here for stepper motor like: enable, full, half, ect ...
}

// - motorPin:  signal pin to send rotation impulses
// - directionPin: signal pin to set clockwire/counter-clockwise rotation
// - steps: nr of steps to make
void driveStepperMotorUsing(int motorPin, int directionPin, int steps) {

	// Maximum allowd steps (8) from one side to another
	//
	//		          _5_
	//		           |
 	//  		 4steps    |   4steps
	//			   |
	// 	0deg(step1)________|________180deg(step 10)
	//
	//

 	// assert(("Maximum number of steps exceeded", abs(steps) < maximumStepsAllowed - 1));

	// Set direction
        digitalWrite(directionPin, steps > 0);

        // Step the motor
        for (int i = 0; i < abs(steps); i++) {
                digitalWrite(motorPin, HIGH);
                usleep(STEP_DELAY_US); // Wait
                digitalWrite(motorPin, LOW);
                usleep(STEP_DELAY_US); // Wait
        }
}

Point convertImageCoordinatesToDriveSteps(Size2f imageSize, Rect box, bool debug) {
	assert(("Image size should be greated than zero", imageSize.width > 0 && imageSize.height > 0));

	auto centerPoint = Point(box.x + box.width / 2, box.y + box.height / 2);

        /*
                 x = column - (width / 2)
                 y = -(row - (height / 2))
        */

        // convert image coordinates to Cartesian

        float x = floorf(centerPoint.x - (imageSize.width / 2));
        float y = -(floorf(centerPoint.y - (imageSize.height / 2)));

	if (debug) {
		printf("deviation from center: x %f y %f\n", x, y);
	}

        /* Convert image coordinates to Cartesian

                ________________|____1____2____3
                3               |               |
                2       ||      |       |       |
                1               |               |
                0-------------(0,0)-------------
                1               |
                2       |||     |       |V
                3               |

                Formula:

                   x = column - (width / 2)
                   y = -(row - (height / 2))

                Into: avg time to make this calculations is 0.004 sec (4ms)
                Todo:
                        - Save last positions for (x, y) for cases when calculated steps
                change the sign (we need aditional steps, +3 -> 0 -> -3 = 5 steps needed)
                        - FIFO queue to calculate next steps when we skip image frame
                 (camera 30+fps, YOLO alg15 fps ~ 1/2 we skip). From prev & current steps we can approx. with
                quantum of 0.25 next position.
                When real frame is processed FIFO will be cleaned (In yoloV5 detection we trust)
        */

        // todo: save save current position for cases when pos=3, nextPos=-3
        // when sign changes we need to recalculate steps to 0 and from 0 to desired position.

        int stepsForX = round((numberOfStepsPerQuadrant / (imageSize.width * 0.5)) * float(x));
        int stepsForY = round((numberOfStepsPerQuadrant / (imageSize.height * 0.5)) * float(y));

	if (debug) {
		printf("X => round((%f / (%f * %f) * %f) = %d\n",numberOfStepsPerQuadrant, imageSize.width, 0.5, x, stepsForX);
        	printf("Y => round((%f / (%f * %f) * %f) = %d\n",numberOfStepsPerQuadrant, imageSize.height, 0.5, y, stepsForY);
	}

        return cv::Point(stepsForX, stepsForY);
}

// Send impulses to servo motors to actuate fins
void driveMotors(Point steps) {
	driveStepperMotorUsing(MOTOR_X_STEP_PIN, steps.x, MOTOR_X_DIRECTION_PIN);
        driveStepperMotorUsing(MOTOR_Y_STEP_PIN, steps.y, MOTOR_Y_DIRECTION_PIN);
}

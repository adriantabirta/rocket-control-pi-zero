#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>

using namespace cv;
using namespace dnn;
using namespace std;

const float INPUT_WIDTH = 640.0;
const float INPUT_HEIGHT = 640.0;
const float SCORE_THRESHOLD = 0.2;
const float NMS_THRESHOLD = 0.4;
const float CONFIDENCE_THRESHOLD = 0.4;

const std::vector<cv::Scalar> colors = {cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 0)};

struct Detection {
	int class_id;
    	float confidence;
    	cv::Rect box;
};

std::vector<std::string> load_class_list(String classesPath = "../resources/yolo/classes.txt") {
    std::vector<std::string> class_list;
    std::ifstream ifs(classesPath);
    std::string line;
    while (getline(ifs, line)) {
        class_list.push_back(line);
    }
    return class_list;
}

Net createYoloNetwork(String netPath = "../resources/yolo/yolov5s.onnx") {
    	auto net = cv::dnn::readNet(netPath);
     	std::cout << "Yolo v5 init\n";
     	net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
     	net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
	return net;
}

cv::Mat format_yolov5(const cv::Mat &source) {
    int col = source.cols;
    int row = source.rows;
    int _max = MAX(col, row);
    cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
    source.copyTo(result(cv::Rect(0, 0, col, row)));
    return result;
}

void detect(cv::dnn::Net &net, cv::Mat &image, std::vector<Detection> &output, const std::vector<std::string> className) {
    cv::Mat blob;

    auto input_image = format_yolov5(image);

    cv::dnn::blobFromImage(input_image, blob, 1./255., cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(), true, false);
    net.setInput(blob);
    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    float x_factor = input_image.cols / INPUT_WIDTH;
    float y_factor = input_image.rows / INPUT_HEIGHT;

    float *data = (float *)outputs[0].data;

    const int dimensions = 85;
    const int rows = 25200;

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (int i = 0; i < rows; ++i) {

        float confidence = data[4];
        if (confidence >= CONFIDENCE_THRESHOLD) {

            float * classes_scores = data + 5;
            cv::Mat scores(1, className.size(), CV_32FC1, classes_scores);
            cv::Point class_id;
            double max_class_score;
            minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
            if (max_class_score > SCORE_THRESHOLD) {

                confidences.push_back(confidence);

                class_ids.push_back(class_id.x);

                float x = data[0];
                float y = data[1];
                float w = data[2];
                float h = data[3];
                int left = int((x - 0.5 * w) * x_factor);
                int top = int((y - 0.5 * h) * y_factor);
                int width = int(w * x_factor);
                int height = int(h * y_factor);
                boxes.push_back(cv::Rect(left, top, width, height));
            }

        }

   	data += 85;
    }

    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, nms_result);
    for (int i = 0; i < nms_result.size(); i++) {
        int idx = nms_result[i];
        Detection result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.box = boxes[idx];
        output.push_back(result);
    }
}

// Find object with greates confidence percentage and closest to the center;
Detection detectFirstObjectWithBiggestPrecision(cv::dnn::Net &net, cv::Mat &frame, const std::vector<std::string> &class_list, bool debug) {

	std::vector<Detection> output;

	detect(net, frame, output, class_list);

	int detections = output.size();

	// sort by closest to the center
	auto frameCenter = Point(frame.size().width / 2, frame.size().height / 2);
	std::sort(output.begin(), output.end(), [frameCenter](const Detection& lhs, const Detection& rhs) {
		auto distanceFromCenterLeft = sqrt(
						    pow(frameCenter.x - (lhs.box.x + (lhs.box.width / 2)),2) +
						    pow(frameCenter.y - (lhs.box.y + (lhs.box.height / 2)),2)
						);

		auto distanceFromCenterRight = sqrt(
                                                     pow(frameCenter.x - (rhs.box.x + (rhs.box.width / 2)), 2) +
                                                     pow(frameCenter.y - (rhs.box.y + (rhs.box.height / 2)), 2)
                                                );

   		return distanceFromCenterLeft < distanceFromCenterRight; //&& lhs.confidence > rhs.confidence;
	});

 	if (debug) {
		printf("found %d objects\n", detections);
   		for (int i = 0; i < detections; ++i) {

            		auto detection = output[i];
            		auto classId = detection.class_id;

			const auto color = colors[classId % colors.size()];
			cv::rectangle(frame, detection.box, color, 1);
			// printf("%d %s %f \n", i, class_list[classId].c_str(), detection.confidence);
        	}
	}

	return output.at(0);
}





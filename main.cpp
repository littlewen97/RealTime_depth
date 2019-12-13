
#include <stdio.h>
#include <stdlib.h>
#include <k4a/k4a.hpp>
#include <k4abt.h>

#include <fstream>
#include <iostream>
#include <vector>
#include <array>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include "Pixel.h"
#include "DepthPixelColorizer.h"
#include "StaticImageProperties.h"


#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }  

using namespace std;
using namespace cv;
using namespace sen;

Rect select;
bool mousedown_flag = false; //鼠标按下的标识符
bool select_flag = false;    //选择区域的标识符
Point origin;
Mat frame;


void onMouse(int event, int x, int y, int, void*)
{
	//注意onMouse是void类型的，没有返回值！
	//为了把这些变量的值传回主函数，这些变量必须设置为全局变量	
	if (mousedown_flag)
	{
		select.x = MIN(origin.x, x);     //不一定要等鼠标弹起才计算矩形框，而应该在鼠标按下开始到弹起这段时间实时计算所选矩形框
		select.y = MIN(origin.y, y);
		select.width = abs(x - origin.x);                  //算矩形宽度和高度
		select.height = abs(y - origin.y);
		//select &= Rect(0, 0, frame.cols, frame.rows);       //保证所选矩形框在视频显示区域之内
		cout << "rect: " << select.x << " " << select.y << " " << select.width << " " << select.height << endl;
	}
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		//cout << "initialize" << endl;
		mousedown_flag = true;
		select_flag = false;
		origin = Point(x, y);
		//cout << "initialize_x&y: " << x << " " << y << endl;
		select = Rect(x, y, 0, 0);           //这里一定要初始化，宽和高为(0,0)是因为在opencv中Rect矩形框类内的点是包含左上角那个点的，但是不含右下角那个点
	}
	else if (event == CV_EVENT_LBUTTONUP)
	{
		//cout << "BUTTON_UP" << endl;
		mousedown_flag = false;
		select_flag = true;
	}
}

void ave_depth(unsigned long sum_depth, int width, int height) {
	long points_num = (width + 1) * (height + 1);
	long ave_depth = sum_depth / points_num;
	cout <<ave_depth << endl;
}

void sum_depth(Rect &select, const k4a::image& depthImage)
{
	const int width = depthImage.get_width_pixels();
	const int height = depthImage.get_height_pixels();
	int bon_w = select.x + select.width;
	int bon_h = select.y + select.height;
	const uint16_t* depthData = reinterpret_cast<const uint16_t*>(depthImage.get_buffer());
	unsigned long sum_depth = 0;
	if (select.y != bon_h && select.x != bon_w) {
		for (int h = select.y; h <= bon_h; ++h)
		{
			for (int w = select.x; w <= bon_w; ++w)
			{
				const size_t currentPixel = static_cast<size_t>(h * width + w);
				sum_depth += (unsigned long)depthData[currentPixel];
				if (h == bon_h && w == bon_w) {
					ave_depth(sum_depth, select.width, select.height);
				}
			}
		}
	}
	
}



int main(int argc, char **argv)
{
	const uint32_t deviceCount = k4a::device::get_installed_count();
	if (deviceCount == 0)
	{
		cout << "no azure kinect devices detected!" << endl;
	}

	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_15;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	config.synchronized_images_only = true;

	cout << "Started opening K4A device..." << endl;
	k4a_device_t device = NULL;
	k4a_device_open(0, &device);
	//k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT);
	k4a_device_start_cameras(device, &config);
	//device.start_cameras(&config);
	cout << "Finished opening K4A device." << endl;

	//body
	k4a_calibration_t sensorCalibration;
	VERIFY(k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &sensorCalibration),
		"Get depth camera calibration failed!");
	int depthWidth = sensorCalibration.depth_camera_calibration.resolution_width;
	int depthHeight = sensorCalibration.depth_camera_calibration.resolution_height;
	cout << depthWidth << " " << depthHeight << endl;
	k4abt_tracker_t tracker = nullptr;
	k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
	tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_CPU;
	VERIFY(k4abt_tracker_create(&sensorCalibration, tracker_config, &tracker), "Body tracker initialization failed!");

	std::vector<Pixel> depthTextureBuffer;
	//std::vector<Pixel> irTextureBuffer;
	uint8_t *colorTextureBuffer;

	k4a_capture_t capture;

	//k4a_image_t depthImage=k4a_capture_get_depth_image(capture);
	//k4a_image_t colorImage= k4a_capture_get_color_image(capture);
	//k4a::image irImage;

	cv::Mat depthFrame;
	cv::Mat colorFrame;
	cv::Mat bodyFrame;
	//cv::Mat irFrame;

	namedWindow("kinect depth map master", 1);
	setMouseCallback("kinect depth map master", onMouse, 0);
	
	while (1)
	{
		if (k4a_device_get_capture(device, &capture, K4A_WAIT_INFINITE)== K4A_WAIT_RESULT_SUCCEEDED)
		{
			{
				k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, capture, 0);
				k4abt_frame_t bodyFrame = nullptr;
				k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, 0);
				if(popFrameResult == K4A_WAIT_RESULT_SUCCEEDED) {
					//cout << "popFrameResult == K4A_WAIT_RESULT_SUCCEEDED" << endl;
					k4a_capture_t originalCapture = k4abt_frame_get_capture(bodyFrame);
					k4a_image_t bodyIndexMap = k4abt_frame_get_body_index_map(bodyFrame);
					const uint8_t* bodyIndexMapBuffer = k4a_image_get_buffer(bodyIndexMap);
					for (int i = 0; i < depthWidth * depthHeight; i++)
					{
						uint8_t bodyIndex = bodyIndexMapBuffer[i];
						if (bodyIndex != K4ABT_BODY_INDEX_MAP_BACKGROUND)
						{
							uint32_t bodyId = k4abt_frame_get_body_id(bodyFrame, bodyIndex);
						}
					}
					uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
					for (uint32_t i = 0; i < numBodies; i++)
					{
						k4abt_body_t body;
						k4abt_frame_get_body_skeleton(bodyFrame, i, &body.skeleton);
						body.id = k4abt_frame_get_body_id(bodyFrame, i);

						for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
						{
							if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW)
							{
								const k4a_float3_t& jointPosition = body.skeleton.joints[joint].position;
								const k4a_quaternion_t& jointOrientation = body.skeleton.joints[joint].orientation;
							}
						}

						float SHOULDER_RIGHT_xPos = body.skeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT].position.xyz.x;
						float SHOULDER_RIGHT_yPos = body.skeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT].position.xyz.y;
						float SHOULDER_LEFT_xPos = body.skeleton.joints[K4ABT_JOINT_SHOULDER_LEFT].position.xyz.x;
						float SHOULDER_LEFT_yPos = body.skeleton.joints[K4ABT_JOINT_SHOULDER_LEFT].position.xyz.y;
						float SPINE_CHEST_xPos = body.skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.xyz.x;
						float SPINE_CHEST_yPos = body.skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.xyz.y;

						float width = abs(SHOULDER_RIGHT_xPos - SHOULDER_LEFT_xPos);
						float height = abs(SHOULDER_RIGHT_yPos - SPINE_CHEST_yPos);

						std::cout << "right: " << (int)SHOULDER_RIGHT_xPos << " y: " << (int)SHOULDER_RIGHT_yPos << std::endl;
						//std::cout << "x: " << 320 + (int)SHOULDER_RIGHT_xPos <<" y: "<< 288 + (int)SHOULDER_RIGHT_yPos <<" width: "<<width<<" height: "<<height<< std::endl;
						select.x = depthWidth/2 + (int)SHOULDER_RIGHT_xPos;
						select.y = depthHeight/2 + (int)SHOULDER_RIGHT_yPos;
						select.width = (int)width;
						select.height = (int)height;
					}

					k4a_image_t depthImage = k4a_capture_get_depth_image(originalCapture);
					//k4a_image_t depthImage = k4a_capture_get_depth_image(capture);
					k4a_image_t colorImage = k4a_capture_get_color_image(capture);
					//irImage = capture.get_ir_image();

					ColorizeDepthImage2(depthImage, DepthPixelColorizer::ColorizeBlueToRed, GetDepthModeRange(config.depth_mode), &depthTextureBuffer);
					//ColorizeDepthImage(irImage, DepthPixelColorizer::ColorizeGreyscale, GetIrLevels(K4A_DEPTH_MODE_PASSIVE_IR), &irTextureBuffer);
					colorTextureBuffer = k4a_image_get_buffer(colorImage);

					depthFrame = cv::Mat(k4a_image_get_height_pixels(depthImage), k4a_image_get_width_pixels(depthImage), CV_8UC4, depthTextureBuffer.data());
					colorFrame = cv::Mat(k4a_image_get_height_pixels(colorImage), k4a_image_get_width_pixels(colorImage), CV_8UC4, colorTextureBuffer);
					//irFrame = cv::Mat(irImage.get_height_pixels(), irImage.get_width_pixels(), CV_8UC4, irTextureBuffer.data());


					//cal_Depth(select, depthImage);
					//画出矩形框
					rectangle(depthFrame, select, Scalar(0, 0, 255), 1, 8, 0);//能够实时显示在画矩形窗口时的痕迹
					//sum_depth(select,depthImage);

					cv::imshow("kinect depth map master", depthFrame);
					//cv::imshow("kinect color frame master", colorFrame);
					//cv::imshow("kinect ir frame master", irFrame);



				}
				
		
				

				
			}

		}
		if (waitKey(30) == 27 || waitKey(30) == 'q')
		{
			
			k4a_capture_release(capture);
			k4a_device_close(device);
			break;
		}
	}
	return 0;
}


///////////////////////////////////////////////////////////////////////////////////////



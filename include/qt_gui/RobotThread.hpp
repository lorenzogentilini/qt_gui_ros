#include <QtCore>
#include <QThread>
#include <QStringList>
#include <QMutex>

#include <stdlib.h>
#include <iostream>
#include "assert.h"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32MultiArray.h>
#include <mav_planning/Action.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#define TASK_LED 6
#define TAKEOFF_LED 0
#define LAND_LED 1
#define EXPLORE_LED 2
#define TRACKING_LED 3
#define INSPECT_LED 4
#define GOTO_LED 5
#define EMERGENCY_LED 7
#define NULL_LED 100

typedef struct{
	char task_type;
	int x_coordinate;
	char y_coordinate;
	int score;
} Task;

class RobotThread: public QObject{
	// Enable Meta-Object Features
	Q_OBJECT

	public:
	// Class Constructor
	RobotThread(int argc, char** argv);

	// Class Destructor
	~RobotThread();

	bool init();
	Q_SLOT void run();
	Q_SIGNAL void newPosition(double, double);
	Q_SIGNAL void setNewImage(cv::Mat);
	Q_SIGNAL void setNewSequence(std::vector<Task>);
	Q_SIGNAL void uploadLed(int);

	void sendSequence(std::vector<Task> seq);
	void optimizeSequence(std::vector<Task> seq);
	void sendManualLand();
	void sendEmergencyStop();
	void sendStartMission();
	void sendNextAction();

	std::string resourcesBaseFolder;

	private:
	int m_argc;
	char** m_argv;

	double m_xPose;
	double m_yPose;

	cv::Mat image;
	std::vector<Task> taskSequence;

	QThread* m_pThread;

	tf2_ros::Buffer _tfBuffer;
	tf2_ros::TransformListener* _tfListener;

	ros::Subscriber poseListener;
	ros::Subscriber imageListener;
	ros::Subscriber sequenceListener;
	ros::Subscriber actionListener;
	ros::Publisher sequencePublisher_sup;
	ros::Publisher sequencePublisher_vrp;
	ros::Publisher actionPublisher;

	ros::Timer actionTimer;

	void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);
	void optSequenceCallback(const std_msgs::Int32MultiArray::ConstPtr & msg);
	void actionCallback(const mav_planning::Action::ConstPtr & msg);
	void timerCallback(const ros::TimerEvent&);
};
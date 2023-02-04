#include <RobotThread.hpp>

RobotThread::RobotThread(int argc, char** argv):
	m_argc(argc), m_argv(argv){

	;
}

RobotThread::~RobotThread(){
	if(ros::isStarted()){
		ros::shutdown();
		ros::waitForShutdown();
	}

	m_pThread->wait();
	delete _tfListener;
}

bool RobotThread::init(){
	m_pThread = new QThread();
	this->moveToThread(m_pThread);
	connect(m_pThread, &QThread::started, this, &RobotThread::run);

	ros::init(m_argc, m_argv, "qt_gui_node");
	if(!ros::master::check())
		return false; // Do Not Start Without Ros.

	ros::start();
	ros::Time::init();
	ros::NodeHandle nh;

	resourcesBaseFolder = nh.param<std::string>("resources_base_folder", "/home/nvidia-agx/catkin_ws/src/qt_gui/qt_resources");
	ROS_INFO("[GUI]: Folder: %s",resourcesBaseFolder.c_str());
	
	// Declare Subscribers and Publishers
	poseListener = nh.subscribe("/mavros/local_position/odom", 1, &RobotThread::poseCallback, this);
	imageListener = nh.subscribe("/fiducial_images/compressed", 1, &RobotThread::imageCallback, this);
	sequenceListener = nh.subscribe("/vrp_solution", 1, &RobotThread::optSequenceCallback, this);
	actionListener = nh.subscribe("/actual_action", 1, &RobotThread::actionCallback, this);

	sequencePublisher_sup = nh.advertise<std_msgs::Int32MultiArray>("/task_sequence_opt", 1);
	sequencePublisher_vrp = nh.advertise<std_msgs::Int32MultiArray>("/task_sequence_raw", 1);
	actionPublisher = nh.advertise<mav_planning::Action>("/gui_action", 1);

	actionTimer = nh.createTimer(ros::Duration(1.0), &RobotThread::timerCallback, this);

	_tfListener = new tf2_ros::TransformListener(_tfBuffer);

	m_pThread->start();
	return true;
}

void RobotThread::run(){
	ros::Rate loopRate(10);

	while(ros::ok()){
		ros::spinOnce();
		loopRate.sleep();
	}

	return;
}

void RobotThread::poseCallback(const nav_msgs::Odometry::ConstPtr & msg){
	tf2::Transform mo_transformation;
	tf2::Transform ob_transformation;

	try{
		tf2::fromMsg(_tfBuffer.lookupTransform("map", "odom", ros::Time(0)).transform, mo_transformation);
	} catch(tf2::TransformException &ex){
		return;
	}

	tf2::Vector3 ob_translation(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);							
	tf2::Quaternion ob_quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

	ob_transformation.setRotation(ob_quaternion);
	ob_transformation.setOrigin(ob_translation);
	tf2::Transform mb_transformation = mo_transformation*ob_transformation;
	tf2::Vector3 mb_translation = mb_transformation.getOrigin();

	QMutex * pMutex = new QMutex();

	pMutex->lock();
	m_xPose = mb_translation.x();
	m_yPose = mb_translation.y();
	pMutex->unlock();

	delete pMutex;
	Q_EMIT newPosition(m_xPose, m_yPose);
}

void RobotThread::imageCallback(const sensor_msgs::CompressedImage::ConstPtr &msg){
	try{
		QMutex * pMutex = new QMutex();
		cv::Mat new_image = cv::imdecode(cv::Mat(msg->data), 1);
		//cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

		pMutex->lock();
		// image = cv_ptr->image;
		image = new_image;
		pMutex->unlock();

		delete pMutex;    
	} catch(cv_bridge::Exception & e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

	Q_EMIT setNewImage(image);
}

void RobotThread::optSequenceCallback(const std_msgs::Int32MultiArray::ConstPtr &msg){
	QMutex * pMutex = new QMutex();

	pMutex->lock();
	taskSequence.clear();
	taskSequence.shrink_to_fit();

	// Convert MSG to Task Structure
	for(uint ii = 0; ii < msg->data.size(); ii+=4){
		Task tt;
		tt.task_type = msg->data[ii];
		tt.x_coordinate = msg->data[ii+1];
		tt.y_coordinate = msg->data[ii+2];
		tt.score = msg->data[ii+3];

		taskSequence.push_back(tt);
	}
	pMutex->unlock();
	
	delete pMutex;

	Q_EMIT setNewSequence(taskSequence);
}

void RobotThread::timerCallback(const ros::TimerEvent&){
	Q_EMIT uploadLed(NULL_LED);
}

void RobotThread::actionCallback(const mav_planning::Action::ConstPtr & msg){
	switch(msg->action){
		case mav_planning::Action::TAKEOFF:{
			Q_EMIT uploadLed(TAKEOFF_LED);
			break;
		}

		case mav_planning::Action::EXPLORE:{
			Q_EMIT uploadLed(EXPLORE_LED);
			break;
		}

		case mav_planning::Action::TRACK:{
			Q_EMIT uploadLed(TRACKING_LED);
			break;
		}

		case mav_planning::Action::GOTO:{
			Q_EMIT uploadLed(GOTO_LED);
			break;
		}

		case mav_planning::Action::INSPECT:{
			Q_EMIT uploadLed(INSPECT_LED);
			break;
		}

		case mav_planning::Action::LAND:{
			Q_EMIT uploadLed(LAND_LED);
			break;
		}

		case mav_planning::Action::EMERGENCY_STOP:{
			Q_EMIT uploadLed(EMERGENCY_LED);
			break;
		}
	}
}

void RobotThread::sendSequence(std::vector<Task> seq){
	// Convert Task in MSG & Publish
	std_msgs::Int32MultiArray msg;
	for(uint ii = 0; ii < seq.size(); ii++){
		msg.data.push_back(seq[ii].task_type);
		msg.data.push_back(seq[ii].x_coordinate);
		msg.data.push_back(seq[ii].y_coordinate);
	}

	sequencePublisher_sup.publish(msg);
}

void RobotThread::optimizeSequence(std::vector<Task> seq){
	// Convert Task in MSG & Publish
	std_msgs::Int32MultiArray msg;
	for(uint ii = 0; ii < seq.size(); ii++){
		msg.data.push_back(seq[ii].task_type);
		msg.data.push_back(seq[ii].x_coordinate);
		msg.data.push_back(seq[ii].y_coordinate);
		msg.data.push_back(seq[ii].score);
	}

	sequencePublisher_vrp.publish(msg);
}

void RobotThread::sendManualLand(){
	mav_planning::Action msg;
	msg.action = mav_planning::Action::LAND;

	actionPublisher.publish(msg);
}

void RobotThread::sendNextAction(){
	mav_planning::Action msg;
	msg.action = mav_planning::Action::NEXT_ACTION;

	actionPublisher.publish(msg);
}

void RobotThread::sendEmergencyStop(){
	mav_planning::Action msg;
	msg.action = mav_planning::Action::EMERGENCY_STOP;

	actionPublisher.publish(msg);
}

void RobotThread::sendStartMission(){
	mav_planning::Action msg;
	msg.action = mav_planning::Action::START;

	actionPublisher.publish(msg);
}
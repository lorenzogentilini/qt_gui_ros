#include <ControlWindow.hpp>

// To Set Boxes Colors
// label_status[ii]->setStyleSheet(NOT_ACTIVE);
// label_status[ii]->setStyleSheet(ACTIVE_R);
// label_status[ii]->setStyleSheet(ACTIVE_G);
// label_status[ii]->setStyleSheet(ACTIVE_O);

// Class Constructor
ControlWindow::ControlWindow(int argc, char** argv, QWidget* parent): QWidget(parent), m_RobotThread(argc, argv){
	m_RobotThread.init();

	qRegisterMetaType<cv::Mat>("cv::Mat");
	qRegisterMetaType<std::vector<Task>>("std::vector<Task>");

	// Set-up All Widgets
	loadImages();

	// Images
	label_map = new QLabel();
	label_img = new QLabel();
	label_map->setPixmap(QPixmap::fromImage(QImage(img_mm.data, img_mm.cols, img_mm.rows, img_mm.step, QImage::Format_RGB888)));
	label_img->setPixmap(QPixmap::fromImage(QImage(img_ph.data, img_ph.cols, img_ph.rows, img_ph.step, QImage::Format_RGB888)));
	label_map->setAlignment(Qt::AlignCenter);
	label_img->setAlignment(Qt::AlignCenter);

	// Drone Status
	label_status.resize(8);
	for(uint ii = 0; ii < label_status.size(); ii++){
		label_status[ii] = new QLabel();
		label_status[ii]->setStyleSheet(NOT_ACTIVE);
		label_status[ii]->setAlignment(Qt::AlignCenter);
		label_status[ii]->setWordWrap(true);
	}

	// The Eight States
	label_status[0]->setText(tr("Takeoff"));
	label_status[1]->setText(tr("Land"));
	label_status[2]->setText(tr("Explore"));
	label_status[3]->setText(tr("Track Robot"));
	label_status[4]->setText(tr("Inspect Building"));
	label_status[5]->setText(tr("Move To Position"));
	label_status[6]->setText(tr("Optimal Sequence Received"));
	label_status[7]->setText(tr("Emergency Stop"));

	// Boxes For Task Handling
	comboBox_ac.resize(8);
	comboBox_xx.resize(8);
	comboBox_nn.resize(8);
	lineEdit_ss.resize(8);

	for(uint ii = 0; ii < comboBox_ac.size(); ii++){
		comboBox_ac[ii] = new QComboBox();
		comboBox_xx[ii] = new QComboBox();
		comboBox_nn[ii] = new QComboBox();
		lineEdit_ss[ii] = new QLineEdit();

		comboBox_ac[ii]->addItem(" ");
		comboBox_ac[ii]->addItem("F");
		comboBox_ac[ii]->addItem("A");
		
		comboBox_xx[ii]->addItem(" ");
		comboBox_xx[ii]->addItem("1");
		comboBox_xx[ii]->addItem("2");
		comboBox_xx[ii]->addItem("3");
		comboBox_xx[ii]->addItem("4");
		comboBox_xx[ii]->addItem("5");
		comboBox_xx[ii]->addItem("6");
		comboBox_xx[ii]->addItem("7");
		comboBox_xx[ii]->addItem("8");
		comboBox_xx[ii]->addItem("9");
		comboBox_xx[ii]->addItem("10");
		comboBox_xx[ii]->addItem("11");
		comboBox_xx[ii]->addItem("12");
		comboBox_xx[ii]->addItem("13");
		comboBox_xx[ii]->addItem("14");
		comboBox_xx[ii]->addItem("15");
		comboBox_xx[ii]->addItem("16");
		comboBox_xx[ii]->addItem("17");
		comboBox_xx[ii]->addItem("18");
		comboBox_xx[ii]->addItem("19");
		comboBox_xx[ii]->addItem("20");

		comboBox_nn[ii]->addItem(" ");
		comboBox_nn[ii]->addItem("A");
		comboBox_nn[ii]->addItem("B");
		comboBox_nn[ii]->addItem("C");
		comboBox_nn[ii]->addItem("D");
		comboBox_nn[ii]->addItem("E");
		comboBox_nn[ii]->addItem("F");
		comboBox_nn[ii]->addItem("G");
		comboBox_nn[ii]->addItem("H");
		comboBox_nn[ii]->addItem("I");
		comboBox_nn[ii]->addItem("L");
	}

	// Buttons
	pushButton_str = new QPushButton("Start Mission");
	pushButton_emr = new QPushButton("Emergency Stop");
	pushButton_val = new QPushButton("Validate Sequence");
	pushButton_sen = new QPushButton("Compute Opt. Sequence");
	pushButton_lan = new QPushButton("Manual Landing");
	pushButton_img = new QPushButton("Freeze Image");
	pushButton_act = new QPushButton("Next Task");

	pushButton_str->setMinimumSize(BUTTON_SIZE);
	pushButton_emr->setMinimumSize(BUTTON_SIZE);
	pushButton_val->setMinimumSize(BUTTON_SIZE);
	pushButton_sen->setMinimumSize(BUTTON_SIZE);
	pushButton_lan->setMinimumSize(BUTTON_SIZE);
	pushButton_img->setMinimumSize(BUTTON_SIZE);
	pushButton_act->setMinimumSize(BUTTON_SIZE);

	// Set-up Group Boxes
	groupBox_position = new QGroupBox(tr("Drone Position"));
	groupBox_status = new QGroupBox(tr("Drone Status"));
	groupBox_actions = new QGroupBox(tr("Actions"));
	groupBox_imageViewer = new QGroupBox(tr("Image Viewer"));
	groupBox_taskHandler = new QGroupBox(tr("Task Sequence Handler"));

	groupBox_nnTask.resize(8);
	groupBox_nnTask[0] = new QGroupBox("1: ");
	groupBox_nnTask[1] = new QGroupBox("2: ");
	groupBox_nnTask[2] = new QGroupBox("3: ");
	groupBox_nnTask[3] = new QGroupBox("4: ");
	groupBox_nnTask[4] = new QGroupBox("5: ");
	groupBox_nnTask[5] = new QGroupBox("6: ");
	groupBox_nnTask[6] = new QGroupBox("7: ");
	groupBox_nnTask[7] = new QGroupBox("8: ");

	// Set-up Layouts
	horizontalLayouts.resize(21);
	verticalLayouts.resize(4);

	for(uint ii = 0; ii < horizontalLayouts.size(); ii++){
		horizontalLayouts[ii] = new QHBoxLayout();
	}

	for(uint ii = 0; ii < verticalLayouts.size(); ii++){
		verticalLayouts[ii] = new QVBoxLayout();
	}

	horizontalLayouts[0]->addWidget(label_map);
	horizontalLayouts[1]->addWidget(label_img);

	groupBox_position->setLayout(horizontalLayouts[0]);
	groupBox_imageViewer->setLayout(horizontalLayouts[1]);

	horizontalLayouts[2]->addWidget(groupBox_position);
	horizontalLayouts[2]->addWidget(groupBox_imageViewer);

	horizontalLayouts[3]->addWidget(label_status[0]);
	horizontalLayouts[3]->addWidget(label_status[1]);
	verticalLayouts[0]->addLayout(horizontalLayouts[3]);

	horizontalLayouts[4]->addWidget(label_status[2]);
	horizontalLayouts[4]->addWidget(label_status[3]);
	verticalLayouts[0]->addLayout(horizontalLayouts[4]);

	horizontalLayouts[5]->addWidget(label_status[4]);
	horizontalLayouts[5]->addWidget(label_status[5]);
	verticalLayouts[0]->addLayout(horizontalLayouts[5]);

	horizontalLayouts[6]->addWidget(label_status[6]);
	horizontalLayouts[6]->addWidget(label_status[7]);
	verticalLayouts[0]->addLayout(horizontalLayouts[6]);
	groupBox_status->setLayout(verticalLayouts[0]);
	
	for(uint ii = 0; ii < groupBox_nnTask.size(); ii++){
		horizontalLayouts[7+ii]->addWidget(comboBox_ac[ii]);
		horizontalLayouts[7+ii]->addWidget(comboBox_xx[ii]);
		horizontalLayouts[7+ii]->addWidget(comboBox_nn[ii]);
		horizontalLayouts[7+ii]->addWidget(lineEdit_ss[ii]);
		groupBox_nnTask[ii]->setLayout(horizontalLayouts[7+ii]);
	}

	horizontalLayouts[16]->addWidget(groupBox_nnTask[0]);
	horizontalLayouts[16]->addWidget(groupBox_nnTask[1]);
	verticalLayouts[1]->addLayout(horizontalLayouts[16]);

	horizontalLayouts[17]->addWidget(groupBox_nnTask[2]);
	horizontalLayouts[17]->addWidget(groupBox_nnTask[3]);
	verticalLayouts[1]->addLayout(horizontalLayouts[17]);

	horizontalLayouts[18]->addWidget(groupBox_nnTask[4]);
	horizontalLayouts[18]->addWidget(groupBox_nnTask[5]);
	verticalLayouts[1]->addLayout(horizontalLayouts[18]);

	horizontalLayouts[19]->addWidget(groupBox_nnTask[6]);
	horizontalLayouts[19]->addWidget(groupBox_nnTask[7]);
	verticalLayouts[1]->addLayout(horizontalLayouts[19]);

	groupBox_taskHandler->setLayout(verticalLayouts[1]);

	verticalLayouts[2]->addWidget(pushButton_str);
	verticalLayouts[2]->addWidget(pushButton_emr);
	verticalLayouts[2]->addWidget(pushButton_lan);
	verticalLayouts[2]->addWidget(pushButton_sen);
	verticalLayouts[2]->addWidget(pushButton_val);
	verticalLayouts[2]->addWidget(pushButton_img);
	verticalLayouts[2]->addWidget(pushButton_act);

	groupBox_actions->setLayout(verticalLayouts[2]);

	horizontalLayouts[20]->addWidget(groupBox_status);
	horizontalLayouts[20]->addWidget(groupBox_taskHandler);
	horizontalLayouts[20]->addWidget(groupBox_actions);

	verticalLayouts[3]->addLayout(horizontalLayouts[2]);
	verticalLayouts[3]->addLayout(horizontalLayouts[20]);

	windowLayout = new QVBoxLayout();
	windowLayout->addLayout(verticalLayouts[3]);

  	setLayout(windowLayout);
  	setWindowTitle(tr("Control Window"));
  	show();

	// Create Connections
	connect(pushButton_str, &QPushButton::released, this, &ControlWindow::handleStartButton);
	connect(pushButton_emr, &QPushButton::released, this, &ControlWindow::handleEmergencyButton);
	connect(pushButton_val, &QPushButton::released, this, &ControlWindow::handleValidateButton);
	connect(pushButton_sen, &QPushButton::released, this, &ControlWindow::handleSequenceButton);
	connect(pushButton_lan, &QPushButton::released, this, &ControlWindow::handleLandButton);
	connect(pushButton_img, &QPushButton::released, this, &ControlWindow::handleImageButton);
	connect(pushButton_act, &QPushButton::released, this, &ControlWindow::handleActionButton);

  	connect(&m_RobotThread, &RobotThread::newPosition, this, &ControlWindow::updatePosition);
	connect(&m_RobotThread, &RobotThread::setNewImage, this, &ControlWindow::showNewImage);
	connect(&m_RobotThread, &RobotThread::setNewSequence, this, &ControlWindow::getNewSequence);
	connect(&m_RobotThread, &RobotThread::uploadLed, this, &ControlWindow::uploadStatusLed);
}

// Class Destructor
ControlWindow::~ControlWindow(){
	// Destroy all Objects
	delete windowLayout;
}

void ControlWindow::loadImages(){
	img_qq = cv::imread(m_RobotThread.resourcesBaseFolder+"/quadIcon.png", cv::IMREAD_UNCHANGED);
	img_mm = cv::imread(m_RobotThread.resourcesBaseFolder+"/map.png");
	img_ph = cv::imread(m_RobotThread.resourcesBaseFolder+"/Robot.png");

	cv::resize(img_qq, img_qq, cv::Size(30, 30));
	cv::resize(img_mm, img_mm, cv::Size(), 0.7, 0.7);
	cv::resize(img_ph, img_ph, cv::Size(600, 450));	
	
	cv::cvtColor(img_mm, img_mm, cv::COLOR_BGR2RGB);
	cv::cvtColor(img_ph, img_ph, cv::COLOR_BGR2RGB);
}

void ControlWindow::updatePosition(double x, double y){
	// Consider Positive Positions
	x = abs(x);
	y = abs(y);

	// Project Outside Map Points Inside Map
	x = x < 0 ? 0 : x; 
	y = y < 0 ? 0 : y;

	x = x > 20 ? 20 : x; 
	y = y > 10 ? 10 : y;  

	// Print Positions
	cv::Mat img = img_mm.clone();
	previousPositions.push_back(cv::Point(x*img.cols/20, y*img.rows/10));

	if(previousPositions.size() >= PREV_POSES_N){
		previousPositions.erase(previousPositions.begin());
	}

	cv::Mat new_img;
	for(uint ii = 0; ii < previousPositions.size(); ii++){
		if(ii != previousPositions.size()-1){
			cv::circle(img, previousPositions[ii], 1, cv::Scalar(200,0,0), 2);
		} else{
			overlayImage(img, img_qq, new_img, cv::Point(x*img.cols/20 - img_qq.size().width/2, y*img.rows/10 - img_qq.size().height/2), 1.0);
		}
	}

	label_map->setPixmap(QPixmap::fromImage(QImage(new_img.data, new_img.cols, new_img.rows, new_img.step, QImage::Format_RGB888)));
}

void ControlWindow::overlayImage(const cv::Mat &background, const cv::Mat &foreground, cv::Mat &output, cv::Point2i location, double opacity){
	background.copyTo(output);
  	
	for(int y = std::max(location.y , 0); y < background.rows; ++y){
		int fY = y - location.y;

		if(fY >= foreground.rows){
			break;
		}

		for(int x = std::max(location.x, 0); x < background.cols; ++x){
			int fX = x - location.x;

			if(fX >= foreground.cols){
				break;
			}

			double opacity_level = ((double)foreground.data[fY * foreground.step + fX * foreground.channels() + 3]) / 255.;
			if(opacity >= 0.0 && opacity < 1.0){
				opacity_level *= opacity;
			}

			for(int c = 0; opacity_level > 0 && c < output.channels(); ++c){
				unsigned char foregroundPx = foreground.data[fY * foreground.step + fX * foreground.channels() + c];
				unsigned char backgroundPx = background.data[y * background.step + x * background.channels() + c];
				output.data[y*output.step + output.channels()*x + c] = backgroundPx * (1.-opacity_level) + foregroundPx * opacity_level;
			}
		}
	}
}

void ControlWindow::uploadStatusLed(uint led){
	// Handle Sequence Led
	led_count ++;

	if(led == TASK_LED){
		led_count = 0;
		label_status[6]->setStyleSheet(ACTIVE_G);
		return;
	}

	if(led_count >= SWITCH_COUNT){
		label_status[6]->setStyleSheet(NOT_ACTIVE);
	}

	if(led == NULL_LED){
		return;
	}

	// Handle Status Led
	for(uint ii = 0; ii < label_status.size(); ii++){
		if(ii == 6){
			continue;
		}

		label_status[ii]->setStyleSheet(NOT_ACTIVE);
	}

	label_status[led]->setStyleSheet(ACTIVE_O);
	if(led == EMERGENCY_LED){
		label_status[led]->setStyleSheet(ACTIVE_R);
	}
}

void ControlWindow::showNewImage(cv::Mat img){
	if(freezeImage){
		return;
	}

	cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
	cv::resize(img, img, cv::Size(600, 450));
	label_img->setPixmap(QPixmap::fromImage(QImage(img.data, img.cols, img.rows, img.step, QImage::Format_RGB888)));
}

void ControlWindow::handleStartButton(){
	ROS_INFO("[GUI]: Start Mission!");
	m_RobotThread.sendStartMission();
}

void ControlWindow::handleEmergencyButton(){
	ROS_INFO("[GUI]: Emergency Stop!");
	m_RobotThread.sendEmergencyStop();
}

void ControlWindow::handleLandButton(){
	ROS_INFO("[GUI]: Manual Landing!");
	m_RobotThread.sendManualLand();
}

void ControlWindow::handleImageButton(){
	if(freezeImage){
		ROS_INFO("[GUI]: Unfreeze Image Streaming!");
		freezeImage = false;
	} else{
		ROS_INFO("[GUI]: Freeze Image Streaming!");
		freezeImage = true;
	}
}

void ControlWindow::handleActionButton(){
	ROS_INFO("[GUI]: Next Action!");
	m_RobotThread.sendNextAction();
}

void ControlWindow::handleValidateButton(){
	ROS_INFO("[GUI]: Sequence Validated!");

	// if(taskSequence.size() == 0){
		readSequence();
	// }

	m_RobotThread.sendSequence(taskSequence);
}

void ControlWindow::handleSequenceButton(){
	ROS_INFO("[GUI]: Compute Opt. Sequence!");
	readSequence();

	m_RobotThread.optimizeSequence(taskSequence);
}

void ControlWindow::readSequence(){
	taskSequence.clear();
	taskSequence.shrink_to_fit();
	
	for(uint ii = 0; ii < comboBox_ac.size(); ii++){
		Task tt;

		tt.task_type = comboBox_ac[ii]->currentText().toStdString()[0];
		tt.x_coordinate = comboBox_xx[ii]->currentText().toInt();
		tt.y_coordinate = comboBox_nn[ii]->currentText().toStdString()[0];
		tt.score = lineEdit_ss[ii]->text().toInt();

		if(tt.task_type == ' '){
			break;
		}

		ROS_INFO("[GUI]: %d Task: %c %d %c %d", ii+1, tt.task_type, tt.x_coordinate, tt.y_coordinate, tt.score);
		taskSequence.push_back(tt);
	}

	ROS_WARN("[GUI]: %ld Tasks Detected", taskSequence.size());

}

void ControlWindow::getNewSequence(std::vector<Task> seq){
	taskSequence = seq;

	for(uint ii = 0; ii < taskSequence.size(); ii++){
		comboBox_ac[ii]->setCurrentText(QString(taskSequence[ii].task_type));
		comboBox_nn[ii]->setCurrentText(QString(taskSequence[ii].y_coordinate));

		QString string;
		string.setNum(taskSequence[ii].score);
		lineEdit_ss[ii]->setText(string);

		string.setNum(taskSequence[ii].x_coordinate);
		comboBox_xx[ii]->setCurrentText(string);
	}

	if(comboBox_ac.size() - taskSequence.size() > 0){
		for(uint ii = taskSequence.size(); ii < comboBox_ac.size(); ii++){
			comboBox_ac[ii]->setCurrentText(" ");
			comboBox_xx[ii]->setCurrentText(" ");
			comboBox_nn[ii]->setCurrentText(" ");
			lineEdit_ss[ii]->setText(" ");
		}
	}

	// Control Led Status
	uploadStatusLed(TASK_LED);
}
#include <QtWidgets>
#include <QPushButton>
#include <QAbstractButton>
#include <QString>
#include <QComboBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTextStream>
#include <QLineEdit>
#include <QPalette>
#include <QIcon>

#include <RobotThread.hpp>

#define PREV_POSES_N 200

#define ACTIVE_R "QLabel{background-color: rgb(255, 128, 128); color: rgb(71, 71, 107); border-style: outset; border-width: 2px; border-radius: 10px; border-color: beige; font: 14px; min-width: 10em; padding: 6px;}"
#define ACTIVE_G "QLabel{background-color: rgb(121, 210, 166); color: rgb(71, 71, 107); border-style: outset; border-width: 2px; border-radius: 10px; border-color: beige; font: 14px; min-width: 10em; padding: 6px;}"
#define ACTIVE_O "QLabel{background-color: rgb(255, 163, 102); color: rgb(71, 71, 107); border-style: outset; border-width: 2px; border-radius: 10px; border-color: beige; font: 14px; min-width: 10em; padding: 6px;}"
#define NOT_ACTIVE "QLabel{background-color: rgb(194, 214, 214); color: rgb(71, 71, 107); border-style: outset; border-width: 2px; border-radius: 10px; border-color: beige; font: 14px; min-width: 10em; padding: 6px;}"

#define SWITCH_COUNT 5

class ControlWindow: public QWidget{
    // Enable Meta-Object Features
    Q_OBJECT

    public:
    // Class Constructor
    ControlWindow(int argc, char** argv, QWidget* parent = 0);

	Q_SLOT void uploadStatusLed(uint led);
    Q_SLOT void updatePosition(double x, double y);
    Q_SLOT void showNewImage(cv::Mat img);
	Q_SLOT void getNewSequence(std::vector<Task> seq);

    void handleStartButton();
    void handleEmergencyButton();
    void handleLandButton();
    void handleImageButton();
    void handleValidateButton();
    void handleSequenceButton();
    void handleActionButton();

	void readSequence();

    // Class Destructor
    ~ControlWindow();

    private:
    RobotThread m_RobotThread;

	// Group Boxes
    QGroupBox* groupBox_position;
    QGroupBox* groupBox_status;
    QGroupBox* groupBox_actions;
	QGroupBox* groupBox_imageViewer;
	QGroupBox* groupBox_taskHandler;
	std::vector<QGroupBox*> groupBox_nnTask;

	// Layouts
	QVBoxLayout* windowLayout;
	std::vector<QHBoxLayout*> horizontalLayouts;
	std::vector<QVBoxLayout*> verticalLayouts;

	// Widgets
	QLabel* label_map;
	QLabel* label_img;
	std::vector<QLabel*> label_status;
	std::vector<QComboBox*> comboBox_ac;
	std::vector<QComboBox*> comboBox_xx;
	std::vector<QComboBox*> comboBox_nn;
	std::vector<QLineEdit*> lineEdit_ss;

	QPushButton* pushButton_str;
	QPushButton* pushButton_emr;
	QPushButton* pushButton_val;
	QPushButton* pushButton_sen;
	QPushButton* pushButton_lan;
	QPushButton* pushButton_img;
	QPushButton* pushButton_act;

	cv::Mat img_mm;
	cv::Mat img_qq;
	cv::Mat img_ph;

	const QSize BUTTON_SIZE = QSize(100, 50);

	std::vector<cv::Point> previousPositions;
	std::vector<Task> taskSequence;

	int led_count = 0;
	bool freezeImage = false;

	void loadImages();
	void overlayImage(const cv::Mat &background, const cv::Mat &foreground, cv::Mat &output, cv::Point2i location, double opacity = 1.0);
};
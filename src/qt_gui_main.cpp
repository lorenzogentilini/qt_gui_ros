#include <QApplication>
#include <ControlWindow.hpp>

int main(int argc, char *argv[]){
  // Initialize QT Application
  QApplication qa(argc, argv);

  // Initialize Control Window
  ControlWindow cw(argc, argv);

  return qa.exec();
}
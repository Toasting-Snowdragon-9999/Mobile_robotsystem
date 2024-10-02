#include <iostream>
#include "hello_world.h"
#include <QCoreApplication>
#include <QSoundEffect>

int main(int argc, char** argv){
	HelloWorld hw;
	hw.print();

	QCoreApplication a(argc, argv);

	QSoundEffect dog;
	dog.setSource(QUrl::fromLocalFile("../tester_sarah/dog.wav"));
	dog.setLoopCount(1);
	dog.setVolume(100);

	dog.play()

	return 0;
}
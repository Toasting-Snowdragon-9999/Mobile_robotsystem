#include <iostream>
#include "hello_world.h"
#include <QCoreApplication>
#include <QSoundEffect>
#include <QMediaPlayer>
#include <QApplication>
#include <QUrl>

int main(int argc, char** argv){
	HelloWorld hw;
	hw.print();

	QApplication app(argc, argv);

    QMediaPlayer* player = new QMediaPlayer();

    // Set the media file you want to play
    player->setMedia(QUrl::fromLocalFile("../tester_sarah/dog.wav"));

    // Start playback
    player->play();

    // Check if it's playing
    if (player->state() == QMediaPlayer::PlayingState) {
        return 1;  // Playback is happening
    } else {
        return 0;  // Not playing
    }

    return app.exec();
}
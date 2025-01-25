#include "commandbuttons.h"
#include <mavsdk/mavsdk.h>
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    CommandButtons win;
    win.show();
    return a.exec();
}

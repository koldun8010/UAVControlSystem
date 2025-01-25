#ifndef NOTIFICATION_H
#define NOTIFICATION_H

#include <QWidget>

namespace Ui {
class Notification;
}

class Notification : public QWidget
{
    Q_OBJECT

public:
    explicit Notification(QWidget *parent = nullptr);
    ~Notification();
    void checked(bool checked);

private:
    Ui::Notification *ui;


signals:
    void closeNotification();
private slots:
    void on_okButton_clicked();
};

#endif // NOTIFICATION_H

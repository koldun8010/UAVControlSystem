#include "notification.h"
#include "ui_notification.h"

Notification::Notification(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Notification)
{
    ui->setupUi(this);
}

Notification::~Notification()
{
    delete ui;
}

void Notification::checked(bool checked)
{
    if (checked) {
        ui->messageLabel->setText("Все системы в норме");
        //ui->messageLabel->setAlignment()
    } else {
        //ui->messageLabel->setText("Системы не в норме")
    }
}

void Notification::on_okButton_clicked()
{
    emit(closeNotification());
}


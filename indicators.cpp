#include "indicators.h"
#include "ui_indicators.h"

Indicators::Indicators(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Indicators)
{
    ui->setupUi(this);
}

Indicators::~Indicators()
{
    delete ui;
}

void Indicators::ready()
{
    ui->BINSStateLabel->setStyleSheet(" QLabel { border-radius: 8px; border-style: outset; border-width: 1px; background-color: green; }");
    ui->GNSSStateLabel->setStyleSheet(" QLabel { border-radius: 8px; border-style: outset; border-width: 1px; background-color: green; }");
    ui->OESStateLabel->setStyleSheet(" QLabel { border-radius: 8px; border-style: outset; border-width: 1px; background-color: green; }");
    ui->communStateLabel->setStyleSheet(" QLabel { border-radius: 8px; border-style: outset; border-width: 1px; background-color: green; }");

    ui->batteryProgressBar->setValue(100);
}

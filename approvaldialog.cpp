#include "approvaldialog.h"
#include "ui_approvaldialog.h"

ApprovalDialog::ApprovalDialog(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ApprovalDialog)
{
    ui->setupUi(this);
}

ApprovalDialog::~ApprovalDialog()
{
    delete ui;
}

void ApprovalDialog::on_approvalButtonBox_clicked(QAbstractButton *button)
{
    if (button->text() == "&OK") {
        emit(exec());
    }
    emit(closeDialog());
}


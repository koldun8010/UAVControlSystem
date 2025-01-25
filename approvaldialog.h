#ifndef APPROVALDIALOG_H
#define APPROVALDIALOG_H

#include "qabstractbutton.h"
#include <QWidget>

namespace Ui {
class ApprovalDialog;
}

class ApprovalDialog : public QWidget
{
    Q_OBJECT

public:
    explicit ApprovalDialog(QWidget *parent = nullptr);
    ~ApprovalDialog();

signals:
    void closeDialog();
    void exec();

private slots:
    void on_approvalButtonBox_clicked(QAbstractButton *button);

private:
    Ui::ApprovalDialog *ui;
public:
    bool flag;
};

#endif // APPROVALDIALOG_H

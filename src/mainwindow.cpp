#include "include/mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{
}

MainWindow::~MainWindow()
{
}

void MainWindow::on_actionExit_triggered()
{
    this->close();
}

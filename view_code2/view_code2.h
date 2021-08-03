#pragma once

#include <QtWidgets/QWidget>
#include "ui_view_code2.h"
#include"mythread.h"
#include<qpushbutton.h>
#include<qthread.h>
#include <qserialportinfo.h>
#include<qserialport.h>
#include <qmessagebox.h>
#include <qtooltip.h>
#include<qbytearray.h>
enum class UartState { ON, OFF };
class view_code2 : public QWidget
{
	Q_OBJECT

public:
	view_code2(QWidget *parent = Q_NULLPTR);
	~view_code2();
	void uartInit(void);
	void on_pushButton_uart_sw_clicked();
	void on_pushButton_uart_rfresh_clicked();
	void Senddata(int,int, int);
public slots:
	void receive_data();
private:
	Ui::view_code2Class ui;
	QThread *thread;
	QPushButton pushbutton;
	mythread *vice;
	UartState currentUartState = UartState::OFF;
	QSerialPort *currentSerialPort;
};

#include "view_code2.h"
#include<qpushbutton.h>
#include<qslider.h>
#include<QtSerialPort\qserialportinfo.h>
#include <qserialportinfo.h>
#include<qserialport.h>
#include<qdebug.h>
view_code2::view_code2(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);

	thread = new QThread(this);
	vice = new mythread();
	vice->moveToThread(thread);

	connect(vice, &mythread::kinect_signal, this, &view_code2::on_pushButton_uart_sw_clicked);
	currentSerialPort = new QSerialPort(this);
	uartInit();

	//////////串口发送数据/////////////
	connect(ui.refresh_button, &QPushButton::clicked, this, &view_code2::on_pushButton_uart_rfresh_clicked);
	connect(ui.pushButton_uart_sw, &QPushButton::clicked, this, &view_code2::on_pushButton_uart_sw_clicked);
	connect(vice, &mythread::com_signal, this, &view_code2::Senddata);
	connect(thread, &QThread::started, vice, &mythread::startthread);
	connect(vice, &mythread::find_singal, this,    //传递正在找什么物体
		[=](char judge)
	{
		switch (judge)
		{
		case '1':
			ui.find->setText(u8"正在识别一号球");
			break;
		case '2':
			ui.find->setText(u8"正在识别二号球");
			break;
		case '3':
			ui.find->setText(u8"正在识别三号球");
			break;
		case '4':
			ui.find->setText(u8"正在识别四号球");
			break;
		case '5':
			ui.find->setText(u8"正在识别两个篮球");
			break;
		case '6':
			ui.find->setText(u8"正在识别两个排球");
			break;
		case 'a':
			ui.find->setText(u8"正在识别矩形");
			break;
		default:
			break;
		}

	}
	);

	connect(vice, &mythread::rect_singal, this,   //传递识别矩形数据的槽 传递矩形中心的横坐标,灰色比例,识别矩形的时间
		[=](int out_x, double rate, double rect_time, double area_rect,int sign)
	{
		QString string_rate = QString::number(rate, 10, 2);
		ui.gray_rate_value->setText(string_rate);

		QString string_out_x = QString::number(out_x, 10);
		ui.rect_x_value->setText(string_out_x);

		QString string_rect_time = QString::number(rect_time, 10, 2);
		ui.rect_time_value->setText(string_rect_time);

		QString string_area_rect = QString::number(area_rect, 10, 2);
		ui.rect_area_value->setText(string_area_rect);

		if (sign == 1)
		{
			ui.rect_white_set0_button_2->setStyleSheet("background-color: rgb(255,0,0)");
		}
		else
		{
			ui.rect_white_set0_button_2->setStyleSheet("background-color: rgb(255,255,255)");
		}

	}
	);

	connect(vice, &mythread::ball_singal, this,  //传递找到球的圆心横坐标以及距离
		[=](int point_x, int dist, double ball_time,int upperpart_threshold)
	{
		QString string_point_x = QString::number(point_x, 10);
		ui.circle_x_value->setText(string_point_x);

		QString string_dist = QString::number(dist, 10);
		ui.dist_value->setText(string_dist);

		QString string_ball_time = QString::number(ball_time, 10, 4);
		ui.ball_time_value->setText(string_ball_time);

		QString string_upperpart_threshold = QString::number(upperpart_threshold, 10);
		ui.upperpart_threshold_value->setText(string_upperpart_threshold);
	}
	);

	connect(vice, &mythread::ball1_singal, this,  //传递一号球，红蓝白篮球的各个颜色比例
		[=](double ball1_red, double ball1_blue,double ball1_white)
	{
		QString string_ball1_red = QString::number(ball1_red, 10, 2);
		ui.ball1_rate1_value->setText(string_ball1_red);

		QString string_ball1_blue = QString::number(ball1_blue, 10, 2);
		ui.ball1_rate2_value->setText(string_ball1_blue);

		QString string_ball1_white = QString::number(ball1_white, 10, 2);
		ui.ball1_rate3_value->setText(string_ball1_white);
	}
	);

	connect(vice, &mythread::ball2_signal, this,//传递二号球，黄蓝白篮球的各个颜色比例
		[=](double ball2_yellow, double ball2_blue,double ball2_white)
	{
		QString string_ball2_yellow = QString::number(ball2_yellow, 10, 2);
		ui.ball2_rate1_value->setText(string_ball2_yellow);

		QString string_ball2_blue = QString::number(ball2_blue, 10, 2);
		ui.ball2_rate2_value->setText(string_ball2_blue);

		QString string_ball2_white = QString::number(ball2_white, 10, 2);
		ui.ball2_rate3_value->setText(string_ball2_white);
	}
	);

	connect(vice, &mythread::ball3_signal, this,//传递三号球，灰蓝色篮球的各个颜色比例
		[=](double ball3_gray, double ball3_blue, double ball3_yellow)
	{
		QString string_ball3_gray = QString::number(ball3_gray, 10, 2);
		ui.ball3_rate1_value->setText(string_ball3_gray);

		QString string_ball3_blue = QString::number(ball3_blue, 10, 2);
		ui.ball3_rate2_value->setText(string_ball3_blue);

		QString string_ball3_yellow = QString::number(ball3_yellow, 10, 2);
		ui.ball3_rate3_value->setText(string_ball3_yellow);
	}
	);

	connect(vice, &mythread::ball4_signal, this,//传递四号球，橙色篮球的各个颜色比例
		[=](double ball4_orange)
	{
		QString string_ball4_orange = QString::number(ball4_orange, 10, 2);
		ui.ball4_rate1_value->setText(string_ball4_orange);

	}
	);
	
	///////////////置零函数的信号与槽连接//////////
	connect(ui.blue_set0_button, &QPushButton::clicked, this,//////////蓝色置零/////////
		[=]()
	{
		vice->blue_set();
		ui.blue_reset_button->setStyleSheet("background-color: rgb(255,0,0)");
		ui.blue_set0_button->setStyleSheet("background-color: rgb(255,0,0)");
	}
	);
	connect(ui.blue2_set0_button, &QPushButton::clicked, this,//////////蓝2置零/////////
		[=]()
	{
		vice->blove_set();
		ui.blue2_set0_button->setStyleSheet("background-color: rgb(255,0,0)");
		ui.blue2_reset_button->setStyleSheet("background-color: rgb(255,0,0)");
	}
	);
	connect(ui.gray_set0_button, &QPushButton::clicked, this,//////////灰色置零/////////
		[=]()
	{
		vice->gray_set();
		ui.gray_reset_button->setStyleSheet("background-color: rgb(255,0,0)");
		ui.gray_set0_button->setStyleSheet("background-color: rgb(255,0,0)");
	}
	);
	connect(ui.red1_set0_button, &QPushButton::clicked, this,//////////红1置零/////////
		[=]()
	{
		vice->red1_set();
		ui.red1_reset_button->setStyleSheet("background-color: rgb(255,0,0)");
		ui.red1_set0_button->setStyleSheet("background-color: rgb(255,0,0)");
	}
	);
	connect(ui.red2_set0_button, &QPushButton::clicked, this,//////////红2置零/////////
		[=]()
	{
		vice->red2_set();		
		ui.red2_set0_button->setStyleSheet("background-color: rgb(255,0,0)");
		ui.red2_reset_button->setStyleSheet("background-color: rgb(255,0,0)");
	}
	);
	connect(ui.yellow_set0_button, &QPushButton::clicked, this,//////////黄色置零/////////
		[=]()
	{
		vice->yellow_set();
		ui.yellow_reset_button->setStyleSheet("background-color: rgb(255,0,0)");
		ui.yellow_set0_button->setStyleSheet("background-color: rgb(255,0,0)");
	}
	);
	connect(ui.white_set0_button, &QPushButton::clicked, this,//////////白色置零/////////
		[=]()
	{
		vice->white_set();
		ui.white_reset_button->setStyleSheet("background-color: rgb(255,0,0)");
		ui.white_set0_button->setStyleSheet("background-color: rgb(255,0,0)");
	}
	);
	connect(ui.orange_set0_button, &QPushButton::clicked, this,//////////橙色置零/////////
		[=]()
	{
		vice->orange_set();
		ui.orange_reset_button->setStyleSheet("background-color: rgb(255,0,0)");
		ui.orange_set0_button->setStyleSheet("background-color: rgb(255,0,0)");
	}
	);
	connect(ui.rect_white_set0_button, &QPushButton::clicked, this,//////////矩形白色置零/////////
		[=]()
	{
		vice->rect_white_set();
		ui.rect_white_reset_button->setStyleSheet("background-color: rgb(255,0,0)");
		ui.rect_white_set0_button->setStyleSheet("background-color: rgb(255,0,0)");
	}
	);
	connect(ui.rect_gray_set0_button, &QPushButton::clicked, this,//////////矩形灰色置零/////////
		[=]()
	{
		vice->rect_gray_set();
		ui.rect_gray_reset_button->setStyleSheet("background-color: rgb(255,0,0)");
		ui.rect_gray_set0_button->setStyleSheet("background-color: rgb(255,0,0)");
	}
	);
	connect(ui.lemon_set0_button, &QPushButton::clicked, this,//////////柠檬置零/////////
		[=]()
	{
		vice->lemon_set();
		ui.lemon_reset_button->setStyleSheet("background-color: rgb(255,0,0)");
		ui.lemon_set0_button->setStyleSheet("background-color: rgb(255,0,0)");
	}
	);
	/////////////////////球复位函数的信号与槽连接///////////
	connect(ui.ball1_reset_button, &QPushButton::clicked, this,//////////一号球复位/////////
		[=]()
	{
		vice->ball1_reset();
	}
	);
	connect(ui.ball2_reset_button, &QPushButton::clicked, this,//////////二号球复位/////////
		[=]()
	{
		vice->ball2_reset();
	}
	);
	connect(ui.ball3_reset_button, &QPushButton::clicked, this,//////////三号球复位/////////
		[=]()
	{
		vice->ball3_reset();
	}
	);
	connect(ui.ball4_reset_button, &QPushButton::clicked, this,//////////四号球复位/////////
		[=]()
	{
		vice->ball4_reset();
	}
	);

	//////////////////颜色复位信号与槽连接/////////////////
	connect(ui.blue_reset_button, &QPushButton::clicked, this,//////////蓝色复位/////////
		[=]()
	{
		vice->blue_reset();
		ui.blue_reset_button->setStyleSheet("background-color: rgb(255,255,255)");
		ui.blue_set0_button->setStyleSheet("background-color: rgb(255,255,255)");
	}
	);
	connect(ui.blue2_reset_button, &QPushButton::clicked, this,//////////蓝2复位/////////
		[=]()
	{
		vice->blove_reset();
		ui.blue2_reset_button->setStyleSheet("background-color: rgb(255,255,255)");
		ui.blue2_set0_button->setStyleSheet("background-color: rgb(255,255,255)");
	}
	);
	connect(ui.gray_reset_button, &QPushButton::clicked, this,//////////灰色复位/////////
		[=]()
	{
		vice->gray_reset();
		ui.gray_reset_button->setStyleSheet("background-color: rgb(255,255,255)");
		ui.gray_set0_button->setStyleSheet("background-color: rgb(255,255,255)");
	}
	);
	connect(ui.red1_reset_button, &QPushButton::clicked, this,//////////红1复位/////////
		[=]()
	{
		vice->red1_reset();
		ui.red1_reset_button->setStyleSheet("background-color: rgb(255,255,255)");
		ui.red1_set0_button->setStyleSheet("background-color: rgb(255,255,255)");
	}
	);
	connect(ui.red2_reset_button, &QPushButton::clicked, this,//////////红2复位/////////
		[=]()
	{
		vice->red2_reset();
		ui.red2_reset_button->setStyleSheet("background-color: rgb(255,255,255)");
		ui.red2_set0_button->setStyleSheet("background-color: rgb(255,255,255)");
	}
	);
	connect(ui.yellow_reset_button, &QPushButton::clicked, this,//////////黄色复位/////////
		[=]()
	{
		vice->yellow_reset();
		ui.yellow_reset_button->setStyleSheet("background-color: rgb(255,255,255)");
		ui.yellow_set0_button->setStyleSheet("background-color: rgb(255,255,255)");
	}
	);
	connect(ui.white_reset_button, &QPushButton::clicked, this,//////////白色复位/////////
		[=]()
	{
		vice->white_reset();
		ui.white_reset_button->setStyleSheet("background-color: rgb(255,255,255)");
		ui.white_set0_button->setStyleSheet("background-color: rgb(255,255,255)");
	}
	);
	connect(ui.orange_reset_button, &QPushButton::clicked, this,//////////橙色复位/////////
		[=]()
	{
		vice->orange_reset();
		ui.orange_reset_button->setStyleSheet("background-color: rgb(255,255,255)");
		ui.orange_set0_button->setStyleSheet("background-color: rgb(255,255,255)");
	}
	);
	connect(ui.rect_white_reset_button, &QPushButton::clicked, this,//////////矩形白色复位/////////
		[=]()
	{
		vice->rect_white_reset();
		ui.rect_white_reset_button->setStyleSheet("background-color: rgb(255,255,255)");
		ui.rect_white_set0_button->setStyleSheet("background-color: rgb(255,255,255)");
	}
	);
	connect(ui.rect_gray_reset_button, &QPushButton::clicked, this,//////////矩形灰色复位/////////
		[=]()
	{
		vice->rect_gray_reset();
		ui.rect_gray_reset_button->setStyleSheet("background-color: rgb(255,255,255)");
		ui.rect_gray_set0_button->setStyleSheet("background-color: rgb(255,255,255)");
	}
	);
	connect(ui.lemon_reset_button, &QPushButton::clicked, this,//////////柠檬复位/////////
		[=]()
	{
		vice->lemon_reset();
		ui.lemon_reset_button->setStyleSheet("background-color: rgb(255,255,255)");
		ui.lemon_set0_button->setStyleSheet("background-color: rgb(255,255,255)");
	}
	);
	/////////////球数据的保存///////////////////////////         
	connect(ui.ball1_save_button, &QPushButton::clicked, this,//////////一号球保存/////////
		[=]()
	{
		vice->ball1_save();
	}
	);
	connect(ui.ball2_save_button, &QPushButton::clicked, this,//////////二号球保存/////////
		[=]()
	{
		vice->ball2_save();
	}
	);
	connect(ui.ball3_save_button, &QPushButton::clicked, this,//////////三号球保存/////////
		[=]()
	{
		vice->ball3_save();
	}
	);
	connect(ui.ball4_save_button, &QPushButton::clicked, this,//////////四号球保存/////////
		[=]()
	{
		vice->ball4_save();
	}
	);
	connect(ui.rect_white_save_button, &QPushButton::clicked, this,//////////矩形灰色数据保存/////////
		[=]()
	{
		vice->rect_white_save();
	}
	);
	connect(ui.rect_gray_save_button, &QPushButton::clicked, this,//////////矩形灰色数据保存/////////
		[=]()
	{
		vice->rect_gray_save();
	}
	);

	connect(ui.all_save_button, &QPushButton::clicked, this,//////////矩形灰色数据保存/////////
		[=]()
	{
		vice->all_save();
	}
	);
	//////////////////颜色暂时保存信号与槽连接/////////////////
	connect(ui.blue_tem_button, &QPushButton::clicked, this,//////////蓝色复位/////////
		[=]()
	{
		vice->blue_tem();
	}
	);
	connect(ui.blue2_tem_button, &QPushButton::clicked, this,//////////蓝2复位/////////
		[=]()
	{
		vice->blove_tem();
	}
	);
	connect(ui.gray_tem_button, &QPushButton::clicked, this,//////////灰色复位/////////
		[=]()
	{
		vice->gray_tem();
	}
	);
	connect(ui.red1_tem_button, &QPushButton::clicked, this,//////////红1复位/////////
		[=]()
	{
		vice->red1_tem();
	}
	);
	connect(ui.red2_tem_button, &QPushButton::clicked, this,//////////红2复位/////////
		[=]()
	{
		vice->red2_tem();
	}
	);
	connect(ui.yellow_tem_button, &QPushButton::clicked, this,//////////黄色复位/////////
		[=]()
	{
		vice->yellow_tem();
	}
	);
	connect(ui.white_tem_button, &QPushButton::clicked, this,//////////白色复位/////////
		[=]()
	{
		vice->white_tem();
	}
	);
	connect(ui.orange_tem_button, &QPushButton::clicked, this,//////////橙色复位/////////
		[=]()
	{
		vice->orange_tem();
	}
	);
	connect(ui.rect_white_tem_button, &QPushButton::clicked, this,//////////矩形白色复位/////////
		[=]()
	{
		vice->rect_white_tem();
	}
	);
	connect(ui.rect_gray_tem_button, &QPushButton::clicked, this,//////////矩形灰色复位/////////
		[=]()
	{
		vice->rect_gray_tem();
	}
	);
	connect(ui.lemon_tem_button, &QPushButton::clicked, this,//////////柠檬复位/////////
		[=]()
	{
		vice->lemon_tem();
	}
	);
	//窗口关闭后停止线程
	connect(this, &QObject::destroyed, this, [=]() {
		vice->mythreadUartState = threadUartState::OFF;
		vice->KinectState = KinectUartState::OFF;
		if (thread->isRunning())
		{			
			thread->quit();
			thread->wait();
		}});

	
}

view_code2::~view_code2()
{

}
void view_code2::uartInit(void)
{
	currentUartState = UartState::OFF;
	//radarUartState = UartState::OFF;
	/* 所有当前可用的串口 */
	ui.comboBox_com->clear();
	foreach(auto const &info, QSerialPortInfo::availablePorts())
	{
		ui.comboBox_com->addItem(info.portName() + ": " + info.description());
	//	ui.comboBox_com_radar->addItem(info.portName() + ": " + info.description());
	}

	ui.comboBox_com->setCurrentIndex(0);
	ui.comboBox_com->setToolTip(ui.comboBox_com->currentText());

	//ui.comboBox_com_radar->setCurrentIndex(0);
	//ui.comboBox_com_radar->setToolTip(ui.comboBox_com->currentText());
}
void view_code2::on_pushButton_uart_sw_clicked()
{
	if (currentUartState == UartState::ON)
	{
		currentSerialPort->close();
		currentUartState = UartState::OFF;
		vice->mythreadUartState = threadUartState::OFF;
		vice->KinectState = KinectUartState::OFF;
		ui.pushButton_uart_sw->setText(QString::fromUtf8(u8"启动"));

		if (thread->isRunning())
		{
			thread->quit();
			thread->wait();
		}

		ui.comboBox_com->setEnabled(true);
	}
	else
	{
		currentSerialPort->setPortName(ui.comboBox_com->currentText().split(':').at(0));
		currentSerialPort->setBaudRate(115200);
		//默认参数设置
		currentSerialPort->setFlowControl(QSerialPort::NoFlowControl);// 无流控制
		currentSerialPort->setDataBits(QSerialPort::Data8);//数据为 8
		currentSerialPort->setStopBits(QSerialPort::OneStop);//停止位一位
		currentSerialPort->setParity(QSerialPort::NoParity);//无校验位
		currentSerialPort->setReadBufferSize(10); //接收缓存10个字节

		if (currentSerialPort->open(QSerialPort::ReadWrite))
		{
			currentUartState = UartState::ON;
			vice->mythreadUartState = threadUartState::ON;
			vice->KinectState = KinectUartState::ON;
			////线程开始
			thread->start();
			//连接槽信号 接收
			connect(currentSerialPort, &QSerialPort::readyRead, this, &view_code2::receive_data);
			ui.pushButton_uart_sw->setText(u8"关闭串口");
			ui.comboBox_com->setEnabled(false);
		//	ui.comboBox_baud->setEnabled(false);
		}
		else {
			QMessageBox::critical(this, tr("Error"), tr("Fail to turn on this device!"));
		}
	}
}
//串口信息刷新
void view_code2::on_pushButton_uart_rfresh_clicked()
{
	if (currentUartState == UartState::ON )
		QToolTip::showText(QCursor::pos(), tr("Please turn off the current device first."));
	else
		uartInit();
}
void view_code2::Senddata(int data1, int data2,int data3)
{
	char sendData[10];
	sendData[0] = '@';
	sendData[1] = '^';
	sendData[2] = 'v';
	sendData[3] = (data1 >> 8) & 0xff;
	sendData[4] = data1 & 0xff;
	sendData[5] = (data2>>8) & 0xff;
	sendData[6] = data2 & 0xff;
	sendData[7] = (data3>>8) & 0xff;
	sendData[8] = data3 & 0xff;
	sendData[9] = 0;
	for (int i = 0; i < 9; i++)
		sendData[9] += sendData[i];
	qDebug() << int(sendData[9]) << endl;
	currentSerialPort->write(sendData, 10);
}
void view_code2::receive_data()
{
	QByteArray buf;
	while (currentSerialPort->bytesAvailable())
	{
		buf = currentSerialPort->read(1);
		if (buf.at(0) == '1' || buf.at(0) == '2' || buf.at(0) == '3' || buf.at(0) == '4' || buf.at(0) == '5' || buf.at(0) == '6' || buf.at(0) == '7' || buf.at(0) == '8' || buf.at(0) == 'a')
		{
			vice->com_data = buf.at(0);
		}

		
	}

}
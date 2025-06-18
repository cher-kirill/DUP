#include <QApplication>
#include <QMainWindow>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <QTimer>
#include <QtCharts>
#include <QByteArray>
#include <QDebug>
#include <QElapsedTimer>
#include <QVector>
#include <QPair>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QGroupBox>
#include <QStackedWidget>
#include <QLineEdit>
#include <QThread>
#include <QCheckBox>
#include <QMutex>
#include <QWaitCondition>
#include <stdio.h>
#include <stdlib.h>
#include <chai.h>
#include <QWheelEvent>
#include <QtCharts/QChartView>
#include <QSharedMemory>
#include <QMessageBox>
#include <QCoreApplication>

void customMessageHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg) {

    QDir().mkpath("logs");  // Создаст папку, если нет
    static QFile logFile(QString("logs/log_%1.txt").arg(
        QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss")
    ));

    if (!logFile.isOpen()) {
        logFile.open(QIODevice::Append | QIODevice::Text);
    }

    QTextStream out(&logFile);
    QString time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");
    QString typeStr;

    switch (type) {
    case QtDebugMsg:    typeStr = "DEBUG"; break;
    case QtInfoMsg:     typeStr = "INFO"; break;
    case QtWarningMsg:  typeStr = "WARNING"; break;
    case QtCriticalMsg: typeStr = "CRITICAL"; break;
    case QtFatalMsg:    typeStr = "FATAL"; break;
    }

    out << time << " [" << typeStr << "] " << msg << Qt::endl;
    out.flush();

    if (type == QtFatalMsg)
        abort();
}

class WorkerThread : public QThread {
    Q_OBJECT

signals:
    void valueProcessed(int value);
    void dataUpdated(int32_t positionServo, int32_t speedServo, int32_t positionDup, uint64_t timeStamp);
    void errorSend(QString errorMsg);

public slots:

    void disconnectDup(){

        CiStop(channelDup);
        msleep(100);
        CiClose(channelDup);
        msleep(100);
        stateConnectDup = false;
    }

    void disconnectServo(){

        CiStop(channelServo);
        msleep(100);
        CiClose(channelServo);
        msleep(100);
        stateConnectServo = false;
    }

    void zeroDup() { // Слот для получения данных из основного потока

        if(commandDup == ""){

            commandDup = "zero";

        }

        // Обработка данных
    }

    void zeroServo(){

        if(commandServo == ""){

            commandServo = "zero";

        }

    }

    void startMoveStaticModeServo(float speed, float angle){
        if(commandServo == ""){

            commandServo = "start_move_static_mode";
            commandSpeedServo = speed;
            commandAngleServo = angle;
            //qDebug() << speed;

        }
    }

    void startMoveDynamicModeServo(float speed, float rangeMin, float rangeMax){

        if(commandServo == ""){

            commandServo = "start_move_dynamic_mode";
            commandSpeedServo = speed;
            commandAngleServo = rangeMin;
            commandRangeMinServo = rangeMin;
            commandRangeMaxServo = rangeMax;


        }
    }

    void startMoveSpeedModeServo(float speed){

        if(commandServo == ""){

            commandServo = "start_move_speed_mode";
            commandSpeedServo = speed;
            //qDebug() << "speed";

        }
    }

    void stopServo(){

        if(commandServo == ""){

            commandServo = "stop";

        }

    }

public:

    bool stateConnectServo = false;
    bool stateConnectDup = false;
    bool flagMove = false;
    bool dirMove = false;
    uint8_t channelDup = 2;
    uint8_t channelServo = 2;
    float offsetBacklashServo = 0.00;

    WorkerThread(QObject *parent = nullptr) : QThread(parent) {}

    void run() override {

        int ret;
        //channelDup = 0;
        //channelServo = 1;

        //  open channel

        // main loop
        while (running) {

            //qDebug() << "time" ;
            time_ms = elapsedTimer.elapsed();

            if(stateConnectDup){

                if((getDataDup() == 0)){

                    time_ms = elapsedTimer.elapsed();
                    //qDebug() << "time" << time_ms;
                    //msleep(10);
                    if(stateConnectServo){

                        if((ret = getDataServo()) == 0){

                            //qDebug() << "Speed" << Qt::hex << speedServo << "Pos servo" << positionServo << "Pos Dup" << positionDup;
                            emit dataUpdated(positionServo, speedServo, positionDup, time_ms); // Emit signal

                        }
                        else {
                            //qDebug() << ret << "error servo";
                        }

                        if(commandServo != ""){

                            if(commandServo == "start_move_static_mode"){
                                sendCommandStartStaticModeServo();
                                //qDebug() << "обработчик";
                                commandServo = "";

                            } else if (commandServo == "start_move_dynamic_mode"){
                                sendCommandStartDynamicModeServo();
                                commandServo = "";

                            } else if (commandServo == "start_move_speed_mode"){
                                sendCommandStartSpeedModeServo();
                                commandServo = "";

                            } else if (commandServo == "stop"){
                                sendCommandStopServo();
                                commandServo = "";

                            } else if (commandServo == "zero"){
                                setZeroServo();
                                commandServo = "";
                            }

                        }

                        if(flagMove){

                            checkMoveServo();

                        }


                    }


                    if(commandDup != "") {
                        if(commandDup == "zero"){

                            sendCommandZeroDup();
                            commandDup = "";

                        }
                    }
                } //else qDebug() << "error Dup";
            }
        }

        CiClose(channelServo);
        CiClose(channelDup);

    }

    void connectDup(){

        int ret;

        if(!stateConnectServo){

            if ((ret = CiInit()) < 0) {
                qDebug() << "No Init" << ret;

            }
        }

        if ((ret = CiOpen(channelDup, CIO_CAN11 | CIO_CAN29)) < 0) {
            qDebug() << "No Open" << ret;

        }

        // set baud rate to 500Kbit
        if ((ret = CiSetBaud(channelDup, BCI_500K)) < 0) {
            qDebug() << "No speed" << ret;
            CiClose(channelDup);
        }

        if(CiStart(channelDup) == 0){
            elapsedTimer.start();
            stateConnectDup = true;
        }
    }

    void connectServo(){

        int ret;

        if(!stateConnectDup){

            if ((ret = CiInit()) < 0) {
                qDebug() << "No Init" << ret;

            }
        }

        if ((ret = CiOpen(channelServo, CIO_CAN11 | CIO_CAN29)) < 0) {
            qDebug() << "No Open" << ret;

        }

        // set baud rate to 500Kbit
        if ((ret = CiSetBaud(channelServo, BCI_500K)) < 0) {
            qDebug() << "No speed" << ret;
            CiClose(channelServo);
        }



        if(CiStart(channelServo) == 0){
            //elapsedTimer.start();
            stateConnectServo = true;
        }
        else qDebug() << "No start";

        uint16_t rsCnt = 0;



        msleep(100);

        CiRcQueCancel(channelServo, &rsCnt);

         msleep(100);

        if((ret = getDataServo()) != 0){
            stateConnectServo = false;
            CiClose(channelServo);
            qDebug() << "No start";
            emit errorSend("Не удалось установить подключение к сервоконтроллеру");

        }


    }

    void getServoSpeed(){

    }

    void getServoPosition(){

    }

    void getDupPosition(){

    }

    int sendCommandStartStaticModeServo(){


        if(setModePositionServo() != 0) return -1;

        if(sendEnableServo() != 0) return -1;
        if(sendSpeedInPositionServo() != 0) return -1;
        if(sendPositionServo() != 0) return -1;
        if(sendCommandGoToPosition() != 0) return -1;
        //qDebug() <<  commandServo;
        return 0;
    }

    int sendCommandStartDynamicModeServo(){

        if(setModePositionServo() != 0) return -1;
        if(sendEnableServo() != 0) return -1;
        if(sendSpeedInPositionServo() != 0) return -1;
        if(sendPositionServo() != 0) return -1;
        if(sendCommandGoToPosition() != 0) return -1;
        flagMove = true;
        return 0;
    }

    int sendCommandStartSpeedModeServo(){
        if(setModeSpeedServo() != 0) return -1;
        if(sendEnableServo() != 0) return -1;
        if(sendSpeedInSpeedServo() != 0) return -1;

        return 0;
    }

    int sendCommandStopServo(){

        if(sendDisableServo() != 0) return -1;
        flagMove = false;
        return 0;
    }

    void checkMoveServo(){

        if(getControlWord() > 0){

            if(getControlWord() & 0x400){

                if(commandRangeMinServo != 0 && commandRangeMaxServo != 0){
                    //qDebug() << commandRangeMinServo;
                    //qDebug() << float(positionServo) / 7065.6;
                    if( std::abs(float(positionServo) / 7065.6 - commandRangeMinServo) < 1){
                        //qDebug() << "range1";
                        commandAngleServo = commandRangeMaxServo;
                        sendEnableServo();
                        sendSpeedInPositionServo();
                        sendPositionServo();
                        sendCommandGoToPosition();
                    }

                    if( std::abs(float(positionServo) / 7065.6 - commandRangeMaxServo) < 1){
                        //qDebug() << "range2";
                        commandAngleServo = commandRangeMinServo;
                        sendEnableServo();
                        sendSpeedInPositionServo();
                        sendPositionServo();
                        sendCommandGoToPosition();
                    }
                }
            }
        }
    }

    int getControlWord(){

        canmsg_t frame;
        canwait_t servoWait;
        int ret;

        servoWait.chan = channelServo;
        servoWait.wflags = CI_WAIT_RC | CI_WAIT_ER;

        frame.id = 0x608;
        frame.len = 8;
        frame.data[0] = 0x40;
        frame.data[1] = 0x41;
        frame.data[2] = 0x60;
        frame.data[3] = 0x00;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        //frame.data = {0x40, 0x64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
        frame.flags = 0;

        if ((ret = CiTransmit(channelServo, &frame)) != 0) return -1;

        ret = CiWaitEvent(&servoWait, 1, 100);

        if (ret > 0) {
            if (servoWait.rflags & CI_WAIT_RC) {
                ret = CiRead(channelServo, &frame, 1);
                    if (ret == 1) {
                        //qDebug() << "control word" << Qt::hex << frame.data[0] << frame.data[1] << frame.data[2] << frame.data[3] << frame.data[4] << frame.data[5] << frame.data[6] << frame.data[7];
                        if(frame.id == 0x588 && frame.data[1] == 0x41){
                            return frame.data[4] | (frame.data[5] << 8) | (frame.data[6] << 16) | (frame.data[7] << 24);
                        } else return -5;
                    } else return -2;
                }
        } else return -3;
        return -1;
    }

    int sendDisableServo(){

        return sendDataServo(0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00);

    }

    int sendEnableServo(){

        return sendDataServo(0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00);


    }

    int setModeSpeedServo(){

        return sendDataServo(0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00);

    }

    int setModePositionServo(){

        return sendDataServo(0x2F, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00);

    }

    int sendSpeedInPositionServo(){

        int32_t hexSpeedServo = int32_t(commandSpeedServo * 19.40625);
        return sendDataServo(0x23, 0x81, 0x60, 0x00, hexSpeedServo & 0xFF, hexSpeedServo >> 8 & 0xFF, hexSpeedServo >> 16 & 0xFF, hexSpeedServo >> 24 & 0xFF);




    }

    int sendSpeedInSpeedServo(){

        int32_t hexSpeedServo = int32_t(commandSpeedServo * 19.40625);
        return sendDataServo(0x23, 0xFF, 0x60, 0x00, hexSpeedServo & 0xFF, hexSpeedServo >> 8 & 0xFF, hexSpeedServo >> 16 & 0xFF, hexSpeedServo >> 24 & 0xFF);

    }

    int sendPositionServo(){

        if((commandAngleServo - (positionServo / 7065.6)) < 0) dirMove = false;
        else dirMove = true;

        int32_t hexPositionServo = int32_t(commandAngleServo * 7065.6) + offsetPositionServo;
        return sendDataServo(0x23, 0x7A, 0x60, 0x00, hexPositionServo & 0xFF, hexPositionServo >> 8 & 0xFF, hexPositionServo >> 16 & 0xFF, hexPositionServo >> 24 & 0xFF);


    }

    int sendCommandGoToPosition(){

        return sendDataServo(0x2B, 0x40, 0x60, 0x00, 0x1F, 0x00, 0x00, 0x00);

    }

    void setZeroServo(){

        QString appDirectory = QCoreApplication::applicationDirPath();
        //qDebug() << "WorkerThread: Программа запущена из директории:" << appDirectory;

        offsetBacklashServo = readOffsetBacklash(appDirectory + "/config.txt");

        //qDebug() << "offsetBacklashServo" << offsetBacklashServo;

        commandAngleServo = positionServo / 7065.6 - 30;
        commandSpeedServo = 50;
        sendCommandStartStaticModeServo();

        msleep(1000);


        commandAngleServo += 30;
        sendCommandStartStaticModeServo();

        msleep(1000);

        getDataServo();

        //qDebug() << "positionServo" << positionServo / 7065.6;

        offsetPositionServo += positionServo;

        dirMove = true;

    }

    float readOffsetBacklash(const QString &filePath) {
        QFile file(filePath);

        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            qDebug() << "Не удалось открыть файл:" << filePath;
            return 0;
        }

        QTextStream in(&file);
        float offsetBacklash = 0.0;

        QString line = in.readLine().trimmed();

            //qDebug() << line;
            // Проверяем, начинается ли строка с "offset_backlash"
            if (line.startsWith("offset_backlash")) {
                // Разбиваем строку по символу "="
                QStringList parts = line.split("=");
                if (parts.size() >= 2) {
                    // Вторая часть может содержать комментарий, поэтому обрежем его по символу "//"
                    QString valuePart = parts[1].split("//").front().trimmed();
                    offsetBacklash = valuePart.toDouble();

                }
            }

        file.close();
        return offsetBacklash;
    }

    int sendCommandZeroDup(){

        canmsg_t frame;
        canmsg_t frame2;
        int ret;
        canwait_t dupWait;


        dupWait.chan = channelDup;
        dupWait.wflags = CI_WAIT_RC | CI_WAIT_ER;

        frame.id = 0x18EFE40B;
        frame.len = 8;
        frame.data[0] = 0xFF;
        frame.data[1] = 0xFF;
        frame.data[2] = 0xAA;
        frame.data[3] = 0xFF;
        frame.data[4] = 0xAA;
        frame.data[5] = 0x55;
        frame.data[6] = 0xFF;
        frame.data[7] = 0x4F;
        //frame.data = {0x40, 0x64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
        frame.flags = 4;

        if ((ret = CiTransmit(channelDup, &frame)) != 0) return -1;

        ret = CiWaitEvent(&dupWait, 1, 250);
        //qDebug() << ret;
        ret = CiRead(channelDup, &frame2, 1);
        //qDebug() << Qt::hex << frame2.data[0] << frame2.data[1] << frame2.data[2] << frame2.data[3] << frame2.data[4] << frame2.data[5] << frame2.data[6] << frame2.data[7];
        while(frame2.data[0] != 0xFF && frame2.data[1] != 0xFF){
            ret = CiWaitEvent(&dupWait, 1, 250);
            //qDebug() << ret;
            ret = CiRead(channelDup, &frame2, 1);
            //qDebug() << Qt::hex << frame2.data[0] << frame2.data[1] << frame2.data[2] << frame2.data[3] << frame2.data[4] << frame2.data[5] << frame2.data[6] << frame2.data[7];
        }

        msleep(100);
        if (ret > 0){

            frame.data[4] = 0x0F;
            frame.data[5] = 0xF0;

            if ((ret = CiTransmit(channelDup, &frame)) != 0) return -1;
            //qDebug() << commandDup;
            //qDebug() << Qt::hex << frame.data[0] << frame.data[1] << frame.data[2] << frame.data[3] << frame.data[4] << frame.data[5] << frame.data[6] << frame.data[7];
            ret = CiWaitEvent(&dupWait, 1, 250);
        }

        //



        return 0;

    }

    int getDataServo(){

        canmsg_t frame;
        canwait_t servoWait;
        int ret;

        servoWait.chan = channelServo;
        servoWait.wflags = CI_WAIT_RC | CI_WAIT_ER;

        frame.id = 0x608;
        frame.len = 8;
        frame.data[0] = 0x40;
        frame.data[1] = 0x64;
        frame.data[2] = 0x60;
        frame.data[3] = 0x00;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        //frame.data = {0x40, 0x64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
        frame.flags = 0;

        if ((ret = CiTransmit(channelServo, &frame)) != 0) return -1;

        ret = CiWaitEvent(&servoWait, 1, 100);

        if (ret > 0) {
            if (servoWait.rflags & CI_WAIT_RC) {
                ret = CiRead(channelServo, &frame, 1);
                    if (ret == 1) {
                        //qDebug() << "pos" << Qt::hex << frame.data[0] << frame.data[1] << frame.data[2] << frame.data[3] << frame.data[4] << frame.data[5] << frame.data[6] << frame.data[7];
                        if(frame.id == 0x588 && frame.data[1] == 0x64){
                            positionServo = frame.data[4] | (frame.data[5] << 8) | (frame.data[6] << 16) | (frame.data[7] << 24);
                            positionServo -= offsetPositionServo;
                        } else return -1;
                    } else return -2;
                }
        } else return -3;

        frame.id = 0x608;
        frame.len = 8;
        frame.data[0] = 0x40;
        frame.data[1] = 0x6C;
        frame.data[2] = 0x60;
        frame.data[3] = 0x00;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;

        //frame.data = {0x40, 0x6C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};

        if ((ret = CiTransmit(channelServo, &frame)) != 0) return -1;

        ret = CiWaitEvent(&servoWait, 1, 100);

        if (ret > 0) {
            if (servoWait.rflags & CI_WAIT_RC) {
                ret = CiRead(channelServo, &frame, 1);
                    if (ret == 1) {
                        if(frame.id == 0x588 && frame.data[1] == 0x6C){
                            speedServo = frame.data[4] | (frame.data[5] << 8) | (frame.data[6] << 16) | (frame.data[7] << 24);

                            return 0;
                        } else return -4;
                    } else return -5;
                }
        } else return -6;
        return -7;
    }

    int getDataDup(){

        canmsg_t frame;
        canwait_t dupWait;
        int ret;

        dupWait.chan = channelDup;
        dupWait.wflags = CI_WAIT_RC | CI_WAIT_ER;

        ret = CiWaitEvent(&dupWait, 1, 100);

        if (ret > 0) {
            if (dupWait.rflags & CI_WAIT_RC) {
                ret = CiRead(channelDup, &frame, 1);
                if (ret == 1) {
                    positionDup = frame.data[0] | (frame.data[1] << 8);

                    return 0;
                } else return -5;
            } else return -1;

        } else return -2;

        return -6;
    }

    int sendDataServo(uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7){
        canmsg_t frame_send;
        canmsg_t frame_answer;
        canwait_t servoWait;
        int ret;

        servoWait.chan = channelServo;
        servoWait.wflags = CI_WAIT_TR | CI_WAIT_ER;

        frame_send.id = 0x608;
        frame_send.len = 8;
        frame_send.data[0] = data0;
        frame_send.data[1] = data1;
        frame_send.data[2] = data2;
        frame_send.data[3] = data3;
        frame_send.data[4] = data4;
        frame_send.data[5] = data5;
        frame_send.data[6] = data6;
        frame_send.data[7] = data7;
        //frame.data = {0x40, 0x64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
        frame_send.flags = 0;


        qDebug() << "Send command data:" << Qt::hex << frame_send.data[0] << frame_send.data[1] << frame_send.data[2] << frame_send.data[3] << frame_send.data[4] << frame_send.data[5] << frame_send.data[6] << frame_send.data[7];

        if ((ret = CiTransmit(channelServo, &frame_send)) != 0) {

            qDebug() << "error CiTransmit" << ret;
            return -1;

        }

        //msleep(2);
        if((ret = CiWaitEvent(&servoWait, 1, 100)) <= 0){

            qDebug() << "error CiWaitEvent TR" << ret;
            return -1;
        }



        servoWait.wflags = CI_WAIT_RC | CI_WAIT_ER;

        if((ret = CiWaitEvent(&servoWait, 1, 100)) <= 0){

            qDebug() << "error CiWaitEvent RC" << ret;
            return -1;
        }

        if (ret > 0) {

            if (servoWait.rflags & CI_WAIT_RC) {

                ret = CiRead(channelServo, &frame_answer, 1);
                qDebug() << "Answer command data:" << Qt::hex << frame_answer.data[0] << frame_answer.data[1] << frame_answer.data[2] << frame_answer.data[3] << frame_answer.data[4] << frame_answer.data[5] << frame_answer.data[6] << frame_answer.data[7];
                    if (ret == 1) {
                        if(frame_answer.id == 0x588 && frame_answer.data[0] == 0x60){

                            return 0;

                        } else return -5;
                    } else return -2;
                }
        } else return -3;


    return 0;
    }



    void stop() {

        if(stateConnectServo){
            sendDisableServo();
            msleep(100);
            CiStop(channelServo);
            msleep(100);
            CiClose(channelServo);
            msleep(100);
            stateConnectServo = false;
        }
        if(stateConnectDup){
            CiStop(channelDup);
            msleep(100);
            CiClose(channelDup);
            msleep(100);
            stateConnectDup = false;
        }


        running = false;
    }

private:

    QMutex mutex;
    bool running = true;

    QElapsedTimer serialTimer;
    int32_t speedServo = 0;
    int32_t positionServo = 0;
    int32_t offsetPositionServo = 0;
    int32_t positionDup = 0;
    uint64_t time_ms = 0;
    QString commandDup = "";
    QString commandServo = "";
    QElapsedTimer elapsedTimer;
    float commandSpeedServo = 0;
    float commandAngleServo = 0;
    float commandRangeMinServo = 0;
    float commandRangeMaxServo = 0;
    //enum class commands {
    //    SetPositionServo;
    //    SetSpeedToPosition;
    //    SetSpeedToSpeed;
    //    SetZeroDup;
    //}

};

class CustomChartView : public QChartView {
    Q_OBJECT
public:
    CustomChartView(QChart *chart,
                    QWidget *parent = nullptr,
                    QVector<CustomChartView *> *allCharts = nullptr,
                    QLineSeries *positionSeries = nullptr,
                    QLineSeries *dupSeries = nullptr,
                    QLineSeries *speedSeries = nullptr,
                    QLineSeries *errorSeries = nullptr)
        : QChartView(chart, parent),
          capturing(false),
          allCharts(allCharts),
          isDragging(false),
          hasHoverLine(false),
          hoverX(0),
          positionSeries(positionSeries),
          dupSeries(dupSeries),
          speedSeries(speedSeries),
          errorSeries(errorSeries)
    {
        setMouseTracking(true);
        viewport()->setMouseTracking(true);
        tooltip = new QLabel(nullptr); // отдельное окно
        tooltip->setWindowFlags(Qt::ToolTip);
        tooltip->setStyleSheet("QLabel { background: #efefef; border: 1px solid gray; }");
        tooltip->hide();
    }

    void setCapturing(bool c) {
        capturing = c;
        if (allCharts) {
            for (auto *view : *allCharts) {
                view->hasHoverLine = false;
                if (view->tooltip) view->tooltip->hide();
                view->viewport()->update();
            }
        }
    }

    void setHoverX(qreal x) {
        hoverX = x;
        hasHoverLine = true;
        viewport()->update();
    }

signals:
    void zoomResetSignal(); // Add this signal

protected:
    void toggleGrid() {
        for (QAbstractAxis *axis : chart()->axes()) {
            QValueAxis *valueAxis = qobject_cast<QValueAxis *>(axis);
            if (valueAxis) {
                valueAxis->setGridLineVisible(!valueAxis->isGridLineVisible());
            }
        }
    }

    void changeGraphColor() {
        QColor color = QColorDialog::getColor(Qt::white, this, "Выберите цвет графика");
        if (color.isValid()) {
            for (QAbstractSeries *series : chart()->series()) {
                QLineSeries *lineSeries = qobject_cast<QLineSeries *>(series);
                if (lineSeries) {
                    lineSeries->setColor(color);
                }
            }
        }
    }

    void resetZoom() {
        chart()->zoomReset();
        emit zoomResetSignal(); // Now declared
    }

    void showGraphSettingsMenu(const QPoint &globalPos) {
        QMenu menu;
        QAction *toggleGridAction = menu.addAction("Показать/Скрыть сетку");
        QAction *changeColorAction = menu.addAction("Изменить цвет графика");
        QAction *resetZoomAction = menu.addAction("Сбросить масштаб");

        QAction *selectedAction = menu.exec(globalPos);
        if (selectedAction == toggleGridAction) {
            toggleGrid();
        } else if (selectedAction == changeColorAction) {
            changeGraphColor();
        } else if (selectedAction == resetZoomAction) {
            resetZoom();
        }
    }

    void mousePressEvent(QMouseEvent *event) override {
        if (event->button() == Qt::LeftButton) {
            isDragging = true;
            lastMousePos = event->pos();
        } else if (event->button() == Qt::RightButton) {
            if (chart()->plotArea().contains(event->pos())) {
                showGraphSettingsMenu(event->globalPos());
                return;
            }
        }
        QChartView::mousePressEvent(event);
    }

    void mouseMoveEvent(QMouseEvent *event) override {
        if (capturing && allCharts) {
            qreal x = chart()->mapToValue(event->pos()).x();
            for (auto *view : *allCharts) {
                view->setHoverX(x);
                if (view == this) {
                    QString tipText = view->getTooltipText(x);
                    QPoint tooltipPos = this->viewport()->mapToGlobal(event->pos()) + QPoint(20, 20);
                    view->tooltip->setText(tipText);
                    view->tooltip->adjustSize();
                    view->tooltip->move(tooltipPos);
                    view->tooltip->show();
                } else {
                    if (view->tooltip) view->tooltip->hide();
                }
            }
        } else {
            if (tooltip) tooltip->hide();
        }
        if (isDragging) {
            QPointF delta = chart()->mapToValue(lastMousePos) - chart()->mapToValue(event->pos());
            lastMousePos = event->pos();
            QValueAxis *axisX = qobject_cast<QValueAxis *>(chart()->axes(Qt::Horizontal).first());
            QValueAxis *axisY = qobject_cast<QValueAxis *>(chart()->axes(Qt::Vertical).first());
            if (axisX && axisY) {
                axisX->setRange(axisX->min() + delta.x(), axisX->max() + delta.x());
                axisY->setRange(axisY->min() + delta.y(), axisY->max() + delta.y());
            }
            if (allCharts) {
                for (CustomChartView *other : *allCharts) {
                    if (other != this) {
                        QValueAxis *otherX = qobject_cast<QValueAxis *>(other->chart()->axes(Qt::Horizontal).first());
                        if (otherX) {
                            otherX->setRange(otherX->min() + delta.x(), otherX->max() + delta.x());
                        }
                    }
                }
            }
        }
        QChartView::mouseMoveEvent(event);
    }

    void mouseReleaseEvent(QMouseEvent *event) override {
        if (event->button() == Qt::LeftButton) {
            isDragging = false;
        }
        QChartView::mouseReleaseEvent(event);
    }

    void wheelEvent(QWheelEvent *event) override {
        if (capturing) {
            qreal factor = (event->angleDelta().y() > 0) ? 1.25 : 0.8;
            chart()->zoom(factor);
            if (allCharts) {
                for (CustomChartView *other : *allCharts) {
                    if (other != this) {
                        QValueAxis *x = qobject_cast<QValueAxis *>(other->chart()->axes(Qt::Horizontal).first());
                        if (x) {
                            qreal center = x->min() + (x->max() - x->min()) / 2.0;
                            qreal range = (x->max() - x->min()) / factor;
                            x->setRange(center - range / 2.0, center + range / 2.0);
                        }
                    }
                }
            }
            event->accept();
        } else {
            QChartView::wheelEvent(event);
        }
    }

    void leaveEvent(QEvent *event) override {
        if (tooltip) tooltip->hide();
        hasHoverLine = false;
        viewport()->update();
        QChartView::leaveEvent(event);
    }

    void drawForeground(QPainter *painter, const QRectF &rect) override {
        QChartView::drawForeground(painter, rect);
        if (capturing && hasHoverLine && !chart()->series().isEmpty()) {
            QPen pen(Qt::DashLine);
            pen.setColor(Qt::gray);
            painter->setPen(pen);
            auto *series = chart()->series().first();
            QPointF pt = chart()->mapToPosition(QPointF(hoverX, 0), series);
            painter->drawLine(QPointF(pt.x(), rect.top()),
                              QPointF(pt.x(), rect.bottom()));
        }
    }

    // Эта функция возвращает значения на заданной x-координате для всех серий
    QString getTooltipText(qreal x) {
        // Если серия не существует или пуста — выводим 0
        //qDebug() << "moveee";
        auto pointOrZero = [](QLineSeries* s, qreal x) {
            if (!s || s->count() == 0) return 0.0;
            const auto &points = s->pointsVector();
            auto it = std::min_element(points.begin(), points.end(),
                [x](const QPointF &a, const QPointF &b) {
                    return std::abs(a.x() - x) < std::abs(b.x() - x);
                });
            return (it != points.end()) ? it->y() : 0.0;
        };

        double pos = pointOrZero(positionSeries, x);



        double dup = pointOrZero(dupSeries, x);



        double speed = pointOrZero(speedSeries, x);

        //qDebug() << "poveee";
        double err = pointOrZero(errorSeries, x);

        //qDebug() << "koveee";

        QString text;
        text += QString("Позиция серво: %1\n").arg(pos, 0, 'f', 4);
        text += QString("Позиция ДУП: %1\n").arg(dup, 0, 'f', 4);
        text += QString("Скорость: %1\n").arg(speed, 0, 'f', 4);
        text += QString("Ошибка: %1\n").arg(err, 0, 'f', 4);
        text += QString("Время: %1").arg(x, 0, 'f', 1);

        //qDebug() << "soveee";
        return text;
    }

private:
    bool capturing;
    bool hasHoverLine;
    bool isDragging;
    QPoint lastMousePos;
    QVector<CustomChartView *> *allCharts;
    qreal hoverX;
    QLabel *tooltip;
    QLineSeries *positionSeries;
    QLineSeries *dupSeries;
    QLineSeries *speedSeries;
    QLineSeries *errorSeries;
};

class MainWindow : public QMainWindow {
    Q_OBJECT

signals:
    void zeroDup(); // Сигнал для передачи данных во второй поток
    void zeroServo();
    void startMoveStaticModeServo(float speed, float angle);
    void startMoveDynamicModeServo(float speed, float rangeMin, float rangeMax);
    void startMoveSpeedModeServo(float speed);
    void disconnectDup();
    void disconnectServo();
    void stopServo();

public:
    MainWindow() {
        QWidget *central = new QWidget;
        QHBoxLayout *mainLayout = new QHBoxLayout(central); // Change to horizontal layout

        QVBoxLayout *leftLayout = new QVBoxLayout; // Create a vertical layout for the left side
        QWidget *leftWidget = new QWidget; // Wrap the left layout in a widget
        leftWidget->setLayout(leftLayout);
        leftWidget->setFixedWidth(250); // Set a fixed width for the left part

        // Серво block
        QGroupBox *servoGroup = new QGroupBox("Серво");
        QVBoxLayout *servoLayout = new QVBoxLayout(servoGroup);
        servoGroup->setFixedHeight(450); // Set fixed height for the "Серво" block

        portBoxServo = new QComboBox;
        portBoxServo->addItem("Channel 1");
        portBoxServo->addItem("Channel 2");
        QPushButton *connectBtnServo = new QPushButton("Подключиться Серво");
        connect(connectBtnServo, &QPushButton::clicked, this, [this, connectBtnServo]() {
            if (workerThread->stateConnectServo) {
                emit disconnectServo();
                connectBtnServo->setText("Подключиться Серво");
            } else {
                connectServo();
                if (workerThread->stateConnectServo) {
                    connectBtnServo->setText("Отключиться Серво");
                    setControlsEnabled(false);
                }
            }
        });

        positionLabel = new QLabel("Позиция: -");
        speedLabel = new QLabel("Скорость: -");

        servoLayout->addWidget(portBoxServo);
        servoLayout->addWidget(connectBtnServo);
        servoLayout->addWidget(positionLabel);
        servoLayout->addWidget(speedLabel);

        QHBoxLayout *modeLayout = new QHBoxLayout;
        QLabel *modeLabel = new QLabel("Режим:");
        QComboBox *modeBox = new QComboBox;
        modeBox->addItem("Статичный");
        modeBox->addItem("Динамичный");
        modeBox->addItem("Ручной");
        modeLayout->addWidget(modeLabel);
        modeLayout->addWidget(modeBox);
        servoLayout->addLayout(modeLayout);

        // Dynamic widgets for modes
        QStackedWidget *modeStack = new QStackedWidget;

        // Статичный mode
        QWidget *staticModeWidget = new QWidget;
        QVBoxLayout *staticLayout = new QVBoxLayout(staticModeWidget);

        QHBoxLayout *staticSpeedLayout = new QHBoxLayout;
        QLabel *staticSpeedLabel = new QLabel("Скорость:");
        staticSpeedInput = new QLineEdit;
        QLabel *staticSpeedUnit = new QLabel("об/мин");
        QLabel *staticSpeedConvertedLabel = new QLabel("0 град/с"); // Label for converted speed
        connect(staticSpeedInput, &QLineEdit::textChanged, this, [staticSpeedConvertedLabel](const QString &text) {
            bool ok;
            float speedRPM = text.toFloat(&ok);
            if (ok) {
                float speedRadPerSec = speedRPM * 6; // Conversion formula
                staticSpeedConvertedLabel->setText(QString::number(speedRadPerSec, 'f', 2) + " град/с");
            } else {
                staticSpeedConvertedLabel->setText("0 град/с");
            }
        });
        staticSpeedLayout->addWidget(staticSpeedLabel);
        staticSpeedLayout->addWidget(staticSpeedInput);
        staticSpeedLayout->addWidget(staticSpeedUnit);

        QHBoxLayout *staticConvertedLayout = new QHBoxLayout;
        staticConvertedLayout->addWidget(staticSpeedConvertedLabel);

        QHBoxLayout *staticAngleLayout = new QHBoxLayout;
        QLabel *staticAngleLabel = new QLabel("Угол:");
        staticAngleInput = new QLineEdit;
        QLabel *staticAngleUnit = new QLabel("°");
        staticAngleLayout->addWidget(staticAngleLabel);
        staticAngleLayout->addWidget(staticAngleInput);
        staticAngleLayout->addWidget(staticAngleUnit);

        QPushButton *staticSetButton = new QPushButton("Установить");
        connect(staticSetButton, &QPushButton::clicked, this, [this, staticSetButton]() {
            float angle = staticAngleInput->text().toFloat();
            float speed = staticSpeedInput->text().toFloat();
            emit startMoveStaticModeServo(speed, angle);
        });

        staticLayout->addLayout(staticSpeedLayout);
        staticLayout->addLayout(staticConvertedLayout); // Add converted speed label
        staticLayout->addLayout(staticAngleLayout);
        staticLayout->addWidget(staticSetButton);
        modeStack->addWidget(staticModeWidget);

        // Динамичный mode
        QWidget *dynamicModeWidget = new QWidget;
        QVBoxLayout *dynamicLayout = new QVBoxLayout(dynamicModeWidget);

        QHBoxLayout *dynamicSpeedLayout = new QHBoxLayout;
        QLabel *dynamicSpeedLabel = new QLabel("Скорость:");
        dynamicSpeedInput = new QLineEdit;
        QLabel *dynamicSpeedUnit = new QLabel("об/мин");
        QLabel *dynamicSpeedConvertedLabel = new QLabel("0 град/с"); // Label for converted speed
        connect(dynamicSpeedInput, &QLineEdit::textChanged, this, [dynamicSpeedConvertedLabel](const QString &text) {
            bool ok;
            float speedRPM = text.toFloat(&ok);
            if (ok) {
                float speedRadPerSec = speedRPM * 6; // Conversion formula
                dynamicSpeedConvertedLabel->setText(QString::number(speedRadPerSec, 'f', 2) + " град/с");
            } else {
                dynamicSpeedConvertedLabel->setText("0 град/с");
            }
        });
        dynamicSpeedLayout->addWidget(dynamicSpeedLabel);
        dynamicSpeedLayout->addWidget(dynamicSpeedInput);
        dynamicSpeedLayout->addWidget(dynamicSpeedUnit);

        QHBoxLayout *dynamicConvertedLayout = new QHBoxLayout;
        dynamicConvertedLayout->addWidget(dynamicSpeedConvertedLabel);

        QHBoxLayout *dynamicMinAngleLayout = new QHBoxLayout;
        QLabel *dynamicMinAngleLabel = new QLabel("Мин. угол:");
        dynamicMinAngleInput = new QLineEdit;
        QLabel *dynamicMinAngleUnit = new QLabel("°");
        dynamicMinAngleLayout->addWidget(dynamicMinAngleLabel);
        dynamicMinAngleLayout->addWidget(dynamicMinAngleInput);
        dynamicMinAngleLayout->addWidget(dynamicMinAngleUnit);

        QHBoxLayout *dynamicMaxAngleLayout = new QHBoxLayout;
        QLabel *dynamicMaxAngleLabel = new QLabel("Макс. угол:");
        dynamicMaxAngleInput = new QLineEdit;
        QLabel *dynamicMaxAngleUnit = new QLabel("°");
        dynamicMaxAngleLayout->addWidget(dynamicMaxAngleLabel);
        dynamicMaxAngleLayout->addWidget(dynamicMaxAngleInput);
        dynamicMaxAngleLayout->addWidget(dynamicMaxAngleUnit);

        QPushButton *dynamicStartButton = new QPushButton("Старт");
        connect(dynamicStartButton, &QPushButton::clicked, this, [this, dynamicStartButton]() {
            if (workerThread->flagMove) {
                emit stopServo();
                dynamicStartButton->setText("Старт");
            } else {
                float rangeMax = dynamicMaxAngleInput->text().toFloat();
                float rangeMin = dynamicMinAngleInput->text().toFloat();
                float speed = dynamicSpeedInput->text().toFloat();
                emit startMoveDynamicModeServo(speed, rangeMin, rangeMax);
                dynamicStartButton->setText("Стоп");
            }
        });

        dynamicLayout->addLayout(dynamicSpeedLayout);
        dynamicLayout->addLayout(dynamicConvertedLayout); // Add converted speed label
        dynamicLayout->addLayout(dynamicMinAngleLayout);
        dynamicLayout->addLayout(dynamicMaxAngleLayout);
        dynamicLayout->addWidget(dynamicStartButton);
        modeStack->addWidget(dynamicModeWidget);

        // Ручной mode
        QWidget *manualModeWidget = new QWidget;
        QVBoxLayout *manualLayout = new QVBoxLayout(manualModeWidget);

        QHBoxLayout *manualSpeedLayout = new QHBoxLayout;
        QLabel *manualSpeedLabel = new QLabel("Скорость:");
        manualSpeedInput = new QLineEdit;
        QLabel *manualSpeedUnit = new QLabel("об/мин");
        QLabel *manualSpeedConvertedLabel = new QLabel("0 град/с"); // Label for converted speed
        connect(manualSpeedInput, &QLineEdit::textChanged, this, [manualSpeedConvertedLabel](const QString &text) {
            bool ok;
            float speedRPM = text.toFloat(&ok);
            if (ok) {
                float speedRadPerSec = speedRPM * 6; // Conversion formula
                manualSpeedConvertedLabel->setText(QString::number(speedRadPerSec, 'f', 2) + " град/с");
            } else {
                manualSpeedConvertedLabel->setText("0 град/с");
            }
        });
        manualSpeedLayout->addWidget(manualSpeedLabel);
        manualSpeedLayout->addWidget(manualSpeedInput);
        manualSpeedLayout->addWidget(manualSpeedUnit);

        QHBoxLayout *manualConvertedLayout = new QHBoxLayout;
        manualConvertedLayout->addWidget(manualSpeedConvertedLabel);

        QHBoxLayout *manualButtonLayout = new QHBoxLayout;
        QPushButton *manualLeftButton = new QPushButton("←");
        QPushButton *manualStopButton = new QPushButton("Стоп");
        QPushButton *manualRightButton = new QPushButton("→");

        // Обработка нажатия кнопки ←
        connect(manualLeftButton, &QPushButton::clicked, this, [this]() {
            float speed = manualSpeedInput->text().toFloat();
            emit startMoveSpeedModeServo(-speed); // Передача отрицательной скорости
        });

        connect(manualStopButton, &QPushButton::clicked, this, [this]() {
            emit stopServo();
        });


        // Обработка нажатия кнопки →
        connect(manualRightButton, &QPushButton::clicked, this, [this]() {
            float speed = manualSpeedInput->text().toFloat();
            emit startMoveSpeedModeServo(speed); // Передача положительной скорости
        });

        manualButtonLayout->addWidget(manualLeftButton);
        manualButtonLayout->addWidget(manualStopButton);
        manualButtonLayout->addWidget(manualRightButton);

        manualLayout->addLayout(manualSpeedLayout);
        manualLayout->addLayout(manualConvertedLayout); // Add converted speed label
        manualLayout->addLayout(manualButtonLayout);
        modeStack->addWidget(manualModeWidget);

        // Add mode stack to layout
        servoLayout->addWidget(modeStack);

        // Change mode stack based on dropdown selection
        connect(modeBox, &QComboBox::currentIndexChanged, modeStack, &QStackedWidget::setCurrentIndex);

        // Add "Обнулить" button
        QPushButton *resetButton = new QPushButton("Обнулить");

        connect(resetButton, &QPushButton::clicked, this, [this, resetButton]() {

            emit zeroServo();

        });


        servoLayout->addWidget(resetButton);

        // Add servo group to left layout
        leftLayout->addWidget(servoGroup);

        // ДУП block
        QGroupBox *dupGroup = new QGroupBox("ДУП");
        QVBoxLayout *dupLayout = new QVBoxLayout(dupGroup);

        portBoxDUP = new QComboBox;
        portBoxDUP->addItem("Channel 2");
        portBoxDUP->addItem("Channel 1");
        dupLayout->addWidget(portBoxDUP);

        // Add "Подключиться ДУП" button
        QPushButton *connectDUPBtn = new QPushButton("Подключиться ДУП");
        connect(connectDUPBtn, &QPushButton::clicked, this, [this, connectDUPBtn]() {
            if (workerThread->stateConnectDup) {
                emit disconnectDup();
                timer->stop();
                setControlsEnabled(false);
                //workerThread->stop();
                connectDUPBtn->setText("Подключиться ДУП");
            } else {
                connectDUP();
                if (workerThread->stateConnectDup) {
                    connectDUPBtn->setText("Отключиться ДУП");
                }
            }
        });
        dupLayout->addWidget(connectDUPBtn);

        dupLabel = new QLabel("ДУП позиция: -");
        dupLayout->addWidget(dupLabel);

        // Add "Инверсия" checkbox above resetDupButton
        QCheckBox *invertDupCheckbox = new QCheckBox("Инверсия");
        connect(invertDupCheckbox, &QCheckBox::stateChanged, this, [this](int state) {
            invertDup = (state == Qt::Checked);
            //qDebug() << "Инверсия ДУП:" << (invertDup ? "Включена" : "Выключена");
        });
        dupLayout->addWidget(invertDupCheckbox);

        // Add "Обнулить" button below dupLabel
        QPushButton *resetDupButton = new QPushButton("Обнулить");
        connect(resetDupButton, &QPushButton::clicked, this, [this]() {
            // Add functionality for resetting DUP here
            emit zeroDup();
            //qDebug() << "Обнулить ДУП clicked.";
        });
        dupLayout->addWidget(resetDupButton);

        // Add blocks to the left layout
        leftLayout->addWidget(servoGroup);
        leftLayout->addWidget(dupGroup);

        errorLabel = new QLabel("Ошибка: -");
        leftLayout->addWidget(errorLabel);

        maxErrorLabel = new QLabel("Максимальная ошибка: -");
        leftLayout->addWidget(maxErrorLabel);

        allCharts = new QVector<CustomChartView *>(); // Инициализация allCharts

        positionSeries = new QLineSeries();
        dupSeries = new QLineSeries();
        speedSeries = new QLineSeries();
        errorSeries = new QLineSeries();

        // First chart: Positions
        QChart *positionChart = new QChart();
        positionChart->setTitle("Позиция Серво и ДУП"); // Set title inside the chart

        positionChart->addSeries(positionSeries);
        positionChart->addSeries(dupSeries);

        QValueAxis *positionAxisX = new QValueAxis;
        QValueAxis *positionAxisY = new QValueAxis;
        positionAxisY->setRange(-1600, 1600); // Set Y-axis range for positions
        positionChart->setAxisX(positionAxisX, positionSeries);
        positionChart->setAxisY(positionAxisY, positionSeries);
        positionChart->setAxisX(positionAxisX, dupSeries);
        positionChart->setAxisY(positionAxisY, dupSeries);
        positionChart->legend()->hide();

        positionChart->setMargins(QMargins(0, 0, 0, 0)); // добавьте эту строку

        CustomChartView *positionChartView = new CustomChartView(positionChart, this, allCharts, positionSeries, dupSeries, speedSeries, errorSeries);
        allCharts->append(positionChartView);

        // Second chart: Speed
        QChart *speedChart = new QChart();
        speedChart->setTitle("Скорость"); // Set title inside the chart

        speedChart->addSeries(speedSeries);

        QValueAxis *speedAxisX = new QValueAxis;
        QValueAxis *speedAxisY = new QValueAxis;
        speedAxisY->setRange(-500, 500); // Set Y-axis range for speed
        speedChart->setAxisX(speedAxisX, speedSeries);
        speedChart->setAxisY(speedAxisY, speedSeries);
        speedChart->legend()->hide();

        speedChart->setMargins(QMargins(6, 0, 0, 0)); // добавьте эту строку

        CustomChartView *speedChartView = new CustomChartView(speedChart, this, allCharts, positionSeries, dupSeries, speedSeries, errorSeries);
        allCharts->append(speedChartView);

        // Third chart: Error
        QChart *errorChart = new QChart();
        errorChart->setTitle("Ошибка Угла"); // Set title inside the chart

        errorChart->addSeries(errorSeries);

        QValueAxis *errorAxisX = new QValueAxis;
        QValueAxis *errorAxisY = new QValueAxis;
        errorAxisY->setRange(-5, 5); //
        errorAxisX->setTitleText("Время (мс)");
        errorChart->setAxisX(errorAxisX, errorSeries);
        errorChart->setAxisY(errorAxisY, errorSeries); // Auto-calibrate Y-axis
        errorChart->legend()->hide();

        errorChart->setMargins(QMargins(17, 0, 0, 0)); // добавьте эту строку

        CustomChartView *errorChartView = new CustomChartView(errorChart, this, allCharts, positionSeries, dupSeries, speedSeries, errorSeries);
        allCharts->append(errorChartView);

        //positionChartView->layout()->setContentsMargins(100,0,0,0);
        //positionChart->setMargins(QMargins(10, 10, 10, 10)); // добавьте эту строку
        //positionChartView->update();


        // Add charts to the main layout
        QVBoxLayout *chartsLayout = new QVBoxLayout;
        chartsLayout->addWidget(positionChartView);
        chartsLayout->addWidget(speedChartView);
        chartsLayout->addWidget(errorChartView);


        // Add "Сохранить таблицу" and "Захват" buttons below the charts
        QHBoxLayout *buttonLayout = new QHBoxLayout;

        // Add "Движение графика" checkbox
        QCheckBox *dynamicGraphCheckbox = new QCheckBox("Движение графика");
        buttonLayout->addWidget(dynamicGraphCheckbox);

        // Add "Очистить данные и график" button
        QPushButton *clearDataButton = new QPushButton("Очистить данные и график");
        connect(clearDataButton, &QPushButton::clicked, this, [this]() {
            positionPoints.clear();
            angleDupPoints.clear();
            speedPoints.clear();
            errorPoints.clear();

            positionSeries->clear();
            dupSeries->clear();
            speedSeries->clear();
            errorSeries->clear();

            positionData.clear();
            speedData.clear();
            dupData.clear();
            errorData.clear();
            maxError = 0;

            //qDebug() << "Данные и графики очищены.";


        });
        buttonLayout->addWidget(clearDataButton);

        QPushButton *captureBtn = new QPushButton("Захват");
        connect(captureBtn, &QPushButton::clicked, this, [this, captureBtn, positionChartView, speedChartView, errorChartView]() {
            if (timer->isActive()) {
                timer->stop();
                captureBtn->setText("Старт");
                //qDebug() << "Графики остановлены.";
                // Отключаем масштабирование при остановке захвата
                positionChartView->setCapturing(true);
                speedChartView->setCapturing(true);
                errorChartView->setCapturing(true);
            } else {
                timer->start(50);
                captureBtn->setText("Захват");
                //qDebug() << "Графики запущены.";
                // Включаем масштабирование при захвате
                positionChartView->setCapturing(false);
                speedChartView->setCapturing(false);
                errorChartView->setCapturing(false);

                // Сбрасываем диапазон оси Y для графиков позиции и скорости
                QValueAxis *positionAxisY = qobject_cast<QValueAxis *>(positionChartView->chart()->axes(Qt::Vertical).first());
                if (positionAxisY) {
                    positionAxisY->setRange(-1600, 1600);
                }

                QValueAxis *speedAxisY = qobject_cast<QValueAxis *>(speedChartView->chart()->axes(Qt::Vertical).first());
                if (speedAxisY) {
                    speedAxisY->setRange(-500, 500);
                }

                QValueAxis *errorAxisY = qobject_cast<QValueAxis *>(errorChartView->chart()->axes(Qt::Vertical).first());
                if (errorAxisY) {
                    errorAxisY->setRange(-5, 5);
                }
            }
        });

        
        buttonLayout->addWidget(captureBtn);

        QPushButton *saveBtn = new QPushButton("Сохранить таблицу");
        connect(saveBtn, &QPushButton::clicked, this, &MainWindow::saveToExcel);
        buttonLayout->addWidget(saveBtn);
        chartsLayout->addLayout(buttonLayout);

        // Новый блок для графиков и элементов управления
        QWidget *chartsBlock = new QWidget;
        QVBoxLayout *chartsBlockLayout = new QVBoxLayout(chartsBlock);
        chartsBlockLayout->addLayout(chartsLayout);

        // Изменяем добавление элементов в основной макет
        mainLayout->addWidget(leftWidget);                     // левый блок
        mainLayout->addWidget(chartsBlock);                    // блок графиков

        // Новый блок справа с двумя надписями
        QWidget *rightWidget = new QWidget;
        rightWidget->setFixedWidth(130);
        QVBoxLayout *rightLayout = new QVBoxLayout(rightWidget);
        rightLayout->setAlignment(Qt::AlignTop);  // выравнивание по верху

        // --- ДОБАВИТЬ: чекбоксы для управления отображением графиков ---
        QCheckBox *servoChartCheck = new QCheckBox();
        servoChartCheck->setChecked(true); // По умолчанию включено

        QLabel *servoPosLabel = new QLabel("Позиция Серво");
        servoPosLabel->setStyleSheet("QLabel { color: #209fdf; font-weight: bold;}");

        QHBoxLayout *servoRowLayout = new QHBoxLayout;
        servoRowLayout->addWidget(servoChartCheck);
        servoRowLayout->addWidget(servoPosLabel);
        servoRowLayout->addStretch(); // Чтобы прижать к началу

        QCheckBox *dupChartCheck = new QCheckBox();
        dupChartCheck->setChecked(true);

        QLabel *dupPosLabel = new QLabel("Позиция ДУП");
        dupPosLabel->setStyleSheet("QLabel { color: #99ca53; font-weight: bold;}");

        QHBoxLayout *dupRowLayout = new QHBoxLayout;
        dupRowLayout->addWidget(dupChartCheck);
        dupRowLayout->addWidget(dupPosLabel);
        dupRowLayout->addStretch();
        // ---------------------------------------------

        rightLayout->addLayout(servoRowLayout);
        rightLayout->addLayout(dupRowLayout);
        mainLayout->addWidget(rightWidget);

        connect(servoChartCheck, &QCheckBox::toggled, this, [this](bool checked) {
            positionSeries->setVisible(checked);
        });
        connect(dupChartCheck, &QCheckBox::toggled, this, [this](bool checked) {
            dupSeries->setVisible(checked);
        });

        setCentralWidget(central);

        timer = new QTimer(this);
        connect(timer, &QTimer::timeout, this, [this, dynamicGraphCheckbox]() {
            updateGraph(dynamicGraphCheckbox->isChecked());
        });

        workerThread = new WorkerThread(this);

        // Соединяем сигнал из MainWindow с слотом в WorkerThread
        connect(this, &MainWindow::zeroDup, workerThread, &WorkerThread::zeroDup);

        connect(this, &MainWindow::zeroServo, workerThread, &WorkerThread::zeroServo);

        connect(this, &MainWindow::startMoveStaticModeServo, workerThread, &WorkerThread::startMoveStaticModeServo);

        connect(this, &MainWindow::startMoveDynamicModeServo, workerThread, &WorkerThread::startMoveDynamicModeServo);

        connect(this, &MainWindow::startMoveSpeedModeServo, workerThread, &WorkerThread::startMoveSpeedModeServo);

        connect(this, &MainWindow::stopServo, workerThread, &WorkerThread::stopServo);

        connect(workerThread, &WorkerThread::dataUpdated, this, &MainWindow::handleProcessedValue);

        connect(workerThread, &WorkerThread::errorSend, this, &MainWindow::handleError);

        connect(this, &MainWindow::disconnectServo, workerThread, &WorkerThread::disconnectServo);

        connect(this, &MainWindow::disconnectDup, workerThread, &WorkerThread::disconnectDup);

        controlledWidgets.append(modeBox);

        controlledWidgets.append(staticSpeedInput);
        controlledWidgets.append(staticAngleInput);
        controlledWidgets.append(staticSetButton);

        controlledWidgets.append(dynamicSpeedInput);
        controlledWidgets.append(dynamicMinAngleInput);
        controlledWidgets.append(dynamicMaxAngleInput);
        controlledWidgets.append(dynamicStartButton);

        controlledWidgets.append(manualSpeedInput);
        controlledWidgets.append(manualLeftButton);
        controlledWidgets.append(manualRightButton);

        controlledWidgets.append(resetButton);
        controlledWidgets.append(resetDupButton);
        controlledWidgets.append(invertDupCheckbox);

        controlledWidgets.append(clearDataButton);
        controlledWidgets.append(captureBtn);
        controlledWidgets.append(saveBtn);
        controlledWidgets.append(dynamicGraphCheckbox);

        setControlsEnabled(false);

        // Example of enqueuing data
        //QTimer::singleShot(1000, this, [this]() {
        //    workerThread->connectServo(portBoxServo->currentText());
        //});

        // Пример вызова сигнала для передачи данных
        //emit sendDataToWorker(42); // Передача данных во второй поток
    }

    ~MainWindow() {
        delete allCharts; // Удаление allCharts

        workerThread->stop();
        workerThread->wait();

        delete workerThread;
    }

private slots:

    void handleError(QString erroMsg){
        QMessageBox::warning(nullptr, "Ошибка", erroMsg);
    }

    void setControlsEnabled(bool enabled) {
        for (QWidget *w : controlledWidgets) {
            w->setEnabled(enabled);
        }
    }

    void connectServo(){

        if(portBoxServo->currentText() == "Channel 1") workerThread->channelServo = 0;
        else workerThread->channelServo = 1;
        workerThread->connectServo();
        if (workerThread->stateConnectServo && workerThread->stateConnectDup) {
                setControlsEnabled(true);
            } else {
                setControlsEnabled(false);
            }
    }

    void connectDUP() {



        if(portBoxDUP->currentText() == "Channel 1") workerThread->channelDup = 0;
        else workerThread->channelDup = 1;
        workerThread->connectDup();
        timer->start(500);
        elapsedTimer.start();

        if (workerThread->stateConnectServo && workerThread->stateConnectDup) {
                setControlsEnabled(true);
            } else {
                setControlsEnabled(false);
            }

        workerThread->start();

    }

    void resetZoom(){
         QValueAxis *positionAxisY = qobject_cast<QValueAxis *>(positionChartView->chart()->axes(Qt::Vertical).first());
                if (positionAxisY) {
                    positionAxisY->setRange(-1600, 1600);
                }

                QValueAxis *speedAxisY = qobject_cast<QValueAxis *>(speedChartView->chart()->axes(Qt::Vertical).first());
                if (speedAxisY) {
                    speedAxisY->setRange(-500, 500);
                }

                QValueAxis *errorAxisY = qobject_cast<QValueAxis *>(errorChartView->chart()->axes(Qt::Vertical).first());
                if (errorAxisY) {
                    errorAxisY->setRange(-5, 5);
                }
    }

    void updateGraph(bool isDynamic) {
        qint64 currentTime = elapsedTimer.elapsed();
        qint64 windowStart = isDynamic ? qMax(0ll, currentTime - 10000) : qMax(0ll, currentTime - 100000);

        // Фильтруем последние значения
        auto filterPoints = [windowStart](const QVector<QPointF> &points) {
            QVector<QPointF> filtered;
            for (const auto &point : points) {
                if (point.x() >= windowStart) {
                    filtered.append(point);
                }
            }
            return filtered;
        };

        positionSeries->replace(filterPoints(positionPoints));
        dupSeries->replace(filterPoints(angleDupPoints));
        speedSeries->replace(filterPoints(speedPoints));
        errorSeries->replace(filterPoints(errorPoints));

        // Update X-axis range for all charts
        positionSeries->attachedAxes().first()->setRange(windowStart, currentTime);
        speedSeries->attachedAxes().first()->setRange(windowStart, currentTime);
        errorSeries->attachedAxes().first()->setRange(windowStart, currentTime);

        // Auto-scale Y-axis for the error chart
        if (!errorPoints.isEmpty()) {
            auto [minY, maxY] = std::minmax_element(errorPoints.begin(), errorPoints.end(),
                [](const QPointF &a, const QPointF &b) { return a.y() < b.y(); });
            double min = minY->y();
            double max = maxY->y();
            double margin = 0.1 * std::max(std::abs(min), std::abs(max)); // 10% запас

            // По умолчанию диапазон -5...5
            double defaultMin = -5.0, defaultMax = 5.0;

            // Если выходит за пределы — расширяем
            if (min < defaultMin || max > defaultMax) {
                min = std::min(min - margin, defaultMin);
                max = std::max(max + margin, defaultMax);
            } else {
                min = defaultMin;
                max = defaultMax;
            }
            errorSeries->attachedAxes().last()->setRange(min, max);
        }

    }

    void saveToExcel() {
        QString filePath = QFileDialog::getSaveFileName(this, "Сохранить файл", "", "Excel Files (*.csv)");
        if (filePath.isEmpty()) {
            return;
        }

        QFile file(filePath);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            //qDebug() << "Не удалось открыть файл для записи.";
            return;
        }

        QTextStream out(&file);
        out << "Time (ms);Position;Speed;Dup position;Error\n";

        int maxSize = qMax(positionData.size(), qMax(speedData.size(), dupData.size()));
        for (int i = 0; i < maxSize; ++i) {
            QString time = (i < positionData.size()) ? QString::number(positionData[i].first) : "";
            QString position = (i < positionData.size()) ? QString::number(positionData[i].second, 'f', 4).replace('.', ',') : "";
            QString speed = (i < speedData.size()) ? QString::number(speedData[i].second, 'f', 4).replace('.', ',') : "";
            QString dup = (i < dupData.size()) ? QString::number(dupData[i].second, 'f', 4).replace('.', ',') : "";
            QString error = (i < errorData.size()) ? QString::number(errorData[i].second, 'f', 4).replace('.', ',') : "";

            out << time << ";" << position << ";" << speed << ";" << dup << ";" << error << "\n";
        }

        file.close();
        //qDebug() << "Данные успешно сохранены в файл:" << filePath;
    }



    void handleProcessedValue(int32_t positionServo, int32_t speedServo, int32_t positionDup, int64_t timeStamp) {

        //qDebug() << "Speed" << Qt::hex << speedServo << "Pos servo" << positionServo << "Pos Dup" << positionDup << "time:" << timeStamp;
        if(positionDup == 0xFFFF){

            dupLabel->setText("Недоступен");

        } else {

            float positionDupMath = ((float(positionDup) * 0.0009765625) - 31.3740234375) * 57.2958;

            if (invertDup) {
                positionDupMath = -positionDupMath;
            }

            if(!workerThread->dirMove){
                positionDupMath -= workerThread->offsetBacklashServo;
            }

            if (positionDup == 0xFE00){

                dupLabel->setText("Ошибка");
                dupLabel->setStyleSheet("QLabel { color: red;}");

            } else {

                dupLabel->setText("ДУП позиция: " + QString::number(positionDupMath, 'f', 4) + "°");
                dupLabel->setStyleSheet("QLabel { color: black;}");
                angleDupPoints.append(QPointF(timeStamp, positionDupMath));
                dupData.append(qMakePair(timeStamp, positionDupMath));

            }

            float positionServoMath = positionServo / 7065.6;

            positionLabel->setText("Позиция: " + QString::number(positionServoMath, 'f', 4) + "°");
            positionPoints.append(QPointF(timeStamp, positionServoMath));
            positionData.append(qMakePair(timeStamp, positionServoMath));

            float error = positionDupMath - positionServoMath;
            errorLabel->setText("Ошибка: " + QString::number(error, 'f', 4) + "°");


            if (std::abs(error) > maxError) {
                maxError = std::abs(error);
                maxErrorLabel->setText("Максимальная ошибка: " + QString::number(maxError, 'f', 4) + "°");
            }

            errorPoints.append(QPointF(timeStamp, error));
            errorData.append(qMakePair(timeStamp, error));

            float speedServoMath = speedServo / 19.40625;

            speedLabel->setText("Скорость: " + QString::number(speedServoMath, 'f', 4) + " об/мин");
            speedPoints.append(QPointF(timeStamp, speedServoMath));
            speedData.append(qMakePair(timeStamp, speedServoMath));
        }
        //qDebug() << "Speed" << Qt::hex << speedServoMath << "Pos servo" << positionServoMath << "Pos Dup" << positionDupMath << "time:" << timeStamp;

    }

    void someFunction() {
        int dataToSend = 100; // Пример данных
        //emit sendDataToWorker(dataToSend); // Отправка данных во второй поток
    }

private:
    QComboBox *portBoxServo;
    QComboBox *portBoxDUP;

    QLabel *positionLabel;
    QLabel *speedLabel;
    QLabel *dupLabel;
    QLabel *errorLabel;
    QLabel *maxErrorLabel;

    QByteArray buffer;
    QByteArray bufferDUP;

    QTimer *timer;

    QLineSeries *positionSeries;
    QLineSeries *speedSeries;
    QLineSeries *dupSeries;
    QLineSeries *errorSeries;

    QVector<QPointF> positionPoints;
    QVector<QPointF> angleDupPoints;
    QVector<QPointF> speedPoints;
    QVector<QPointF> errorPoints;

    QElapsedTimer elapsedTimer;

    float currentPosition = 0;
    float currentSpeed = 0;

    QVector<QPair<qint64, float>> positionData;
    QVector<QPair<qint64, float>> speedData;
    QVector<QPair<qint64, float>> dupData;
    QVector<QPair<qint64, float>> errorData;

    float maxError = 0;

    QLineEdit *staticAngleInput; // Declare as a member variable
    QLineEdit *staticSpeedInput; // Declare as a member variable
    QLineEdit *dynamicMinAngleInput; // Declare as a member variable
    QLineEdit *dynamicMaxAngleInput; // Declare as a member variable
    QLineEdit *dynamicSpeedInput;
    QLineEdit *manualSpeedInput;


    bool invertDup = false; // Add a member variable to track inversion state

    WorkerThread *workerThread;
    QVector<CustomChartView *> *allCharts; // Добавлена переменная allCharts

    QVector<QWidget*> controlledWidgets;

};

#include "main.moc"

int main(int argc, char *argv[]) {

    qInstallMessageHandler(customMessageHandler);

    QApplication app(argc, argv);

    QSharedMemory sharedMemory("DupSingleInstance");


        if (!sharedMemory.create(1)) {
            QMessageBox::warning(nullptr, "Ошибка", "Приложение уже запущено!");
            return 1;
        }

        QObject::connect(&app, &QCoreApplication::aboutToQuit, [&sharedMemory](){
                sharedMemory.detach();
            });


    app.setWindowIcon(QIcon(":/favicon.ico"));

    MainWindow window;
    window.resize(1100, 800); // Set initial size
    window.setMinimumSize(1100, 800); // Set minimum size
    window.setWindowTitle("Серво ДУП v2.5"); // <-- Ваше название
    window.show();

    int result = app.exec();

        // Явно освобождаем память перед выходом
        sharedMemory.detach();

        return result;
}

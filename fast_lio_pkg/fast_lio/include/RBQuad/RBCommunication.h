#ifndef RBCOMMUNICATION_H
#define RBCOMMUNICATION_H

#include <QTimer>
#include <QThread>
#include <QCoreApplication>
#include <QObject>
#include <QtNetwork>
#include <iostream>

#include "Common.h"
#include "SharedMemory.h"

class RBCommunication : public QObject
{
    Q_OBJECT
public:
    RBCommunication();
    virtual ~RBCommunication();

    QTcpSocket  *socket;

    QByteArray buf;
//     // QVector<USER_COMMAND> commands;

    void Connect(QString ip);
    void Disconnect();

public slots:
    void onConnected();
    void onDisconnected();
    void onReadyRead();
};


class SlotThread : public QThread
{
    Q_OBJECT
public:
    SlotThread();
    ~SlotThread() override;
    QTimer *qTimer;
    
private slots:
    void RobotStateUpdate();

protected:
    void run() override;
};

#endif // RBCOMMUNICATION_H

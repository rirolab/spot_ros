#include "RBCommunication.h"
#define R2D                  5.729577951308232e1

extern pRBCORE_SHM sharedData;

RBCommunication::RBCommunication()
{
    socket = new QTcpSocket(this);
    connect(socket, SIGNAL(connected()), this, SLOT(onConnected()));
    connect(socket, SIGNAL(disconnected()), this, SLOT(onDisconnected()));
    connect(socket, SIGNAL(readyRead()), this, SLOT(onReadyRead()));
}


RBCommunication::~RBCommunication(){}


void RBCommunication::Connect(QString ip){
    if(socket->state() != QAbstractSocket::ConnectedState){
        socket->connectToHost(ip, 8000);
    }
}

void RBCommunication::Disconnect(){
    buf.clear();
    socket->disconnectFromHost();
}

void RBCommunication::onConnected(){
    qDebug() << "Connected..";
    sharedData->LanComm_Status = true;
}

void RBCommunication::onDisconnected(){
    qDebug() << "Disconnected..";
    sharedData->LanComm_Status = false;
}

void RBCommunication::onReadyRead(){

    buf.append(socket->readAll());

    while(1)
    {
        bool is_header = false;
        for(int p = 0; p < buf.size()-1; p++)
        {
            if(buf[p] == 0xFF && buf[p+1] == 0xFE)
            {
                is_header = true;
                buf.remove(0, p);
                break;
            }
        }

        const int packet_size = sizeof(ROBOT_STATE_DATA)+4;
        if(is_header)
        {
            if(buf.size() >= packet_size)
            {
                // check tail
                if(buf[packet_size-2] == 0x00 && buf[packet_size-1] == 0x01)
                {
                    // delete header
                    buf.remove(0, 2);

                    // parising
                    QByteArray tempBuf = buf.left(sizeof(ROBOT_STATE_DATA));
                    buf.remove(0, sizeof(ROBOT_STATE_DATA)+2);

                    ROBOT_STATE_DATA newRobotData;
                    memcpy(&newRobotData, tempBuf.data(), sizeof(ROBOT_STATE_DATA));
                    sharedData->ROBOT_DATA = newRobotData;
                }
                else
                {
                    // delete header only
                    buf.remove(0, 2);
                }
            }
            else
            {
                break;
            }
        }
        else
        {
            buf.clear();
            break;
        }
    }
}



SlotThread::SlotThread()
{
    sharedData = (pRBCORE_SHM)malloc(sizeof(RBCORE_SHM));
    memset(sharedData, 0, sizeof(RBCORE_SHM));

    /* DP: SOCKET Initialization for ROBOT_DATA subscription */
    sharedData = (pRBCORE_SHM)malloc(sizeof(RBCORE_SHM));    
    RBCommunication *comm;
    comm = new RBCommunication();
    comm->Connect("192.168.1.10"); //hardcoded again inside

    qTimer = new QTimer();    
    connect(qTimer, SIGNAL(timeout()), this, SLOT(RobotStateUpdate()));
    qTimer->start(10);

}

SlotThread::~SlotThread(){}

void SlotThread::run()
{
    while(1)
    {
        if (QThread::currentThread()->isInterruptionRequested()){
            return;
        }
    }
}

void SlotThread::RobotStateUpdate()
{
  // std::cout << sharedData->ROBOT_DATA.Sensor.imu.gyro.x()*R2D << " " << sharedData->ROBOT_DATA.Sensor.imu.acc.z() << std::endl;    
}

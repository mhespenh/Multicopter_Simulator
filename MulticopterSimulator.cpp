/***********************************************************************
  Group 3 ECE4574 Spring 2012
  Project 3 - Multicopter Simulator
    Multicopter_Simulator - Control process for simulator (master branch)

  Revisions:
    03/21/2012 - Initial version (master branch)
    03/23/2012 - Implemented dbus communication correctly
***********************************************************************/

#include "MulticopterSimulator.h"
#include <stdio.h>
#include <QDebug>
#include <QDBusArgument>

MulticopterSimulator::MulticopterSimulator(int numProcs, QObject *parent) :
    QObject(parent), bus(QDBusConnection::sessionBus()), sharedMem("PRIVATE_SHARED")
{
    procs = new QProcess[numProcs];
    throttles = new int[numProcs];

    //Signal: procStart- Motor process will emit this signal over the dbus when is starts and connects
    bus.connect("", "/", "edu.vt.ece.procStart", "procStart", this, SLOT(processStarted(QString)));
    bus.connect("", "/", "edu.vt.ece.ack", "ack", this, SLOT(recvMessage(QString)));
    bus.connect("", "/", "edu.vt.ece.updateThrottle", "updateThrottle", this, SLOT(recvUpdate(QString)));
    QStringList args;

    for(int i=0; i<numProcs; i++) {
//        QObject::connect(procs[i], SIGNAL(readyReadStandardError()), this, SLOT(processError()));
//        QObject::connect(procs[i], SIGNAL(readyReadStandardOutput()), this, SLOT(processSTDOUT()));
//        QObject::connect(procs[i], SIGNAL(finished(int,QProcess::ExitStatus)), this, SLOT(processExit(int,QProcess::ExitStatus)));
        QString i_string = QString::number(i);
        args << i_string;
        qDebug() << args;
        procs[i].start("/home/mhespenh/Desktop/Project2/SimMotor/SimMotor", args);
        qDebug() << "Motor process " << i << " started";
        args.pop_front();
    }

    dt = 0.01;     //simulation time
    QTimer *physicsTimer = new QTimer(this);
    connect(physicsTimer, SIGNAL(timeout()), this, SLOT(updatePhysics()));
    physicsTimer->start(dt*1000); //10ms timer
}

MulticopterSimulator::~MulticopterSimulator() {
    qDebug() << "Cleaning up...";
    sharedMem.detach();
    sharedMem.deleteLater();
    delete throttles;
    delete procs;
}

void MulticopterSimulator::recvMessage(QString msg) {
    qDebug() << "Received from motor process (DBus):   " << msg;
}

void MulticopterSimulator::sendDbusMessage(QString msg, int type) {
    QDBusMessage out = QDBusMessage::createSignal("/", "edu.vt.ece.msg", "msg");
    out << msg << type;
    bus.send(out);
}

void MulticopterSimulator::sendUpdateMsg(double pitch, double roll, double altitude) {
    QDBusMessage update = QDBusMessage::createSignal("/", "edu.vt.ece.updateAngles", "updateAngles");
    update << pitch << roll << altitude;
    bus.send(update);
}

bool MulticopterSimulator::writeSharedMem() {
    if (sharedMem.isAttached()) {
        sharedMem.detach();
    }
    data* theData;

     if (!sharedMem.create(sizeof(data))) {
         return false;
     }
     sharedMem.lock();
     theData = (data*)sharedMem.data();
     theData->t0 = 5;
     theData->t1 = 66;
     theData->t2 = 77;
     theData->t3 = 8;
     sharedMem.unlock();
     qDebug() << "Wrote to Shared Memory: " << theData->t0 << theData->t1 << theData->t2 << theData->t3;
     return true;
}

void MulticopterSimulator::updatePhysics() {
//    pitch += ((throttle-25)*cos(pitch*(PI/180)))*cos(roll*(PI/180)) * 0.1;
    sendUpdateMsg(pitch, roll, altitude);
}

void MulticopterSimulator::processStarted(QString reply) {
    qDebug() << "Received from motor process (DBus):   " << reply;
    sendDbusMessage("Hello, world!", 69);
}

void MulticopterSimulator::processError() {
    QByteArray data = proc->readAllStandardError();
    data.chop(1); //remove the \n
    qDebug() << "Received from motor process (STDERR): " << QString(data);

}

void MulticopterSimulator::processExit(int foo, QProcess::ExitStatus bar) {
    qDebug() << "Motor Process Exited with status:     " << bar << "/" << foo;
    qDebug() << "So we'll quit too.";
    QCoreApplication::quit();
}

void MulticopterSimulator::processSTDOUT() {
    QByteArray data = proc->readAllStandardOutput();
    qDebug() << "Received (STDOUT): " << QString(data);
}

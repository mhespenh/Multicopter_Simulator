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

#define PI 3.14159265

MulticopterSimulator::MulticopterSimulator(int numProcs, QObject *parent) :
    QObject(parent), bus(QDBusConnection::sessionBus()), sharedMem("PRIVATE_SHARED")
{
    procs = new QProcess[numProcs];
    throttles = new double[numProcs];

    //Signal: procStart- Motor process will emit this signal over the dbus when is starts and connects
    bus.connect("", "/", "edu.vt.ece.procStart", "procStart", this, SLOT(processStarted(QString)));
    bus.connect("", "/", "edu.vt.ece.ack", "ack", this, SLOT(recvMessage(QString)));
    bus.connect("", "/", "edu.vt.ece.updateThrottle", "updateThrottle", this, SLOT(recvUpdate(int, double)));
    QStringList args;

    for(int i=0; i<numProcs; i++) {
        QString i_string = QString::number(i);
        args << i_string; //which motor is this?
        QString n_string = QString::number(numProcs);
        args << n_string; //how many are there?
        //start the process
        procs[i].start("/home/mhespenh/Desktop/Project2/SimMotor/SimMotor", args);
        qDebug() << "Motor process " << i << " started";
        args.pop_front(); //pop off the args
        args.pop_front(); //so we can use it for the next one
        throttles[i] = 0.1;
    }
    numMotors = numProcs;
    dt = 0.1;     //simulation time

    targetPitch = 0;     //in degrees
    targetRoll  = -10;   //in degrees
    targetAltitude = 20; //in meters
    curPitch = 0.00001;
    curRoll = 0.00001;
    curAltitude = 0;

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

void MulticopterSimulator::recvUpdate(int motorNum, double throttle) {
//    qDebug() << "Received from motor process (DBus):   Motor: " << motorNum << " throttle: " << throttle;
    throttles[motorNum] = (throttle < 0.1) ? throttle=0 : throttle=throttle;
}

void MulticopterSimulator::sendDbusMessage(QString msg, int type) {
    QDBusMessage out = QDBusMessage::createSignal("/", "edu.vt.ece.msg", "msg");
    out << msg << type;
    bus.send(out);
}

void MulticopterSimulator::sendAngleUpdate(double targetPitch, double targetRoll, double targetAltitude) {
    QDBusMessage update = QDBusMessage::createSignal("/", "edu.vt.ece.updateAngles", "updateAngles");
    update << curPitch << curRoll << curAltitude << targetPitch << targetRoll << targetAltitude;
    bus.send(update);
}

double deg2rad(double deg) {
    return deg*(PI/180);
}

void MulticopterSimulator::updatePhysics() {
    double motorPosition = 0;

    for(int i=0; i<4; i++)
        throttles[i] -= 20;

    double curPitchRad = deg2rad(curPitch);
    double curRollRad  = deg2rad(curRoll);

    for(int i=0; i<numMotors; i++) {
        motorPosition = (((360/numMotors) * (i))) * (PI/180);
        curPitch += (sin(motorPosition)*throttles[i]*cos(curPitchRad)*cos(curRollRad));
        curRoll  += (cos(motorPosition)*throttles[i]*cos(curPitchRad)*cos(curRollRad));
  //      curAltitude += throttles[i]*cos(curPitch)*cos(curRoll) * .1;
    }
    curPitch *= .2;
    curRoll  *= .2;

 //   curPitch += (throttles[1] - throttles[3]) * .01;
 //   curRoll  += (throttles[0] - throttles[2]) * .01;
    curAltitude += (throttles[1]*cos(curPitchRad)*cos(curRollRad) + throttles[1]*cos(curPitchRad)*cos(curRollRad) +
                    throttles[2]*cos(curPitchRad)*cos(curRollRad) + throttles[3]*cos(curPitchRad)*cos(curRollRad)) * .01;

    qDebug() << "Current throttles: " << throttles[0] << throttles[1] << throttles[2] << throttles[3];
    qDebug() << "\tTargets (" << targetPitch << "," << targetRoll << ") Current (" << curPitch << "," << curRoll << ")"
            << "Alt (" << targetAltitude << "," << curAltitude << ")";
    sendAngleUpdate(targetPitch, targetRoll, targetAltitude);
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

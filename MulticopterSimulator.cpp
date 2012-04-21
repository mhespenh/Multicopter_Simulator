/***********************************************************************
  Group 3 ECE4574 Spring 2012
  Project 3 - Multicopter Simulator
    Multicopter_Simulator - Control process for simulator (master branch)

  Revisions:
    03/21/2012 - Initial version (master branch)
    03/23/2012 - Implemented dbus communication correctly
    04/20/2012 - Ready to go
***********************************************************************/

#include "MulticopterSimulator.h"
#include <stdio.h>
#include <QDebug>
#include <QDBusArgument>

#define DEBUG

double deg2rad(double deg) {
    return deg*(PI/180);
}

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

    numMotors = numProcs;
    procCount = 0;
    dt = 0.01;     //simulation time step (in ms)

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
        throttles[i] = 0;
        //connect exiting slot at some point
    }

    targetPitch = 0;     //in degrees
    targetRoll  = 0;     //in degrees
    targetAltitude = 0; //in meters
    curPitch = 0;
    curRoll = 0;
    curAltitude = 0;
    cur_x = 0;
    cur_y = 0;
    prev_x = 0;
    prev_y = 0;
    v_x = 0;
    v_y = 0;

    mass = 0.5;       //in kg
    gravity = 9.8;  //in m/s^2
    armLength = 1;

    //wait for all processes to start
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

void MulticopterSimulator::setGravity(float grav) {
    this->gravity = grav;
}

void MulticopterSimulator::setArmLength(float length) {
    this->armLength = length;
}

void MulticopterSimulator::setMass(float mass) {
    this->mass = mass;
}

void MulticopterSimulator::recvMessage(QString msg) {
    qDebug() << "Received from motor process (DBus):   " << msg;
}

void MulticopterSimulator::recvUpdate(int motorNum, double throttle) {
//    qDebug() << "Received from motor process (DBus):   Motor: " << motorNum << " throttle: " << throttle;
    throttles[motorNum] = throttle;
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

void MulticopterSimulator::updatePhysics() {
    double motorPosition = 0;

#ifdef DEBUG
    qDebug() << "Current throttles: " << throttles[0] << throttles[1] << throttles[2] << throttles[3];
    qDebug() << "\tTargets (" << targetPitch << "," << targetRoll << ") Current (" << curPitch << "," << curRoll << ")"
            << "Alt (" << targetAltitude << "," << curAltitude << ")";
#endif

    for(int i=0; i<4; i++)
        throttles[i] -= mass*gravity;

    double curPitchRad = deg2rad(curPitch);
    double curRollRad  = deg2rad(curRoll);

    for(int i=0; i<numMotors; i++) {
        motorPosition = (((360/numMotors) * (i))) * (PI/180);
        curPitch += dt*(sin(motorPosition)*throttles[i]*cos(curPitchRad)*cos(curRollRad));
        curRoll  += dt*(cos(motorPosition)*throttles[i]*cos(curPitchRad)*cos(curRollRad));
        curAltitude += dt*throttles[i]*cos(curPitchRad)*cos(curRollRad);
    }

    curAltitude = curAltitude < 0 ? 0 : curAltitude;
    targetPitch = targetPitch > 30 ? 30 : targetPitch;
    targetRoll = targetRoll > 30 ? 30 : targetRoll;

    if( curAltitude < armLength ) {
        targetPitch = 0;
        targetRoll = 0;
    }

    updatePosition();
    sendAngleUpdate(targetPitch, targetRoll, targetAltitude);
}

void MulticopterSimulator::updatePosition() {
    prev_x = cur_x;
    prev_y = cur_y;
    cur_x += curRoll * dt;
    cur_y += curPitch * dt;
    v_x = (v_x + ((cur_x-prev_x)/dt))/2;
    v_y = (v_y + ((cur_y-prev_y)/dt))/2;
#ifdef DEBUG
    qDebug() << "x,y " << cur_x << "," << cur_y << " vx,vy" << v_x << "," << v_y;
#endif
}

void MulticopterSimulator::processStarted(QString reply) {
    qDebug() << "Received from motor process (DBus) Start:   " << reply;
    procCount++;
}

void MulticopterSimulator::processExit(int foo, QProcess::ExitStatus bar) {
    qDebug() << "Motor Process Exited with status:     " << bar << "/" << foo;
    qDebug() << "So we'll quit too.";
    QCoreApplication::quit();
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

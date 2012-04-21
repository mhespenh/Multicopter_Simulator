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

//#define DEBUG

double deg2rad(double deg) {
    return deg*(PI/180);
}

MulticopterSimulator::MulticopterSimulator(int numProcs, QObject *parent) :
    QObject(parent), bus(QDBusConnection::sessionBus()), sharedMem("PUBLIC_SHARED_MEM")
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
    dt = 0.01;     //simulation time step (in SECONDS) default is 10ms

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
    targetAltitude = 10; //in meters
    curPitch = 0;
    curRoll = 0;
    curAltitude = 0;
    target_x = 0;
    target_y = 0;
    cur_x = 0;
    cur_y = 0;
    prev_x = 0;
    prev_y = 0;
    prev_alt = 0;
    v_x = 0;
    v_y = 0;
    v_z = 0;

    tx = 0;
    ty = 0;

    mass = 1;       //in kg
    gravity = 9.8;  //in m/s^2
    armLength = 1; //really won't work for values larger than around 5
    
    if (sharedMem.attach()) {
        sharedMem.detach();
    }
    else {
        if(!sharedMem.create(sizeof(data))) {
            qDebug() << sharedMem.errorString();
            qDebug() << "Unable to create shared memory space";
            while(1);
        }
     }

    theAI.setArmLength(armLength);
    qDebug() << theAI.setDestination(target_x, target_y);

    QTimer *aiTimer = new QTimer(this);
    connect(aiTimer, SIGNAL(timeout()), this, SLOT(getAngles()));
    aiTimer->start(dt*1000); //100ms timer

    //timer to trigger physics refresh
    QTimer *physicsTimer = new QTimer(this);
    connect(physicsTimer, SIGNAL(timeout()), this, SLOT(updatePhysics()));
    physicsTimer->start(dt*1000); //10ms timer

    //timer to trigger a write to shared memory
    QTimer *sharedMemTimer = new QTimer(this);
    connect(sharedMemTimer, SIGNAL(timeout()), this, SLOT(writeSharedMem()));
    sharedMemTimer->start(100); //100ms timer

}


MulticopterSimulator::~MulticopterSimulator() {
    qDebug() << "Cleaning up...";
    sharedMem.detach();
    sharedMem.deleteLater();
    delete throttles;
    delete procs;
}

void MulticopterSimulator::getAngles() {
    int sx = 20;
    int sy = 20;
    if( (target_y-cur_y) < 0 ) {
        sy *= -1;
    }
    if( (target_x-cur_x) < 0 ) {
        sy *= -1;
    }
    theAI.getTargetAngles(targetPitch, targetRoll, cur_x, cur_y);
    tx = abs(target_x-cur_x) > 20 ? cur_x+sx : target_x;
    ty = abs(target_y-cur_y) > 20 ? cur_y+sy : target_y;
    theAI.setDestination(tx, ty);
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

void MulticopterSimulator::setTargetPosition(int x, int y) {
    this->target_x = x;
    this->target_y = y;
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


//I've added in some "fake" torque effects here by including an armLength
// scaling value on the pitch and roll calculations.  It makes it quicker
// to react to changes the smaller the arm length, but it will also make it
// more unstable.  Such is life.
void MulticopterSimulator::updatePhysics() {
    double motorPosition = 0;
    double f_grav = mass*gravity*3;

    double curPitchRad = deg2rad(curPitch);
    double curRollRad  = deg2rad(curRoll);

    for(int i=0; i<numMotors; i++) {
        motorPosition = (((360/numMotors) * (i))) * (PI/180);
        curPitch += dt*(sin(motorPosition)*(throttles[i]-f_grav)*cos(curPitchRad)*cos(curRollRad))*(1/armLength);
        curRoll  += dt*(cos(motorPosition)*(throttles[i]-f_grav)*cos(curPitchRad)*cos(curRollRad))*(1/armLength);
        curAltitude += dt*(throttles[i]-f_grav)*cos(curPitchRad)*cos(curRollRad)*.8;
    }

    //limiting- don't let the AI do anything too crazy
    curAltitude = curAltitude < 0 ? 0 : curAltitude;
    targetPitch = targetPitch > 30 ? 30 : targetPitch;
    targetRoll = targetRoll > 30 ? 30 : targetRoll;
/*
    if( curAltitude < armLength ) {
        targetPitch = targetPitch > 5 ? 5 : targetPitch;
        targetRoll = targetRoll > 5 ? 5: targetRoll;
    }
*/

#ifdef DEBUG
    qDebug() << "Current throttles: " << throttles[0] << throttles[1] << throttles[2] << throttles[3];
    qDebug() << "\tTargets (" << targetPitch << "," << targetRoll << ") Current (" << curPitch << "," << curRoll << ")"
            << "Alt (" << targetAltitude << "," << curAltitude << ")"
            << "x,y" << target_x << "," << target_y << " tx,ty " << tx << "," << ty;
#endif
    updatePosition();
    sendAngleUpdate(targetPitch, targetRoll, targetAltitude);
}

void MulticopterSimulator::updatePosition() {
    cur_x += curRoll * dt;
    cur_y += curPitch * dt;
    v_x = (v_x + ((cur_x-prev_x)/dt))/2;
    v_y = (v_y + ((cur_y-prev_y)/dt))/2;
    v_z = (v_z + ((curAltitude-prev_alt)/dt)) / 2;
    prev_x = cur_x;
    prev_y = cur_y;
    prev_alt = curAltitude;
#ifdef DEBUG
    qDebug() << "x,y " << cur_x << "," << cur_y;// << " vx,vy,vz" << v_x << "," << v_y << "," << v_z;
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

void MulticopterSimulator::writeSharedMem() {
    if (!sharedMem.isAttached()) { //if not attached
        if(!sharedMem.attach()) {  //attach
            if(!sharedMem.create(sizeof(data))) { //if that fails, try to create
                qDebug() << sharedMem.errorString(); //if can't create
                qDebug() << "Unable to create shared memory space";
                while(1); //we're screwed
                if(!sharedMem.isAttached()) { //then try to reconnect if no attached
                    qDebug() << sharedMem.errorString(); //if that fails give up
                    qDebug() << "Unable to attach to shared memory space";
                    while(1);
                }
            }
        }
    }
    data* theData;

    sharedMem.lock(); //lock the shared memory mutex
    theData = (data*)sharedMem.data();
    theData->t0 = (int)throttles[0];
    theData->t1 = (int)throttles[1];
    theData->t2 = (int)throttles[2];
    theData->t3 = (int)throttles[3];
    theData->pitch = this->curPitch;
    theData->roll = this->curRoll;
    theData->altitude = this->curAltitude;
    theData->cur_x = (int)this->cur_x;
    theData->cur_y = (int)this->cur_y;
    this->target_x = theData->target_x;
    this->target_y = theData->target_y;
    theData->v_x = this->v_x;
    theData->v_y = this->v_y;
    theData->v_z = this->v_z;
    sharedMem.unlock(); //release mutex lock
#ifdef DEBUG
    qDebug() << "Wrote to Shared Memory: " << theData->t0 << theData->t1 << theData->t2 << theData->t3;
#endif
}

#include "MulticopterSimulator.h"
#include <QDebug>

MulticopterSimulator::MulticopterSimulator(QObject *parent) :
    QObject(parent)
{
    proc = new QProcess(this);
    QObject::connect(proc, SIGNAL(readyReadStandardError()), this, SLOT(updateError()));
    QObject::connect(proc, SIGNAL(readyReadStandardOutput()), this, SLOT(updateText()));
    QObject::connect(proc, SIGNAL(finished(int,QProcess::ExitStatus)), this, SLOT(updateExit(int,QProcess::ExitStatus)));
    proc->start("/Users/mhespenh/SimEngine/SimEngine");
}

void MulticopterSimulator::updateError() {
    QByteArray data = proc->readAllStandardError();
    qDebug() << QString(data);

}

void MulticopterSimulator::updateExit(int foo,QProcess::ExitStatus bar) {
    qDebug() << "Exited\n" << foo << "\t" << bar;
}

void MulticopterSimulator::updateText() {
    QByteArray data = proc->readAllStandardOutput();
    qDebug() << QString(data);
}

void MulticopterSimulator::writeData(QByteArray data) {
    proc->write(data);
}

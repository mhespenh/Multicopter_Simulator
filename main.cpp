/***********************************************************************
  Group 3 ECE4574 Spring 2012
  Project 3 - Multicopter Simulator
    Multicopter_Simulator - Control process for simulator (master branch)

  Revisions:
    03/21/2012 - Initial version (master branch)
***********************************************************************/

#include <QtCore/QCoreApplication>
#include <MulticopterSimulator.h>
#include <QDebug>
#include <QtDBus/QtDBus>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    if (!QDBusConnection::sessionBus().isConnected()) {
         fprintf(stderr, "Cannot connect to the D-Bus session bus.\n"
                 "To start it, run:\n"
                 "\teval `dbus-launch --auto-syntax`\n");
         return 1;
     }

    QDBusInterface iface("edu.vt.ece.simmotor", "/", "", QDBusConnection::sessionBus());
     if (iface.isValid()) {
         QDBusReply<QString> reply = iface.call("recvMessage", "Hello", 45);
         if (reply.isValid()) {
             printf("Reply was: %s\n", qPrintable(reply.value()));
             return 0;
         }

         fprintf(stderr, "Call failed: %s\n", qPrintable(reply.error().message()));
         return 1;
     }

     fprintf(stderr, "%s\n", qPrintable(QDBusConnection::sessionBus().lastError().message()));
 //   MulticopterSimulator sim;
 //   sim.writeData("Hello from another process\n");
 //   return a.exec();
     return 1;
}

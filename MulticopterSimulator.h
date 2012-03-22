#include <QObject>
#include <QProcess>

class MulticopterSimulator : public QObject
{
    Q_OBJECT
public:
    explicit MulticopterSimulator(QObject *parent = 0);
    void writeData(QByteArray data);

signals:

public slots:
    void updateError(void);
    void updateText(void);
    void updateExit(int,QProcess::ExitStatus);

private:
    QProcess* proc;
};

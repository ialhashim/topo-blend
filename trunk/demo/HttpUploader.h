#pragma once
#include <QEventLoop>
#include <QFile>
#include <QNetworkAccessManager>
#include <QHttpMultiPart>
#include <QNetworkReply>

class HttpUploader
{
public:
    static QString upload(QString uploadURL, QString filename)
    {
        QUrl url( uploadURL );
        QNetworkRequest request(url);

        QHttpMultiPart * multiPart = new QHttpMultiPart(QHttpMultiPart::FormDataType);

        QHttpPart previewPathPart;
        previewPathPart.setHeader(QNetworkRequest::ContentDispositionHeader, QVariant("form-data; name=\"filepath\""));
        previewPathPart.setBody(filename.toLatin1());

        QHttpPart previewFilePart;
        previewFilePart.setHeader(QNetworkRequest::ContentTypeHeader,    QVariant("application/unknown"));
        previewFilePart.setHeader(QNetworkRequest::ContentDispositionHeader, QVariant("form-data; name=\"userfile\"; filename=\"" + filename + "\""));

        QFile file( filename );
        file.open(QIODevice::ReadOnly);
        previewFilePart.setBodyDevice(&file);

        multiPart->append( previewPathPart );
        multiPart->append( previewFilePart );

        QNetworkAccessManager manager;
        QNetworkReply * netReply = manager.post(request, multiPart);

        // Block until 'finished'
        QEventLoop loop;
        QObject::connect(netReply, SIGNAL(finished()), &loop, SLOT(quit()));
        loop.exec();
        QString response = netReply->readAll();

        // Clean up
        netReply->deleteLater();
        multiPart->deleteLater();

        return response;
    }
};

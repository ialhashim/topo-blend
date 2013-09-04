#pragma once
#include <QObject>
#include <QMap>
#include <QVector>
#include <QStack>
#include <QString>
#include <QVariant>
#include <QDebug>
#include <QElapsedTimer>
#include <QtConcurrentRun>
#include <QDir>
#include <QFileInfo>
#include <QtOpenGL>
#include <QGLWidget>
#include <QPainter>

#include <QGraphicsScene>
#include <QGraphicsItem>

typedef QMap<QString, QVariant> PropertyMap;
typedef QMap<QString, PropertyMap> DatasetMap;

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

#include "StructureGraph.h"

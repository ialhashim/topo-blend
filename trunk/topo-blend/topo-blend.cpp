#include "topo-blend.h"
#include "StarlabMainWindow.h"
#include "StarlabDrawArea.h"
#include "interfaces/ModePluginDockWidget.h"
#include "../segment/CustomDrawObjects.h"

topoblend::topoblend(){
    widget = NULL;
}

void topoblend::create()
{
    if(!widget)
    {
        ModePluginDockWidget * dockwidget = new ModePluginDockWidget(mainWindow());
        widget = new topo_blend_widget(this);
        dockwidget->setWidget(widget);
        dockwidget->setWindowTitle(widget->windowTitle());
        mainWindow()->addDockWidget(Qt::RightDockWidgetArea,dockwidget);
    }
}

void topoblend::decorate()
{

}

Q_EXPORT_PLUGIN(topoblend)

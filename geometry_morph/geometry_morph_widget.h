#ifndef GEOMETRY_MORPH_WIDGET_H
#define GEOMETRY_MORPH_WIDGET_H

#include <QWidget>

class geometry_morph;

namespace Ui {
	class geometry_morph_widget;
}

class geometry_morph_widget : public QWidget
{
	Q_OBJECT

public:
    explicit geometry_morph_widget(geometry_morph* geo_morph, QWidget *parent = 0);
	~geometry_morph_widget();

	public slots:
		void renderViewer();
		void renderAnimation();
		void loadAnimationModel();

private:
	Ui::geometry_morph_widget *ui;
	geometry_morph * gm;

};

#endif // GEOMETRY_MORPH_WIDGET_H

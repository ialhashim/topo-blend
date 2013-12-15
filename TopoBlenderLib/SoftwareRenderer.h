#pragma once
#include <iostream>     // std::cout
#include <QImage>
#include <QPainter>
#include <QPoint>

#include <Eigen/Core>
#include <Eigen/Geometry>

#define RADIANS(deg)    ((deg)/180.0 * M_PI)
#ifndef M_PI_2
#define M_PI    3.14159265358979323846264338328
#define M_PI_2  1.57079632679489661923132169164
#endif
#define cot(x) (tan(M_PI_2 - x))

typedef QPair<Eigen::Vector3d, Eigen::Vector3d> PairPoints;
typedef Eigen::Matrix<double,4,4,Eigen::RowMajor> Matrix4;

double angle = 0.0;

namespace SoftwareRenderer{

	struct DepthCompare{
		bool operator()(const Eigen::Vector3d & a, const Eigen::Vector3d & b) const{
			return a[2] < b[2];
		}
	};

	void horizontalLine(Eigen::MatrixXd & m,  int xpos, int ypos, int x1, double color){
		for(int x = xpos; x <= x1; ++x){
			if(x < 0 || x > m.cols() - 1 || ypos < 0 || ypos > m.rows() - 1) 
				continue;
			m(ypos,x) = color;
		}
	}

	void plot4points(Eigen::MatrixXd & buffer, int cx, int cy, int x, int y, double color){
		horizontalLine(buffer, cx - x, cy + y, cx + x, color);
		if (x != 0 && y != 0)
			horizontalLine(buffer, cx - x, cy - y, cx + x, color);
	}

	void circle(Eigen::MatrixXd & buffer, int cx, int cy, int radius, double color){
		int error = -radius;
		int x = radius;
		int y = 0;

		while (x >= y){
			int lastY = y;

			error += y;
			++y;
			error += y;

			plot4points(buffer, cx, cy, x, lastY, color);

			if (error >= 0){
				if (x != lastY)
					plot4points(buffer, cx, cy, lastY, x, color);
				error -= x;
				--x;
				error -= x;
			}
		}
	}


	Eigen::Vector3d TransformCoordinates(const Eigen::Vector3d & vector, const Matrix4 & m) {
		double x = (vector[0] * m(0)) + (vector[1] * m(4)) + (vector[2] * m(8)) + m(12);
		double y = (vector[0] * m(1)) + (vector[1] * m(5)) + (vector[2] * m(9)) + m(13);
		double z = (vector[0] * m(2)) + (vector[1] * m(6)) + (vector[2] * m(10)) + m(14);
		double w = (vector[0] * m(3)) + (vector[1] * m(7)) + (vector[2] * m(11)) + m(15);
		return Eigen::Vector3d(x / w, y / w, z / w);
	};

	Matrix4 CreateProjectionMatrix(double fov_degrees, double aspect_ratio, double zNear = 1.0, double zFar = 10.0)
	{
		double fov = RADIANS(fov_degrees * 0.5);

		Matrix4 matrix = Matrix4::Identity();

		double t = 1.0 / (std::tan(fov * 0.5));
		matrix(0) = t / aspect_ratio;
		matrix(1) = matrix(2) = matrix(3) = 0.0;
		matrix(5) = t;
		matrix(4) = matrix(6) = matrix(7) = 0.0;
		matrix(8) = matrix(9) = 0.0;
		matrix(10) = -zFar / (zNear - zFar);
		matrix(11) = 1.0;
		matrix(12) = matrix(13) = matrix(15) = 0.0;
		matrix(14) = (zNear * zFar) / (zNear - zFar);

		return matrix;
	}

	Matrix4 CreateWorldMatrix(double transX = 0, double transY = 0, double transZ = 0)
	{
		Matrix4 wmat = Matrix4::Identity();
		wmat.row(3) = Eigen::Vector4d(transX, transY, transZ, 1);
		return wmat;
	}

	Matrix4 CreateViewMatrix( Eigen::Vector3d eye = Eigen::Vector3d(3,-3,3), Eigen::Vector3d target = Eigen::Vector3d(0,0,0), Eigen::Vector3d up = Eigen::Vector3d(0,0.0,1.0)  )
	{
		Matrix4 vmat = Matrix4::Identity();

		Eigen::Vector3d zAxis = (target - eye);
		zAxis.normalize();
		Eigen::Vector3d xAxis = (up.cross(zAxis));
		xAxis.normalize();
		Eigen::Vector3d yAxis = (zAxis.cross(xAxis));
		yAxis.normalize();
		double ex = -(xAxis.dot(eye));
		double ey = -(yAxis.dot(eye));
		double ez = -(zAxis.dot(eye));

		vmat << xAxis[0], xAxis[1], xAxis[2],  ex,
			yAxis[0], yAxis[1], yAxis[2],  ey,
			zAxis[0], zAxis[1], zAxis[2],  ez,
			0		, 0	  , 0		,  1;

		vmat = Matrix4( vmat.transpose() );

		return vmat;
	}

	Eigen::Vector3d Project(const Eigen::Vector3d & coord, const Matrix4 & transMat, const Eigen::Vector2d & viewArea)
	{
		double PixelWidth = viewArea[0];
		double PixelHeight = viewArea[1];

		// transforming the coordinates
		Eigen::Vector4d coord4(coord[0], coord[1], coord[2], 1);

		//Eigen::Vector4d point = transMat * coord4;
		Eigen::Vector3d point = TransformCoordinates(coord, transMat);

		// The transformed coordinates will be based on coordinate system
		// starting on the center of the screen. But drawing on screen normally starts
		// from top left. We then need to transform them again to have x:0, y:0 on top left.
		Eigen::Vector2d screen( (point[0] * PixelWidth) + (PixelWidth * 0.5), (-point[1] * PixelHeight) + (PixelHeight * 0.5));
		return Eigen::Vector3d( screen[0], screen[1], -point[2] );
	}

	QVector< PairPoints > cube()
	{
		QVector< PairPoints > face;

		QVector< PairPoints > faces;

		double len = 0.5;

		face.push_back(PairPoints(Eigen::Vector3d(  len, -len, len ),Eigen::Vector3d(  len,  len, len )));
		face.push_back(PairPoints(Eigen::Vector3d(  len,  len, len ),Eigen::Vector3d( -len,  len, len )));
		face.push_back(PairPoints(Eigen::Vector3d( -len,  len, len ),Eigen::Vector3d( -len, -len, len )));
		face.push_back(PairPoints(Eigen::Vector3d( -len, -len, len ),Eigen::Vector3d(  len, -len, len )));

		for(int i = 0; i < 4; i++)
		{
			QVector<PairPoints> newFace;

			for(int e = 0; e < 4; e++)
			{
				PairPoints line = face[e];

				line.first = Eigen::AngleAxisd(i * 0.5 * M_PI, Eigen::Vector3d::UnitX()) * line.first;
				line.second = Eigen::AngleAxisd(i * 0.5 * M_PI, Eigen::Vector3d::UnitX()) * line.second;

				newFace.push_back(line);
			}

			foreach(PairPoints line, newFace) faces.push_back(line);
		}

		return faces;
	}

	void render( QVector< PairPoints > lines, QImage & img, int width, int height )
	{
		// Rendering device
		img = QImage(width, height, QImage::Format_RGB32);
		QPainter painter(&img);
		painter.setRenderHint(QPainter::Antialiasing);
		painter.setRenderHint(QPainter::HighQualityAntialiasing);
		painter.fillRect(img.rect(), Qt::white);
		painter.setPen(QPen(Qt::black, 1));

		// Camera and projection
		Eigen::Vector2d viewArea( img.width(), img.height() );
		Matrix4 pmat = CreateProjectionMatrix( 80, double(img.width()) / img.height() );
		Matrix4 wmat = CreateWorldMatrix();
		Matrix4 vmat = CreateViewMatrix();
		Matrix4 transformMatrix = wmat * vmat * pmat;

		// World center
		//Eigen::Vector3d p = Project(Eigen::Vector3d(0,0,0), transformMatrix, viewArea);
		//painter.drawEllipse(QPoint( p[0], p[1] ), 3, 3);

		angle += 0.01;

		foreach(PairPoints line, lines)
		{
			Eigen::Vector3d p0 = Project(line.first, transformMatrix, viewArea);
			Eigen::Vector3d p1 = Project(line.second, transformMatrix, viewArea);

			painter.drawLine(QPoint(p0[0], p0[1]), QPoint(p1[0], p1[1]));
		}
	}

	Eigen::MatrixXd render( QVector< Eigen::Vector3d > points, int width = 32, int height = 32, int pointSize = 1, Eigen::Vector3d translate = Eigen::Vector3d(0,0,0) )
	{
		Eigen::MatrixXd img = Eigen::MatrixXd::Zero( height, width );

		// Camera and projection
		Eigen::Vector2d viewArea( width, height );
		Matrix4 pmat = CreateProjectionMatrix( 80, double(width) / height );
		Matrix4 wmat = CreateWorldMatrix( translate[0], translate[1], translate[2] );
		Matrix4 vmat = CreateViewMatrix();
		Matrix4 transformMatrix = wmat * vmat * pmat;

		double minDepth = DBL_MAX;
		double maxDepth = -DBL_MAX;

		QVector<Eigen::Vector3d> allProjected;

		foreach(Eigen::Vector3d point, points){
			Eigen::Vector3d projected = Project(point, transformMatrix, viewArea);

			// off-screen points check
			int x = projected[0];
			int y = projected[1];
			if(x < 0 || x > width - 1 || y < 0 || y > height - 1) 
				continue;

			allProjected.push_back( projected );

			minDepth = qMin( projected[2], minDepth );
			maxDepth = qMax( projected[2], maxDepth );
		}

		qSort(allProjected.begin(), allProjected.end(), DepthCompare() );

		foreach(Eigen::Vector3d point, allProjected)
		{
			int x = point[0];
			int y = point[1];
			double depthVal = (point[2] - minDepth) / (maxDepth - minDepth);

			if(pointSize == 1)
			{
				img(y,x) = depthVal;
			}
			else if(pointSize == 2)
			{
				img(y,x) = depthVal;

				// cross shape
				img(qMax(y-1, 0),x) = depthVal;
				img(qMin(y+1, height-1),x) = depthVal;
				img(y,qMax(x-1,0)) = depthVal;
				img(y,qMin(x+1, width-1)) = depthVal;
			}
			else
				circle(img, x, y, pointSize, depthVal);
		}

		return img;
	}

	void render( QVector< Eigen::Vector3d > points, QImage & img, int width = 32, int height = 32, int pointSize = 1, Eigen::Vector3d translate = Eigen::Vector3d(0,0,0) )
	{
		img = QImage(width, height, QImage::Format_RGB32);
		QPainter painter(&img);
		painter.fillRect(img.rect(), Qt::white);
		painter.setPen(Qt::NoPen);

		Eigen::MatrixXd mimg = render(points, width, height, pointSize, translate);

		for(int y = 0; y < mimg.rows(); y++){
			for(int x = 0; x < mimg.cols(); x++){
				double depthVal = mimg(y,x);
				if(depthVal == 0) continue;

				int d = (1.0 - depthVal) * 255;
				QColor c( d,d,d );

				painter.setBrush(c);
				painter.drawEllipse(QPoint(x,y), pointSize, pointSize);
			}
		}
	}

	Eigen::MatrixXd vectorToMatrix( const Eigen::VectorXd & v, int width, int height )
	{
		Eigen::MatrixXd M(height, width);
		for(int i = 0; i < v.size(); i++) M(i) = v(i);
		return M;
	}

	QImage matrixToImage( const Eigen::MatrixXd & mimg )
	{
		int width = mimg.rows();
		int height = mimg.cols();

		QImage img(width, height, QImage::Format_ARGB32);
		img.fill(QColor(0,0,0,0));

		QPainter painter(&img);

		painter.setPen(Qt::NoPen);
		painter.setBrush(Qt::NoBrush);

		for(int y = 0; y < mimg.rows(); y++){
			for(int x = 0; x < mimg.cols(); x++){
				double depthVal = mimg(y,x);

				if(depthVal == 0) continue;

				int d = (1.0 - depthVal) * 255;
				QColor c( d,d,d );

				painter.setPen( c );
				painter.drawPoint(QPoint(x,y));
			}
		}

		return img;
	}
}

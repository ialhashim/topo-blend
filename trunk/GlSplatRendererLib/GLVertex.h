#pragma once

struct GLVertex{
	float x, y, z;
	float nx, ny, nz;
	GLVertex(float X=0, float Y=0, float Z=0, float nX=0, float nY=0, float nZ=0) : x(X), y(Y), z(Z), nx(nX), ny(nY), nz(nZ) {}
};

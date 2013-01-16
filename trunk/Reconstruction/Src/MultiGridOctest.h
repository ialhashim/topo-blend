#pragma once

#include <stdarg.h>

class CoredMeshData;

void writeOFF(char* fileName, CoredMeshData* mesh);

template< int Degree >
int Execute( int argc , char* argv[] );


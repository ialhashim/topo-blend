#pragma once

#include <stdarg.h>

class CoredMeshData;

void writeOFF(char* fileName, CoredMeshData* mesh);

template< int Degree >
int Execute( int argc , char* argv[] );

template< int Degree >
int ExecuteMemory( int argc , char* argv[], std::vector< std::vector< float > > & positions, std::vector< std::vector< float > > & normals );

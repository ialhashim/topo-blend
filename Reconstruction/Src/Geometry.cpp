/*
Copyright (c) 2006, Michael Kazhdan and Matthew Bolitho
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer. Redistributions in binary form must reproduce
the above copyright notice, this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution. 

Neither the name of the Johns Hopkins University nor the names of its contributors
may be used to endorse or promote products derived from this software without specific
prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#include "Geometry.h"
#include <stdio.h>
#include <string.h>

///////////////////
// CoredMeshData //
///////////////////

TriangulationEdge::TriangulationEdge(void){pIndex[0]=pIndex[1]=tIndex[0]=tIndex[1]=-1;}
TriangulationTriangle::TriangulationTriangle(void){eIndex[0]=eIndex[1]=eIndex[2]=-1;}

/////////////////////////
// CoredVectorMeshData //
/////////////////////////
CoredVectorMeshData::CoredVectorMeshData( void ) { oocPointIndex = polygonIndex = 0; }
void CoredVectorMeshData::resetIterator ( void ) { oocPointIndex = polygonIndex = 0; }
int CoredVectorMeshData::addOutOfCorePoint(const Point3D<float>& p){
	oocPoints.push_back(p);
	return int(oocPoints.size())-1;
}
int CoredVectorMeshData::addPolygon( const std::vector< CoredVertexIndex >& vertices )
{
	std::vector< int > polygon( vertices.size() );
	for( int i=0 ; i<int(vertices.size()) ; i++ ) 
		if( vertices[i].inCore ) polygon[i] =  vertices[i].idx;
		else                     polygon[i] = -vertices[i].idx-1;
	polygons.push_back( polygon );
	return int( polygons.size() )-1;
}
int CoredVectorMeshData::nextOutOfCorePoint(Point3D<float>& p){
	if(oocPointIndex<int(oocPoints.size())){
		p=oocPoints[oocPointIndex++];
		return 1;
	}
	else{return 0;}
}
int CoredVectorMeshData::nextPolygon( std::vector< CoredVertexIndex >& vertices )
{
	if( polygonIndex<int( polygons.size() ) )
	{
		std::vector< int >& polygon = polygons[ polygonIndex++ ];
		vertices.resize( polygon.size() );
		for( int i=0 ; i<int(polygon.size()) ; i++ )
			if( polygon[i]<0 ) vertices[i].idx = -polygon[i]-1 , vertices[i].inCore = false;
			else               vertices[i].idx =  polygon[i]   , vertices[i].inCore = true;
		return 1;
	}
	else return 0;
}
int CoredVectorMeshData::outOfCorePointCount(void){return int(oocPoints.size());}
int CoredVectorMeshData::polygonCount( void ) { return int( polygons.size() ); }

///////////////////////////
// BufferedReadWriteFile //
///////////////////////////
BufferedReadWriteFile::BufferedReadWriteFile( char* fileName , int bufferSize )
{
	_bufferIndex = 0;
	_bufferSize = bufferSize;
	if( fileName ) strcpy( _fileName , fileName ) , tempFile = false;
#ifdef _WIN32
	else strcpy( _fileName , _tempnam( "." , "foo" ) ) , tempFile = true;
#else // !_WIN32
	else strcpy( _fileName , tempnam( "." , "foo" ) ) , tempFile = true;
#endif // _WIN32
	_fp = fopen( _fileName , "w+b" );
	if( !_fp ) fprintf( stderr , "[ERROR] Failed to open file: %s\n" , _fileName ) , exit( 0 );
	_buffer = (char*) malloc( _bufferSize );
}
BufferedReadWriteFile::~BufferedReadWriteFile( void )
{
	free( _buffer );
	fclose( _fp );
	if( tempFile ) remove( _fileName );
}
void BufferedReadWriteFile::reset( void )
{
	if( _bufferIndex ) fwrite( _buffer , 1 , _bufferIndex , _fp );
	_bufferIndex = 0;
	fseek( _fp , 0 , SEEK_SET );
	_bufferIndex = 0;
	_bufferSize = fread( _buffer , 1 , _bufferSize , _fp );
}
bool BufferedReadWriteFile::write( const void* data , size_t size )
{
	if( !size ) return true;
	char* _data = (char*) data;
	size_t sz = _bufferSize - _bufferIndex;
	while( sz<=size )
	{
		memcpy( _buffer+_bufferIndex , _data , sz );
		fwrite( _buffer , 1 , _bufferSize , _fp );
		_data += sz;
		size -= sz;
		_bufferIndex = 0;
		sz = _bufferSize;
	}
	if( size )
	{
		memcpy( _buffer+_bufferIndex , _data , size );
		_bufferIndex += size;
	}
	return true;
}
bool BufferedReadWriteFile::read( void* data , size_t size )
{
	if( !size ) return true;
	char *_data = (char*) data;
	size_t sz = _bufferSize - _bufferIndex;
	while( sz<=size )
	{
		if( size && !_bufferSize ) return false;
		memcpy( _data , _buffer+_bufferIndex , sz );
		_bufferSize = fread( _buffer , 1 , _bufferSize , _fp );
		_data += sz;
		size -= sz;
		_bufferIndex = 0;
		if( !size ) return true;
		sz = _bufferSize;
	}
	if( size )
	{
		if( !_bufferSize ) return false;
		memcpy( _data , _buffer+_bufferIndex , size );
		_bufferIndex += size;
	}
	return true;
}


///////////////////////
// CoredFileMeshData //
///////////////////////
CoredFileMeshData::CoredFileMeshData( void )
{
	oocPoints = polygons = 0;
	
	oocPointFile = new BufferedReadWriteFile();
	polygonFile = new BufferedReadWriteFile();
}
CoredFileMeshData::~CoredFileMeshData( void )
{
	delete oocPointFile;
	delete polygonFile;
}
void CoredFileMeshData::resetIterator ( void )
{
	oocPointFile->reset();
	polygonFile->reset();
}
int CoredFileMeshData::addOutOfCorePoint( const Point3D< float >& p )
{
	oocPointFile->write( &p , sizeof( Point3D< float > ) );
	oocPoints++;
	return oocPoints-1;
}
int CoredFileMeshData::addPolygon( const std::vector< CoredVertexIndex >& vertices )
{
	int pSize = int( vertices.size() );
	std::vector< int > polygon( pSize );
	for( int i=0 ; i<pSize ; i++ ) 
		if( vertices[i].inCore ) polygon[i] =  vertices[i].idx;
		else                     polygon[i] = -vertices[i].idx-1;

	polygonFile->write( &pSize , sizeof(int) );
	polygonFile->write( &polygon[0] , sizeof(int)*pSize );
	polygons++;
	return polygons-1;
}
int CoredFileMeshData::nextOutOfCorePoint( Point3D< float >& p )
{
	if( oocPointFile->read( &p , sizeof( Point3D< float > ) ) ) return 1;
	else return 0;
}
int CoredFileMeshData::nextPolygon( std::vector< CoredVertexIndex >& vertices )
{
	int pSize;
	if( polygonFile->read( &pSize , sizeof(int) ) )
	{
		std::vector< int > polygon( pSize );
		if( polygonFile->read( &polygon[0] , sizeof(int)*pSize ) )
		{
			vertices.resize( pSize );
			for( int i=0 ; i<int(polygon.size()) ; i++ )
				if( polygon[i]<0 ) vertices[i].idx = -polygon[i]-1 , vertices[i].inCore = false;
				else               vertices[i].idx =  polygon[i]   , vertices[i].inCore = true;
			return 1;
		}
		return 0;
	}
	else return 0;
}
int CoredFileMeshData::outOfCorePointCount( void ){ return oocPoints; }
int CoredFileMeshData::polygonCount( void ) { return polygons; }

//////////////////////////
// CoredVectorMeshData2 //
//////////////////////////
CoredVectorMeshData2::CoredVectorMeshData2( void ) { oocPointIndex = polygonIndex = 0; }
void CoredVectorMeshData2::resetIterator ( void ) { oocPointIndex = polygonIndex = 0; }
int CoredVectorMeshData2::addOutOfCorePoint( const CoredMeshData2::Vertex& v )
{
	oocPoints.push_back( v );
	return int(oocPoints.size())-1;
}
int CoredVectorMeshData2::addPolygon( const std::vector< CoredVertexIndex >& vertices )
{
	std::vector< int > polygon( vertices.size() );
	for( int i=0 ; i<int(vertices.size()) ; i++ ) 
		if( vertices[i].inCore ) polygon[i] =  vertices[i].idx;
		else                     polygon[i] = -vertices[i].idx-1;
	polygons.push_back( polygon );
	return int( polygons.size() )-1;
}
int CoredVectorMeshData2::nextOutOfCorePoint( CoredMeshData2::Vertex& v )
{
	if(oocPointIndex<int(oocPoints.size()))
	{
		v = oocPoints[oocPointIndex++];
		return 1;
	}
	else{return 0;}
}
int CoredVectorMeshData2::nextPolygon( std::vector< CoredVertexIndex >& vertices )
{
	if( polygonIndex<int( polygons.size() ) )
	{
		std::vector< int >& polygon = polygons[ polygonIndex++ ];
		vertices.resize( polygon.size() );
		for( int i=0 ; i<int(polygon.size()) ; i++ )
			if( polygon[i]<0 ) vertices[i].idx = -polygon[i]-1 , vertices[i].inCore = false;
			else               vertices[i].idx =  polygon[i]   , vertices[i].inCore = true;
		return 1;
	}
	else return 0;
}
int CoredVectorMeshData2::outOfCorePointCount(void){return int(oocPoints.size());}
int CoredVectorMeshData2::polygonCount( void ) { return int( polygons.size() ); }

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
prior writften permission. 

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

#ifndef POINT_STREAM_INCLUDED
#define POINT_STREAM_INCLUDED

template< class Real >
class PointStream
{
public:
	virtual ~PointStream( void ){}
	virtual void reset( void ) = 0;
	virtual bool nextPoint( Point3D< Real >& p , Point3D< Real >& n ) = 0;
};


template< class Real >
class MemoryPointStream : public PointStream< Real >
{
    int curPos;
    std::vector< std::vector<Real> > *pos, *normal;
public:
    MemoryPointStream( std::vector< std::vector<Real> > * positions, std::vector< std::vector<Real> > * normals  ){
        pos = positions;
        normal = normals;

        reset();
    }
    ~MemoryPointStream( void ){}
    void reset( void ){
        curPos = 0;
    }
    bool nextPoint( Point3D< Real >& p , Point3D< Real >& n ){
        if(curPos == pos->size())
            return false;
        p[0] = pos->at(curPos)[0];
        p[1] = pos->at(curPos)[1];
        p[2] = pos->at(curPos)[2];
        n[0] = normal->at(curPos)[0];
        n[1] = normal->at(curPos)[1];
        n[2] = normal->at(curPos)[2];
        curPos++;
        return true;
    }
};

template< class Real >
class ASCIIPointStream : public PointStream< Real >
{
	FILE* _fp;
public:
	ASCIIPointStream( const char* fileName );
	~ASCIIPointStream( void );
	void reset( void );
	bool nextPoint( Point3D< Real >& p , Point3D< Real >& n );
};

template< class Real >
class BinaryPointStream : public PointStream< Real >
{
	FILE* _fp;
	static const int POINT_BUFFER_SIZE=1024;
	Real _pointBuffer[ POINT_BUFFER_SIZE * 2 * 3 ];
	int _pointsInBuffer , _currentPointIndex;
public:
	BinaryPointStream( const char* filename );
	~BinaryPointStream( void );
	void reset( void );
	bool nextPoint( Point3D< Real >& p , Point3D< Real >& n );
};

template< class Real >
class PLYPointStream : public PointStream< Real >
{
	char* _fileName;
	PlyFile* _ply;
	int _nr_elems;
	char **_elist;

	int _pCount , _pIdx;
	void _free( void );
public:
	PLYPointStream( const char* fileName );
	~PLYPointStream( void );
	void reset( void );
	bool nextPoint( Point3D< Real >& p , Point3D< Real >& n );
};
#include "PointStream.inl"
#endif // POINT_STREAM_INCLUDED

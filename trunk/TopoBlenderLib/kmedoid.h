/*
Copyright 2012 Teo Mrnjavac <teo@kde.org>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once
#include <QTime>

struct kmedoid {

	struct DataSet{
		QVector< QVector< double > > points;
		DataSet() { }
		DataSet( double * data, int rows, int cols )
		{
			QVector< QVector< double > > pnts(rows);
			int k = 0;
			for(int i = 0; i < rows; i++){
				QVector< double > r;
				for(int j = 0; j < cols; j++)
					r.push_back( data[k++] );
				pnts[i] = r;
			}
			points = pnts;
		}
		QVector< QVector<double> > means;
	};

	static double distance( const QVector< double > &first, const QVector< double > &second )
	{
		if( first.size() != second.size() )
			return std::numeric_limits< double >::quiet_NaN();

		double accumulator = 0;

		for( int i = 0; i < first.size(); ++i )
		{
			accumulator += ( first.at( i ) - second.at( i ) )
				* ( first.at( i ) - second.at( i ) );
		}

		return sqrt( accumulator );
	}

	static int pickMean( const QVector<double> &point, const QVector<QVector<double> > &currentMeans)
	{
		int bestMean = 0;
		double bestMeanDistance = std::numeric_limits< double >::infinity();

		for( int k = 0; k < currentMeans.size(); ++k )
		{
			double newDistance = distance( point, currentMeans.at( k ) );
			if( newDistance < bestMeanDistance )
			{
				bestMeanDistance = newDistance;
				bestMean = k;
			}
		}
		return bestMean;
	}

	static QVector< double > findMedoid(  const DataSet &data, const QVector<int> &clusters, int clusterId, QVector<double> oldMedoid)
	{
		QVector< double > candidateMedoid = oldMedoid;

		double cost = 0.0; //let's compute the cost of the current situation...
		for( int i = 0; i < data.points.size(); ++i )
		{
			if( clusters.at( i ) != clusterId )
				continue;
			cost += distance( oldMedoid, data.points.at( i ) );
		}

		//and now for each non-medoid, we compute the cost as if it were the medoid
		for( int i = 0; i < data.points.size(); ++i )
		{
			if( clusters.at( i ) != clusterId )
				continue;

			double newCost = 0.0;
			for( int j = 0; j < data.points.size(); ++j )
			{
				if( clusters.at( j ) != clusterId )
					continue;

				newCost += distance( data.points.at( i ), data.points.at( j ) );
			}

			if( newCost < cost )
				candidateMedoid = data.points.at( i );
		}
		return candidateMedoid;
	}

	static void run(const DataSet &data, int clusterCount, QVector<int> &clusters, QVector< QVector<double> > &currentMeans)
	{
		qsrand( QTime::currentTime().msec() );  //whoops, separate thread so we have to seed

		//let's randomly pick a few means...
		for( int j = 0; j < clusterCount; ++j )
		{
			int randomRow = qrand() % data.points.size();
			//a vector of points
			currentMeans.append( QVector< double >( data.points.at( randomRow ) ) );
		}

		clusters.resize( data.points.size() );

		//let's assign every point to exactly one mean...
		for( int i = 0; i < data.points.size(); ++i )
		{
			clusters[ i ] = pickMean( data.points.at( i ), currentMeans );
			/*the cluster-index of the meanpoint that minimizes the distance*/
		}

		bool clustersChanged = true;
		int iterations = 0;
		while( clustersChanged )
		{
			clustersChanged = false; //let's assume nothing will change in this iteration

			for( int j = 0; j < clusterCount; ++j )
			{
				currentMeans[ j ] = findMedoid( data, clusters, j, currentMeans.at( j ) );
			}
			for( int i = 0; i < data.points.size(); ++i )
			{
				int newClusterId = pickMean( data.points.at( i ), currentMeans );

				if( newClusterId != clusters[ i ] )
				{
					clustersChanged = true;
					clusters[ i ] = newClusterId;
				}
			}
			iterations++;
		}
	}
};


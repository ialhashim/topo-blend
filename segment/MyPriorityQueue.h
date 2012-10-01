#pragma once
#include <set>
#include "CurveskelHelper.h"

namespace CurveskelTypes{

	class MyPriorityQueue
	{
	public:
		class EdgeCompareFunctor{
		public:
			CurveskelTypes::ScalarEdgeProperty elengths;

			EdgeCompareFunctor(CurveskelModel* skel){
				elengths = skel->edge_property<CurveskelTypes::Scalar>("e:length");
			}

			bool operator() (const CurveskelTypes::Edge& lhs, const CurveskelTypes::Edge& rhs) const{
				CurveskelTypes::Scalar p0 = elengths[lhs];
				CurveskelTypes::Scalar p1 = elengths[rhs];
				return (p0 == p1) ? (lhs.idx()<rhs.idx()) : (p0 < p1);
			}
		};   
		EdgeCompareFunctor compareFunctor;
		std::set<CurveskelTypes::Edge, EdgeCompareFunctor> set;

		MyPriorityQueue(CurveskelModel* skel) : compareFunctor(skel), set(compareFunctor){}

		void insert(CurveskelTypes::Edge edge, CurveskelTypes::Scalar length){
			compareFunctor.elengths[edge] = length;
			set.insert(edge);
		}

		CurveskelTypes::Edge pop(){
			CurveskelTypes::Edge e = *(set.begin());
			set.erase(e);
			compareFunctor.elengths[e] = -1.0;
			return e;
		}

		bool update(CurveskelTypes::Edge edge, CurveskelTypes::Scalar /*length*/){
			// erase the edge
			int nerased = set.erase(edge);

			if( nerased != 1 ){
				qDebug() << "I was supposed to erase one element, but I erased: " << nerased; 
				return false;  
			}

			// re-insert it
			set.insert(edge);

			return true;
		}

		bool empty(){ return set.empty(); }

		bool has(CurveskelTypes::Edge edge){ return set.find(edge) != set.end(); }
	};

}

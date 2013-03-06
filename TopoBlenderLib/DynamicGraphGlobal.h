#pragma once
#include <QDebug>
#include <QString>
#include <QMap>

// Flags
enum NODE_STATE{ ACTIVE, SLEEP, DONE, DISCONNECTED };
typedef QVector<QVariant> Flags;

/// Edges
struct SimpleEdge{
    int n[2];
    SimpleEdge(int n1 = -1, int n2 = -1){ n[0] = n1 < n2 ? n1 : n2; n[1] = n1 > n2 ? n1 : n2; }
    bool operator== ( const SimpleEdge & other ) const{
        return (n[0] == other.n[0] && n[1] == other.n[1]);
    }
    bool hasNode(int node_index){
        return n[0] == node_index || n[1] == node_index;
    }
    int otherNode(int node_index){
        return n[0] == node_index ? n[1] : n[0];
    }
    bool operator< (const SimpleEdge & other) const{
        if(this->n[0] <= other.n[0] && this->n[1] < other.n[1])
            return true;
        else
            return false;
    }
};

namespace DynamicGraphs
{
	/// Nodes
	struct SimpleNode{
		int idx;
		SimpleNode(int id = -1):idx(id){}

		// Properties
		QMap<QString, QVariant> property;
		QString str(QString propertyName){ return property[propertyName].toString(); }
		double val(QString propertyName){ return property[propertyName].toDouble(); }
		QVariant set(QString propertyName, QVariant value) { return property[propertyName] = value; }
		bool has(QString propertyName){ return property.contains(propertyName); }
	};

	// Properties
	typedef QMap<QString, QVariant> Properties;
	static inline Properties noProperties(){ Properties p; return p; }
	static inline Properties SingleProperty(QString name, QString value){
		Properties p;
		p[name] = value;
		return p;
	}
	static QDebug operator<<(QDebug dbg, const SimpleNode &n){
		dbg.nospace() << "Node:\n";
		QMapIterator<QString, QVariant> i(n.property);
		while (i.hasNext()) {
			i.next();
			dbg.nospace() << "  [" << i.key() << "] \t= " << i.value() << "\n"; 
		}
		return dbg.space();
	}

	enum EdgeType{ ANY_EDGE, SAME_SHEET, SAME_CURVE, CURVE_SHEET };

	/// Graph state
	struct GraphState
	{
		// Nodes info
		int numSheets;
		int numCurves;
		int numNodes(){ return numSheets + numCurves; }

		// Edges info
		int numCurveEdges;
		int numSheetEdges;
		int numMixedEdges;
		int numEdges(){ return numCurveEdges + numSheetEdges + numMixedEdges; }

		// Display info
		void print()
		{
			qDebug() << "\n\nState:";
			qDebug() << " Nodes  # " << numNodes();
			qDebug() << " Sheets # " << numSheets;
			qDebug() << " Curves # " << numCurves;
			qDebug() << " Edges  # " << numEdges();
			qDebug() << "  Type (curve-curve)  # " << numCurveEdges;
			qDebug() << "  Type (curve-sheet)  # " << numMixedEdges;
			qDebug() << "  Type (sheet-sheet)  # " << numSheetEdges;
		}

		void printShort()
		{
			qDebug() << QString("[%1,%2,%3,%4,%5]").arg(numSheets).arg(numCurves).arg(numCurveEdges).arg(numMixedEdges).arg(numSheetEdges);
		}

		bool equal(const GraphState & other) {
			return numSheets		== other.numSheets 
				&& numCurves		== other.numCurves 
				&& numCurveEdges	== other.numCurveEdges 
				&& numSheetEdges	== other.numSheetEdges 
				&& numMixedEdges	== other.numMixedEdges;
		}

		bool isZero(){
			return (numNodes() + numEdges()) == 0;
		}

		bool isZeroNodes(){
			return numNodes() == 0;
		}
	};
}

// Generate subset combinations
template <typename Iterator>
bool next_combination(const Iterator first, Iterator k, const Iterator last)
{
	/* Credits: Mark Nelson http://marknelson.us */
	if ((first == last) || (first == k) || (last == k))
		return false;
	Iterator i1 = first;
	Iterator i2 = last;
	++i1;
	if (last == i1)
		return false;
	i1 = last;
	--i1;
	i1 = k;
	--i2;
	while (first != i1)
	{
		if (*--i1 < *i2)
		{
			Iterator j = k;
			while (!(*i1 < *j)) ++j;
			std::iter_swap(i1,j);
			++i1;
			++j;
			i2 = k;
			std::rotate(i1,j,last);
			while (last != j)
			{
				++j;
				++i2;
			}
			std::rotate(k,i2,last);
			return true;
		}
	}
	std::rotate(first,k,last);
	return false;
}

// Generate all permutations of a vector
static inline std::vector< std::vector<int> > all_permutation( std::vector<int> data )
{
	std::vector< std::vector<int> > result;
	do { result.push_back( data ); } while ( next_permutation (data.begin(), data.end()) );
	return result;
}

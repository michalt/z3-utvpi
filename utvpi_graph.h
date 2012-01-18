#ifndef UTVPI_GRAPH_H
#define UTVPI_GRAPH_H

#include <unordered_map>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#include "z3.h"

#include "common.h"

/*
 * Graph with all the inequalities. The template parameter T is used for
 * defining the domain.
 */
template <typename T>
class UtvpiGraph {
  public:
    UtvpiGraph() : graph_(), vertex_map_(), logs_(),
                   special_vertex_(boost::add_vertex(graph_)) {
      logs_.push_front(new std::list<Rollback *>());
    }

    void Print();

    void AddInequality(Sign a, VarId x, T c);
    void AddInequality(Sign a, VarId x, Sign b, VarId y, T c);

    void Push();
    void Pop();
    void Reset();

    // Should be defined by the classes that inherit from this one.
    // virtual bool CheckSat() = 0;

  protected:
    /* Forward declaration for nested class. */
    class Rollback;

    /*
     * Typedefs
     */

    typedef typename boost::adjacency_list<boost::vecS,
                                           boost::vecS,
                                           boost::directedS,
                                           SignedVarId,
                                           T> Graph;

    typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
    typedef typename boost::graph_traits<Graph>::edge_descriptor   Edge;
    typedef typename boost::graph_traits<Graph>::vertex_iterator   VertexIter;
    typedef typename boost::graph_traits<Graph>::edge_iterator     EdgeIter;
    typedef typename boost::graph_traits<Graph>::out_edge_iterator OutEdgeIter;

    typedef std::unordered_map<SignedVarId, Vertex> VertexMap;
    typedef std::list< std::list<Rollback *> *>     Logs;

    /*
     * Methods
     */

    UtvpiGraph(const UtvpiGraph<T> &);
    UtvpiGraph<T> operator=(const UtvpiGraph<T> &);

    Vertex LookupAddVertex(Sign, VarId x);
    Edge UpdateAddEdge(Vertex src, T weight, Vertex trg);

    bool FindEdge(Vertex src, Vertex trg, Edge &edge);
    void SetEdge(Vertex src, T weight, Vertex trg);
    void RemoveEdge(Vertex src, Vertex trg);

    void AddEdgeRollback(Vertex src, Vertex trg);
    void AddWeightRollback(Vertex src, T weight, Vertex trg);

    /*
     * Members
     */

    Graph graph_;
    VertexMap vertex_map_;
    Logs logs_;
    Vertex special_vertex_;

    /*
     * Nested Rollback
     */

    class Rollback {
      public:
        virtual void Execute(UtvpiGraph<T> &) = 0;
	virtual ~Rollback() { };
    };

    class EdgeRollback : public Rollback {
      public:
        EdgeRollback(Vertex s, Vertex t) : s_(s), t_(t) { }
        void Execute(UtvpiGraph<T>& graph) {
          std::cout << "EdgeRollback" << std::endl;
          graph.RemoveEdge(s_, t_);
        }
      private:
        Vertex s_, t_;
    };

    class WeightRollback : public Rollback {
      public:
        WeightRollback(Vertex s, T w, Vertex t) : s_(s), t_(t), w_(w) { }
        void Execute(UtvpiGraph<T>& graph) {
          std::cout << "WeightRollback" << std::endl;
          graph.SetEdge(s_, w_, t_);
        }
      private:
        Vertex s_, t_;
        T w_;
    };

};

#include "utvpi_graph-inl.h"

#endif /* UTVPI_GRAPH_H */

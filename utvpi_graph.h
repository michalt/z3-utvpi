#ifndef UTVPI_GRAPH_H
#define UTVPI_GRAPH_H

#include <unordered_map>
#include <memory>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/bellman_ford_shortest_paths.hpp>
#include <boost/graph/breadth_first_search.hpp>

#include "z3.h"

#include "common.h"
#include "reason.h"

/*
 * Graph with all the inequalities. The template parameter T is used for
 * defining the domain.
 */
template <typename T>
class UtvpiGraph {
  public:
    UtvpiGraph() : graph_(), vertex_map_(), logs_(),
        special_vertex_(boost::add_vertex(graph_)), reasons_() {
      logs_.push_front(new std::list<Rollback *>());
    }

    void Print();

    void AddEquality(VarId var_x, VarId var_y);

    void AddInequality(Sign a, VarId x, T c);
    void AddInequality(Sign a, VarId x, Sign b, VarId y, T c);

    void Push();
    void Pop();
    void Reset();

  protected:
    /* Forward declarations for nested classes. */
    class Rollback;

    /*
     * Typedefs
     */

    typedef typename boost::adjacency_list< boost::vecS,
                                            boost::vecS,
                                            boost::directedS,
                                            SignedVarId,
                                            T > Graph;

    typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
    typedef typename boost::graph_traits<Graph>::edge_descriptor   Edge;
    typedef typename boost::graph_traits<Graph>::vertex_iterator   VertexIter;
    typedef typename boost::graph_traits<Graph>::edge_iterator     EdgeIter;
    typedef typename boost::graph_traits<Graph>::out_edge_iterator OutEdgeIter;

    typedef std::unordered_map<SignedVarId, Vertex> VertexMap;
    typedef std::list< std::list<Rollback *> *>     Logs;
    typedef std::unordered_map< std::pair<Vertex, Vertex>,
                                ReasonPtr > ReasonMap;

    /*
     * Methods
     */

    UtvpiGraph(const UtvpiGraph<T> &);
    UtvpiGraph<T> operator=(const UtvpiGraph<T> &);

    Vertex LookupAddVertex(Sign, VarId x);
    Edge UpdateAddEdge(Vertex src, T weight, Vertex trg, ReasonPtr reason);

    bool FindEdge(Vertex src, Vertex trg, Edge &edge);
    void SetEdge(Vertex src, T weight, Vertex trg);
    void RemoveEdge(Vertex src, Vertex trg);

    void AddEdgeRollback(Vertex src, Vertex trg, ReasonPtr reason);
    void AddWeightRollback(Vertex src, T weight, Vertex trg, ReasonPtr reason);

    std::list<ReasonPtr>*
    GetNegativeCycle(const Edge &start_edge, const std::vector<Vertex> &parent);

    std::list<ReasonPtr>*
    GetNegativeCycle(const Vertex &start_vertex, const Vertex &neg_start_vertex,
        const Graph &tmp_graph, const ReasonMap &tmp_reasons);

    /*
     * Members
     */

    Graph graph_;
    VertexMap vertex_map_;
    Logs logs_;
    Vertex special_vertex_;
    ReasonMap reasons_;

    /*
     * Rollback
     */

    class Rollback {
      public:
        virtual void Execute(UtvpiGraph<T> &) = 0;
	virtual ~Rollback() { };
    };

    class EdgeRollback : public Rollback {
      public:
        EdgeRollback(Vertex s, Vertex t, ReasonPtr r) : s_(s), t_(t), r_(r) { }
        void Execute(UtvpiGraph<T>& graph) {
          graph.RemoveEdge(s_, t_);
          graph.reasons_[std::make_pair(s_, t_)] = r_;
        }
      private:
        Vertex s_, t_;
        ReasonPtr r_;
    };

    class WeightRollback : public Rollback {
      public:
        WeightRollback(Vertex s, T w, Vertex t, ReasonPtr r) : s_(s), t_(t), w_(w), r_(r) { }
        void Execute(UtvpiGraph<T>& graph) {
          graph.SetEdge(s_, w_, t_);
          graph.reasons_[std::make_pair(s_, t_)] = r_;
        }
      private:
        Vertex s_, t_;
        T w_;
        ReasonPtr r_;
    };


    struct NegVisitor : public boost::bellman_visitor<> {
      NegVisitor() : neg_edge(new Edge()) { }

      /*
       * Called in the second part of the algorithm whene some edge is still not
       * minimized. This means that there is a negative weight cycle in the graph.
       */
      void edge_not_minimized(Edge e, Graph &g) {
        *neg_edge = e;
      }

      Edge *neg_edge;
    };

    class BfsVisitor : public boost::default_bfs_visitor {
      public:
        BfsVisitor(std::vector<Vertex> &p, Vertex t) : parent_(p), target_(t) { }

        void SetTarget(Vertex t) { target_ = t; }

        void tree_edge(Edge e, const Graph& g) {
          parent_[boost::target(e, g)] = boost::source(e, g);
        }

        void discover_vertex(Vertex u, const Graph&) {
          if (u == target_)
            throw true;
        }

      private:
        std::vector<Vertex> &parent_;
        Vertex target_;
    };


};

/* How difficult could it be to add a hash for pairs to the standard?! */
namespace std {
  template<typename S, typename T> struct hash< pair<S, T> > {
    inline size_t operator()(const pair<S, T> &pair) const {
      size_t seed = 0;
      boost::hash_combine(seed, pair.first);
      boost::hash_combine(seed, pair.second);
      return seed;
    }
  };
}


#include "utvpi_graph-inl.h"

#endif /* UTVPI_GRAPH_H */

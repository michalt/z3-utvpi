#ifndef INEQUALITY_GRAPH_H
#define INEQUALITY_GRAPH_H

#include <unordered_map>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#include "z3.h"

#include "common.h"

/* Z3 uses unsigned int for unique ids. */
namespace {

/* Forward declaration to resolve the circural dependency between
 * InequalityGraph and Rollback. */
template <typename T>
class Rollback;

} /* anonymous namespace */

/* Graph with all the inequalities. The template parameter T is used for
 * defining the domain: either integers or reals. */
template <typename T>
class InequalityGraph {
  public:
    void Print();

    void AddInequality(Sign a, VarId x, T c);
    void AddInequality(Sign a, VarId x, Sign b, VarId y, T c);

    void RemoveInequality(Sign a, VarId x);
    void RemoveInequality(Sign a, VarId x, Sign b, VarId y);

    bool CheckSat();

    friend class WeightRollback;

  private:
    typedef typename boost::adjacency_list<boost::vecS,
                                           boost::vecS,
                                           boost::directedS,
                                           SignedVarId,
                                           T> Graph;

    typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
    typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;

    typedef typename boost::graph_traits<Graph>::vertex_iterator VertexIter;
    typedef typename boost::graph_traits<Graph>::edge_iterator EdgeIter;
    typedef typename boost::graph_traits<Graph>::out_edge_iterator OutEdgeIter;

    typedef std::unordered_map< std::pair<Sign, unsigned int>, Vertex > VertexMap;
    typedef std::list< std::list<Rollback<T> *> > Logs;

    Vertex LookupAddVertex(Sign, VarId x);
    Edge UpdateAddEdge(Vertex src, T weight, Vertex trg);
    void UpdateEdge(Vertex src, T weight, Vertex trg);
    void AddEdgeRollback(Vertex src, Vertex trg);
    void AddWeightRollback(Vertex src, T weight, Vertex trg);
    void GoBack();

    Graph graph_;
    VertexMap vertex_map_;
    Logs logs_;
};

namespace {

template <typename T>
class Rollback {
  public:
    virtual void Execute(InequalityGraph<T> &) = 0;
};

template <typename T>
class EdgeRollback : public Rollback<T> {
  public:
    EdgeRollback(typename InequalityGraph<T>::Vertex s, typename InequalityGraph<T>::Vertex t) : s_(s), t_(t) { }
    virtual void Execute(InequalityGraph<T>& graph) {
      graph.RemoveInequality(s_, t_);
    }
  private:
    typename InequalityGraph<T>::Vertex s_, t_;
};

template <typename T>
class WeightRollback : public Rollback<T> {
  public:
    WeightRollback(typename InequalityGraph<T>::Vertex s, T w, typename InequalityGraph<T>::Vertex t) : s_(s), w_(w), t_(t) { }
    virtual void Execute(InequalityGraph<T> graph) {
      graph.UpdateEdge(s_, w_, t_);
    }
  private:
    typename InequalityGraph<T>::Vertex s_, t_;
    T w_;
};

} /* anonymous namespace */


template <typename T>
void InequalityGraph<T>::Print() {
  EdgeIter iter, end;
  Vertex src, trg;
  for (tie(iter, end) = boost::edges(graph_); iter != end; ++iter) {
    src = boost::source(*iter, graph_);
    trg = boost::target(*iter, graph_);
    SignedVarId var_src = graph_[src];
    SignedVarId var_trg = graph_[trg];
    std::cout << var_src.first
              << var_src.second
              << "(" << src << ")"
              << " goes with weight "
              << graph_[*iter]
              << " to "
              << var_trg.first
              << var_trg.second
              << "(" << trg << ")"
              << std::endl;
  }
}

/* Looks up a vertex for a given variable with sign and if it doesn't exist,
 * creates it. Returns the vertex. */
template <typename T>
typename InequalityGraph<T>::Vertex
InequalityGraph<T>::LookupAddVertex(Sign sign, VarId var_x) {
  SignedVarId s_var = SignedVarId(sign, var_x);
  typename VertexMap::const_iterator iter = vertex_map_.find(s_var);

  Vertex x;
  if (iter == vertex_map_.end()) {
    x = boost::add_vertex(graph_);
    vertex_map_[s_var] = x;
  }
  else {
    x = iter->second;
  }

  /* Record the actual variable with sign. */
  graph_[x] = s_var;
  return x;
}

/* Assumes that the vertices exist in the graph. */
template <typename T>
typename InequalityGraph<T>::Edge InequalityGraph<T>::UpdateAddEdge(Vertex src, T weight, Vertex trg) {

  std::cout << "UpdateAddEdge: start" << std::endl;

  OutEdgeIter iter, end;
  for (boost::tie(iter, end) = boost::out_edges(src, graph_);
       iter != end; ++iter) {
    if (boost::target(*iter, graph_) == trg) {
      /* Found the edge! Update the weight if necessary. */
      if (weight < graph_[*iter]) {
        graph_[*iter] = weight;
      }
      return *iter;
    }
  }

  std::cout << "UpdateAddEdge: after searching" << std::endl;

  /* We haven't found the edge, so we need to create it. */
  Edge edge = boost::add_edge(src, trg, graph_).first;
  std::cout << "UpdateAddEdge: foo" << std::endl;
  graph_[edge] = weight;
  std::cout << "UpdateAddEdge: foo2" << std::endl;

  std::cout << "UpdateAddEdge: returning" << std::endl;
  return edge;
}

/*
 * For every new constraint we need to add two edges:
 * - for   x - y <= k   we add  y+ -k-> x+  and  x- -k-> y-
 * - for   x + y <= k   we add  y- -k-> x+  and  x- -k-> y+
 * - for  -x - y <= k   we add  y+ -k-> x-  and  x+ -k-> y-
 * - for  -x + y <= k   we add  y- -k-> x-  and  x+ -k-> y+
 * Now we can express this a bit more succintly as
 * - for  ax + by <= k  we add  y(-b) -k->  xa  and  x(-a) -k-> yb
 */
template <typename T>
void InequalityGraph<T>::AddInequality(Sign a, VarId var_x, Sign b, VarId var_y, T c) {

  std::cout << "AddInequality: start" << std::endl;

  Vertex x = LookupAddVertex(a, var_x);
  Vertex y = LookupAddVertex(b, var_y);

  Vertex x_neg = LookupAddVertex(negate(a), var_x);
  Vertex y_neg = LookupAddVertex(negate(b), var_y);

  std::cout << "AddInequality: after vertices" << std::endl;

  UpdateAddEdge(y_neg, c, x);
  UpdateAddEdge(x_neg, c, y);

  std::cout << "AddInequality: returning" << std::endl;
}

/*
 * This is a simple inequality. We need to add just one edge:
 * - for   x <= k   we add  x- -2k-> x+
 * - for  -x <= k   we add  x+ -2k-> x-
 * Now we can express this a bit more succintly as
 * - for  ax <= k  we add  x(-a) -2k->  xa
 * Note: the weight is multiplied by 2!
 */
template <typename T>
void InequalityGraph<T>::AddInequality(Sign a, VarId var_x, T c) {
  Vertex x = LookupAddVertex(a, var_x);

  Vertex x_neg = LookupAddVertex(negate(a), var_x);

  UpdateAddEdge(x_neg, 2 * c, x);
}

/* Remove the edge added by the AddInequality(a, x, b, y, c). */
template <typename T>
void InequalityGraph<T>::RemoveInequality(Sign a, VarId var_x, Sign b, VarId var_y) {
  Vertex x = LookupAddVertex(a, var_x);
  Vertex y = LookupAddVertex(b, var_y);

  Vertex x_neg = LookupAddVertex(negate(a), var_x);
  Vertex y_neg = LookupAddVertex(negate(b), var_y);

  boost::remove_edge(y_neg, x, graph_);
  boost::remove_edge(x_neg, y, graph_);
}

/* Remove the edge added by the AddInequality(a, x, c). */
template <typename T>
void InequalityGraph<T>::RemoveInequality(Sign a, VarId var_x) {
  Vertex x = LookupAddVertex(a, var_x);
  Vertex x_neg = LookupAddVertex(negate(a), var_x);

  boost::remove_edge(x_neg, x, graph_);
}

template <typename T>
bool InequalityGraph<T>::CheckSat() {
  std::cout << "CheckSat: not implemented" << std::endl;
}

template <typename T>
void InequalityGraph<T>::AddEdgeRollback(Vertex src, Vertex trg) {
  /* We assume that there was a push callback from Z3 and the list has been
   * allocated. */
  assert(logs_.front() != NULL);

  Rollback<T> *r = new EdgeRollback<T>(src, trg);
  logs_.front().push_front(r);
}

template <typename T>
void InequalityGraph<T>::AddWeightRollback(Vertex src, T weight, Vertex trg) {
  /* We assume that there was a push callback from Z3 and the list has been
   * allocated. */
  assert(logs_.front() != NULL);

  Rollback<T> *r = new WeightRollback<T>(src, weight, trg);
  logs_.front().push_front(r);
}

#endif /* INEQUALITY_GRAPH_H */

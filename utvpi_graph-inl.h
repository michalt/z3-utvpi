#ifndef UTVPI_GRAPH_INL_H
#define UTVPI_GRAPH_INL_H

#include <boost/graph/bellman_ford_shortest_paths.hpp>
#include <boost/graph/strong_components.hpp>

template <typename T>
void UtvpiGraph<T>::Print() {
  EdgeIter iter, end;
  Vertex src, trg;
  for (tie(iter, end) = boost::edges(graph_); iter != end; ++iter) {
    src = boost::source(*iter, graph_);
    trg = boost::target(*iter, graph_);

    /* Suppress printing the special vertex. It's highly confusing since
     * its corresponding SignedVarId is initialized to 0.. */
    if (src == special_vertex_)
      continue;

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
typename UtvpiGraph<T>::Vertex
UtvpiGraph<T>::LookupAddVertex(Sign sign, VarId var_x) {
  SignedVarId s_var = SignedVarId(var_x, sign);
  typename VertexMap::const_iterator iter = vertex_map_.find(s_var);

  Vertex x;
  if (iter == vertex_map_.end()) {
    x = boost::add_vertex(graph_);
    vertex_map_[s_var] = x;

    /* Every vertex must have an edge from the special_vertex_ with weight 0. */
    Edge e = boost::add_edge(special_vertex_, x, graph_).first;
    graph_[e] = 0;
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
typename UtvpiGraph<T>::Edge
UtvpiGraph<T>::UpdateAddEdge(Vertex src, T weight, Vertex trg) {

  std::cout << "UpdateAddEdge: trying search" << std::endl;

  OutEdgeIter iter, end;
  for (boost::tie(iter, end) = boost::out_edges(src, graph_);
       iter != end; ++iter) {
    if (boost::target(*iter, graph_) == trg) {
      /* Found the edge! Update the weight if necessary. */
      if (weight < graph_[*iter]) {
        graph_[*iter] = weight;
        AddWeightRollback(src, weight, trg);
      }
      return *iter;
    }
  }

  std::cout << "UpdateAddEdge: search failed, adding new edge" << std::endl;

  /* We haven't found the edge, so we need to create it. */
  Edge edge = boost::add_edge(src, trg, graph_).first;
  graph_[edge] = weight;
  AddEdgeRollback(src, trg);

  std::cout << "UpdateAddEdge: returning" << std::endl;
  return edge;
}

template <typename T>
bool UtvpiGraph<T>::FindEdge(Vertex src, Vertex trg, Edge& edge) {
  OutEdgeIter iter, end;
  for (boost::tie(iter, end) = boost::out_edges(src, graph_);
       iter != end; ++iter) {
    if (boost::target(*iter, graph_) == trg) {
      edge = *iter;
      return true;
    }
  }
  return false;
}

template <typename T>
void UtvpiGraph<T>::SetEdge(Vertex src, T weight, Vertex trg) {
  Edge edge;
  bool found = FindEdge(src, trg, edge);
  if (found) {
    graph_[edge] = weight;
  }
  assert(found);
}

template <typename T>
void UtvpiGraph<T>::RemoveEdge(Vertex src, Vertex trg) {
  Edge edge;
  bool found = FindEdge(src, trg, edge);
  if (found) {
    boost::remove_edge(edge, graph_);
  }
  assert(found);
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
void UtvpiGraph<T>::AddInequality(Sign a, VarId var_x, Sign b, VarId var_y, T c) {

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
void UtvpiGraph<T>::AddInequality(Sign a, VarId var_x, T c) {
  Vertex x = LookupAddVertex(a, var_x);

  Vertex x_neg = LookupAddVertex(negate(a), var_x);

  UpdateAddEdge(x_neg, 2 * c, x);
}

template <typename T>
void UtvpiGraph<T>::AddEdgeRollback(Vertex src, Vertex trg) {
  /* We assume that there was a push callback from Z3 and the list has been
   * allocated. */
  assert(logs_.front() != NULL);

  Rollback *r = new EdgeRollback(src, trg);
  std::cout << "before" << std::endl;
  logs_.front()->push_front(r);
  std::cout << "after" << std::endl;
}

template <typename T>
void UtvpiGraph<T>::AddWeightRollback(Vertex src, T weight, Vertex trg) {
  /* We assume that there was a push callback from Z3 and the list has been
   * allocated. */
  assert(logs_.front() != NULL);

  Rollback *r = new WeightRollback(src, weight, trg);
  std::cout << "before2" << std::endl;
  logs_.front()->push_front(r);
  std::cout << "after2" << std::endl;
}

template <typename T>
void UtvpiGraph<T>::Pop() {
  std::cout << "UtvpiGraph: Pop" << std::endl;
  std::list<Rollback *> *list = logs_.front();
  logs_.pop_front();

  for (auto &r : *list) {
    r->Execute(*this);
    delete r;
  }
}

template <typename T>
void UtvpiGraph<T>::Push() {
  std::cout << "UtvpiGraph: Push" << std::endl;
  logs_.push_front(new std::list<Rollback *>());
}

/**
 * Clear everything. The state is the same as just after constructing the graph.
 */
template <typename T>
void UtvpiGraph<T>::Reset() {
  std::cout << "UtvpiGraph: Reset" << std::endl;
  graph_.clear();
  vertex_map_.clear();
  logs_.clear();
  logs_.push_front(new std::list<Rollback *>());
  special_vertex_ = boost::add_vertex(graph_);
}


#endif /* UTVPI_GRAPH_INL_H */

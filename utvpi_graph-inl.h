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

/* Remove the edge added by the AddInequality(a, x, b, y, c). */
// template <typename T>
// void UtvpiGraph<T>::RemoveInequality(Sign a, VarId var_x, Sign b, VarId var_y) {
  // Vertex x = LookupAddVertex(a, var_x);
  // Vertex y = LookupAddVertex(b, var_y);

  // Vertex x_neg = LookupAddVertex(negate(a), var_x);
  // Vertex y_neg = LookupAddVertex(negate(b), var_y);

  // boost::remove_edge(y_neg, x, graph_);
  // boost::remove_edge(x_neg, y, graph_);
// }

// [> Remove the edge added by the AddInequality(a, x, c). <]
// template <typename T>
// void UtvpiGraph<T>::RemoveInequality(Sign a, VarId var_x) {
  // Vertex x = LookupAddVertex(a, var_x);
  // Vertex x_neg = LookupAddVertex(negate(a), var_x);

  // boost::remove_edge(x_neg, x, graph_);
// }

/*
 * FIXME: this new vertex and edges should be maintained in the graph at all
 * times... insead of adding and then removing them!!
 */
template <typename T>
bool UtvpiGraph<T>::CheckSat() {
  Vertex special = boost::add_vertex(graph_);
  VertexIter iter, end;
  for (boost::tie(iter, end) = boost::vertices(graph_); iter != end; ++iter) {
    Edge e = boost::add_edge(special, *iter, graph_).first;
    graph_[e] = 0;
  }

  std::size_t size = boost::num_vertices(graph_);

  /* Create distance map. */
  boost::vector_property_map<T> distance(size);
  boost::vector_property_map<Vertex> parent(size);
  // std::vector<T> distance(size);
  // std::vector<Vertex> parent(size);

  bool no_neg_cycle = boost::bellman_ford_shortest_paths
    (graph_, size, boost::root_vertex(special).
                     weight_map(boost::get(boost::edge_bundle, graph_)).
                     distance_map(&distance[0]).
                     predecessor_map(&parent[0]));

  if (!no_neg_cycle) {
    std::cout << "Found negative cycle without tightening!" << std::endl;
    return false;
  }

  for (boost::tie(iter, end) = boost::vertices(graph_); iter != end; ++iter) {
    std::cout << "Vertex: " << graph_[*iter]
              << ", its parent: " << parent[*iter]
              << ", its distance: " << distance[*iter]
              << std::endl;
  }

  Graph tmp_graph(size);
  boost::vector_property_map<Vertex> to_tmp(size);
  boost::vector_property_map<bool> to_tmp_assigned(size);

  /* FIXME: is this actually necessary? */
  for (auto i = to_tmp_assigned.storage_begin();
       i != to_tmp_assigned.storage_end();
       ++i) {
    *i = false;
  }
  // std::vector_property_map<bool> to_tmp_assigned(size, false);
  Vertex tmp_special = boost::add_vertex(tmp_graph);
  to_tmp[special] = tmp_special;
  /*
   * Now we need to create the induced graph by taking those edges from the
   * original one that can possible be the source of a negative cycle that
   * hasen't been detected above. This can happen when we have
   *   x+ - x- <= odd
   * which means that
   *   2x <= odd
   * and so in case of integers it must be that
   *   2x <= odd - 1
   * But this lowers the bound and tuhs can reveal a negative cycle!
   */
  EdgeIter e_iter, e_end;
  Vertex src, trg, tmp_src, tmp_trg;
  Edge tmp_edge;
  for (tie(e_iter, e_end) = boost::edges(graph_); e_iter != e_end; ++e_iter) {

    src = boost::source(*e_iter, graph_);
    trg = boost::target(*e_iter, graph_);

    /* We're only looking for edges that have tight bound, that is the
     * difference between distance is exactly the bound. */
    // FIXME: what about the special thingy..?
    if (src == special || distance[trg] - distance[src] != graph_[*e_iter])
      continue;

    // auto tmp_src_iter = to_tmp.find(src);
    // auto tmp_trg_iter = to_tmp.find(trg);

    if (!to_tmp_assigned[src]) {
      tmp_src = boost::add_vertex(tmp_graph);
      tmp_graph[tmp_src] = graph_[src];
      to_tmp[src] = tmp_src;
      to_tmp_assigned[src] = true;

      tmp_edge = boost::add_edge(tmp_special, tmp_src, tmp_graph).first;
      tmp_graph[tmp_edge] = 0;
    } else {
      tmp_src = to_tmp[src];
    }

    if (!to_tmp_assigned[trg]) {
      tmp_trg = boost::add_vertex(tmp_graph);
      tmp_graph[tmp_trg] = graph_[trg];
      to_tmp[trg] = tmp_trg;
      to_tmp_assigned[trg] = true;

      tmp_edge = boost::add_edge(tmp_special, tmp_trg, tmp_graph).first;
      tmp_graph[tmp_edge] = 0;
    } else {
      tmp_trg = to_tmp[trg];
    }

    tmp_edge = boost::add_edge(tmp_src, tmp_trg, tmp_graph).first;
    tmp_graph[tmp_edge] = graph_[*e_iter];

  }

  size_t tmp_size = boost::num_vertices(tmp_graph);
  boost::vector_property_map<T> tmp_distance(tmp_size);
  boost::vector_property_map<Vertex> tmp_parent(tmp_size);

  std::vector<int> component(tmp_size), discover_time(tmp_size);
  std::vector<boost::default_color_type> color(tmp_size);
  std::vector<Vertex> root(tmp_size);
  unsigned int num = strong_components(tmp_graph, &component[0],
                              boost::root_map(&root[0]).
                                color_map(&color[0]).
                                discover_time_map(&discover_time[0]));

  if (num == tmp_size)
    return true;

  SignedVarId s_varid, neg_s_varid;
  Vertex vertex, neg_vertex, new_vertex, new_neg_vertex;
  for (auto var_iter = vertex_map_.begin();
      var_iter != vertex_map_.end();
      ++var_iter) {

    boost::tie(s_varid, vertex) = *var_iter;
    neg_vertex = vertex_map_[negate(s_varid)];

    if (!to_tmp_assigned[vertex] || !to_tmp_assigned[neg_vertex])
      continue;

    new_vertex = to_tmp[vertex];
    new_neg_vertex = to_tmp[neg_vertex];

    /*
     * If they are in the same strongly connected component in the induced
     * graph and they're distance in the original graph is odd, then it means
     * that we can tighten the bound by substracting 1. But they're already in a
     * cycle with weight 0, so we have a negative cycle!
     */
    if (component[new_vertex] == component[new_neg_vertex]
        && (distance[vertex] - distance[neg_vertex]) % 2 != 0) {
      std::cout << "Found negative cycle by tightening!" << std::endl;
      return false;
    }
  }

  return true;
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

#endif /* UTVPI_GRAPH_INL_H */

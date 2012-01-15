#ifndef UTVPI_GRAPH_Z_INL_H
#define UTVPI_GRAPH_Z_INL_H

/*
 * Unfortunately since the base class is a template, we need to stick this->
 * everywhere and define the typedefs again.. Annoying.
 */
template <typename T>
bool UtvpiGraphZ<T>::CheckSat() {
  typedef typename UtvpiGraph<T>::Graph Graph;
  typedef typename UtvpiGraph<T>::Vertex Vertex;
  typedef typename UtvpiGraph<T>::VertexIter VertexIter;
  typedef typename UtvpiGraph<T>::Edge Edge;
  typedef typename UtvpiGraph<T>::EdgeIter EdgeIter;

  std::size_t size = boost::num_vertices(this->graph_);

  /* Create distance map. */
  std::vector<T> distance(size, 0);
  std::vector<Vertex> parent(size, 0);

  /* Detect cycle with Bellman-Ford algorithm. Also this is why we need the
   * special_vertex_. */
  bool no_neg_cycle = boost::bellman_ford_shortest_paths
    (this->graph_, size, boost::root_vertex(this->special_vertex_).
                           weight_map(boost::get(boost::edge_bundle, this->graph_)).
                           distance_map(&distance[0]).
                           predecessor_map(&parent[0]));

  VertexIter v_iter, v_end;
  for (boost::tie(v_iter, v_end) = boost::vertices(this->graph_);
      v_iter != v_end;
      ++v_iter) {
    std::cout << "Vertex: " << this->graph_[*v_iter]
              << ", its parent: " << parent[*v_iter]
              << ", its distance: " << distance[*v_iter]
              << std::endl;
  }

  if (!no_neg_cycle) {
    std::cout << "Found negative cycle without tightening!" << std::endl;
    return false;
  }

  Graph tmp_graph(size);

  /* Vertex in graph_ -> Vertex in tmp_graph */
  std::vector<Vertex> to_tmp(size);

  /* Vertex in graph_ -> bool */
  std::vector<bool> in_tmp(size, false);

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
  Edge tmp_edge;
  Vertex src, trg, tmp_src, tmp_trg;
  for (tie(e_iter, e_end) = boost::edges(this->graph_); e_iter != e_end; ++e_iter) {

    src = boost::source(*e_iter, this->graph_);
    trg = boost::target(*e_iter, this->graph_);

    /* We're only looking for edges that have tight bound, that is the
     * difference between distance is exactly the bound. */
    /* FIXME: what about the special_vertex_ vertex..? It should be safe to ignore it.. */
    if (src == this->special_vertex_
        || distance[trg] - distance[src] != this->graph_[*e_iter])
      continue;

    if (!in_tmp[src]) {
      tmp_src = boost::add_vertex(tmp_graph);
      tmp_graph[tmp_src] = this->graph_[src];
      to_tmp[src] = tmp_src;
      in_tmp[src] = true;
    } else {
      tmp_src = to_tmp[src];
    }

    if (!in_tmp[trg]) {
      tmp_trg = boost::add_vertex(tmp_graph);
      tmp_graph[tmp_trg] = this->graph_[trg];
      to_tmp[trg] = tmp_trg;
      in_tmp[trg] = true;
    } else {
      tmp_trg = to_tmp[trg];
    }

    tmp_edge = boost::add_edge(tmp_src, tmp_trg, tmp_graph).first;
    tmp_graph[tmp_edge] = this->graph_[*e_iter];

  }

  size_t tmp_size = boost::num_vertices(tmp_graph);

  std::vector<int> component(tmp_size), discover_time(tmp_size);
  std::vector<boost::default_color_type> color(tmp_size);
  std::vector<Vertex> root(tmp_size);

  unsigned int num = strong_components
    (tmp_graph, &component[0], boost::root_map(&root[0]).
                                      color_map(&color[0]).
                                      discover_time_map(&discover_time[0]));

  if (num == tmp_size) {
    /* No two vertices are in the same component. */
    return true;
  }

  /*
   * We go through all the variables and check if they're corresponding vertices
   * (if any) in the induced graph are in the same strongly connected component.
   * If they are and additionally their distance in the original graph is odd,
   * then it means that we can tighten the bound by substracting 1. But they're
   * already in a cycle with weight 0, so we have a negative cycle!
   */
  SignedVarId s_varid, neg_s_varid;
  Vertex vertex, neg_vertex, tmp_vertex, tmp_neg_vertex;
  for (auto var_iter = this->vertex_map_.begin();
      var_iter != this->vertex_map_.end();
      ++var_iter) {

    boost::tie(s_varid, vertex) = *var_iter;
    neg_vertex = this->vertex_map_[negate(s_varid)];

    if (!in_tmp[vertex] || !in_tmp[neg_vertex])
      continue;

    tmp_vertex = to_tmp[vertex];
    tmp_neg_vertex = to_tmp[neg_vertex];

    if (component[tmp_vertex] == component[tmp_neg_vertex]
        && (distance[vertex] - distance[neg_vertex]) % 2 != 0) {
      std::cout << "Found negative cycle by tightening!" << std::endl;
      return false;
    }
  }

  return true;
}

#endif /* UTVPI_GRAPH_Z_INL_H */

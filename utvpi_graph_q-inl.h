#ifndef UTVPI_GRAPH_Q_INL_H
#define UTVPI_GRAPH_Q_INL_H

/*
 * Unfortunately since the base class is a template, we need to stick this->
 * everywhere and define the typedefs again.. Annoying.
 */
template <typename T>
bool UtvpiGraphQ<T>::CheckSat() {
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
    std::cout << "Found negative cycle!" << std::endl;
    return false;
  }

  return true;

}

#endif /* UTVPI_GRAPH_Q_INL_H */

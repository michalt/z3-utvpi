#ifndef UTVPI_GRAPH_Q_INL_H
#define UTVPI_GRAPH_Q_INL_H

#include "common.h"

/*
 * Unfortunately since the base class is a template, we need to stick this->
 * everywhere and define the typedefs again.. Annoying.
 */
template <typename T>
std::pair<bool, std::list<ReasonPtr>*> UtvpiGraphQ<T>::Satisfiable() {

  std::size_t graph_size = boost::num_vertices(this->graph_);

  std::vector<T> distance(graph_size, 0);
  std::vector<Vertex> parent(graph_size, 0);
  NegVisitor neg_visitor;

  /* Detect cycle with Bellman-Ford algorithm. Also this is why we need the
   * special_vertex_. */
  bool no_neg_cycle = boost::bellman_ford_shortest_paths
    (this->graph_, graph_size,
     boost::root_vertex(this->special_vertex_).
     weight_map(boost::get(boost::edge_bundle, this->graph_)).
     distance_map(&distance[0]).
     predecessor_map(&parent[0]).
     visitor(neg_visitor).
     distance_combine(custom_plus<T>()));

#ifdef DEBUG
  {
    VertexIter v_iter, v_end;
    for (boost::tie(v_iter, v_end) = boost::vertices(this->graph_);
        v_iter != v_end;
        ++v_iter) {
      std::cout << "Vertex: " << this->graph_[*v_iter]
                << ", its parent: " << parent[*v_iter]
                << ", its distance: " << distance[*v_iter]
                << std::endl;
    }
  }
#endif

  auto neg_cycle = new std::list<ReasonPtr>();
  if (!no_neg_cycle) {
    std::cout << "Found negative cycle without tightening!" << std::endl;
    neg_cycle = GetNegativeCycle(*neg_visitor.neg_edge, parent);
    assert(neg_cycle != NULL);
    for (auto r : *neg_cycle) {
      std::cout << "Negative cycle: "
                << *r
                << std::endl;
    }
    return std::make_pair(false, neg_cycle);
  }
  return std::make_pair(true, neg_cycle);
}

#endif /* UTVPI_GRAPH_Q_INL_H */

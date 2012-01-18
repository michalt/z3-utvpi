#ifndef UTVPI_GRAPH_Z_H
#define UTVPI_GRAPH_Z_H

#include <boost/graph/bellman_ford_shortest_paths.hpp>

#include "utvpi_graph.h"

template <typename T>
class UtvpiGraphZ : public UtvpiGraph<T> {
  public:
    bool Satisfiable();

  private:
    /*
     * Unfortunately since the base class is a template, we need to stick this->
     * everywhere.. Annoying.
     */
    typedef typename UtvpiGraph<T>::Graph Graph;
    typedef typename UtvpiGraph<T>::Vertex Vertex;
    typedef typename UtvpiGraph<T>::VertexIter VertexIter;
    typedef typename UtvpiGraph<T>::Edge Edge;
    typedef typename UtvpiGraph<T>::EdgeIter EdgeIter;

    std::list<Edge>* GetNegativeCycle(const Edge &start_edge,
        const std::vector<Vertex> &parent);

    struct NegVisitor : public boost::bellman_visitor<> {
      void edge_not_minimized(Edge e, Graph &g) {
        std::cout << "## edge not minimized ##" << e << std::cout;
        neg_edge = e;
      }
      Edge neg_edge;
    };

};

#include "utvpi_graph_z-inl.h"

#endif /* UTVPI_GRAPH_Z_H */

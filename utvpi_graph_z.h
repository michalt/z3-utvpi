#ifndef UTVPI_GRAPH_Z_H
#define UTVPI_GRAPH_Z_H

#include "utvpi_graph.h"

template <typename T>
class UtvpiGraphZ : public UtvpiGraph<T> {
  public:
    std::pair<bool, std::list<ReasonPtr>*> Satisfiable();

  private:
    /*
     * Unfortunately since the base class is a template, we need to stick this->
     * everywhere and define typedefs again.. Annoying.
     */
    typedef typename UtvpiGraph<T>::Graph Graph;
    typedef typename UtvpiGraph<T>::Vertex Vertex;
    typedef typename UtvpiGraph<T>::VertexIter VertexIter;
    typedef typename UtvpiGraph<T>::Edge Edge;
    typedef typename UtvpiGraph<T>::EdgeIter EdgeIter;

    typedef typename UtvpiGraph<T>::ReasonMap ReasonMap;
    typedef typename UtvpiGraph<T>::NegVisitor NegVisitor;

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

#include "utvpi_graph_z-inl.h"

#endif /* UTVPI_GRAPH_Z_H */

#ifndef UTVPI_GRAPH_Q_H
#define UTVPI_GRAPH_Q_H

#include "utvpi_graph.h"

template <typename T>
class UtvpiGraphQ : public UtvpiGraph<T> {
  public:
    std::pair<bool, std::list<ReasonPtr>*> Satisfiable();
  private:
    typedef typename UtvpiGraph<T>::Graph Graph;
    typedef typename UtvpiGraph<T>::Vertex Vertex;
    typedef typename UtvpiGraph<T>::VertexIter VertexIter;
    typedef typename UtvpiGraph<T>::Edge Edge;
    typedef typename UtvpiGraph<T>::EdgeIter EdgeIter;
    typedef typename UtvpiGraph<T>::NegVisitor NegVisitor;

};

#include "utvpi_graph_q-inl.h"

#endif /* UTVPI_GRAPH_Q_H */

#ifndef UTVPI_GRAPH_Q_H
#define UTVPI_GRAPH_Q_H

#include "utvpi_graph.h"

template <typename T>
class UtvpiGraphQ : public UtvpiGraph<T> {
  public:
    bool Satisfiable();
};

#include "utvpi_graph_q-inl.h"

#endif /* UTVPI_GRAPH_Q_H */

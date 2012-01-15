#ifndef UTVPI_GRAPH_Z_H
#define UTVPI_GRAPH_Z_H

#include "utvpi_graph.h"

template <typename T>
class UtvpiGraphZ : public UtvpiGraph<T> {
  public:
    bool CheckSat();
};

#include "utvpi_graph_z-inl.h"

#endif /* UTVPI_GRAPH_Z_H */

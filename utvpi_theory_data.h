#ifndef UTVPI_THEORY_DATA_H
#define UTVPI_THEORY_DATA_H

#include <z3.h>


/*
 * Stores all the necessary data for the theory. Note that predicate, minus and
 * plus should really be constants, however, this seems problematic. To create
 * them using Z3 API we need to create UtvpiData object first, but that would
 * require creating the predicate, minus and plus...
 */
template <template <typename> class Utvpi, typename T>
struct UtvpiData {
  UtvpiData() : utvpi(), svpi(), minus(), plus(), id_to_ast(), graph() { }
  Z3_func_decl utvpi, svpi;
  Z3_ast minus, plus;
  std::unordered_map<VarId, Z3_ast> id_to_ast;
  Utvpi<T> graph;
};

#endif /* UTVPI_THEORY_DATA_H */

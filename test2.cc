#include <iostream>

#include <boost/graph/graph_traits.hpp>

#include <z3.h>

#include "utvpi_graph_z.h"
// #include "utvpi_graph_q.h"
#include "utvpi_theory.h"

int main(int argc, char *argv[]) {
  Z3_context ctx = mk_context();
  Z3_theory theory = MkTheory< UtvpiGraphZ<int> >(ctx);

  Z3_ast formula = Z3_parse_smtlib2_file(ctx, "test.smt", 0, 0, 0, 0, 0, 0);
  std::cout << "Parsed the following formula:" << std::endl
            << Z3_ast_to_string(ctx, formula) << std::endl;

  Z3_assert_cnstr(ctx, formula);

  Z3_model m      = 0;
  Z3_lbool result = Z3_check_and_get_model(ctx, &m);
  switch (result) {
    case Z3_L_FALSE:
        printf("unsat\n");
        break;
    case Z3_L_UNDEF:
        printf("unknown\n");
        printf("potential model:\n%s\n", Z3_model_to_string(ctx, m));
        break;
    case Z3_L_TRUE:
        printf("sat\n%s\n", Z3_model_to_string(ctx, m));
        break;
    }
    if (m) {
        Z3_del_model(ctx, m);
    }

  Z3_del_context(ctx);
}

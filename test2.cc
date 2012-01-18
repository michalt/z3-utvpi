#include <iostream>

#include <boost/graph/graph_traits.hpp>

#include <z3.h>

#include "utvpi_graph_z.h"
#include "utvpi_graph_q.h"
#include "utvpi_theory.h"

namespace boost {
  template <>
  struct closed_plus<mpz_class> {
    mpz_class operator()(const mpz_class& a, const mpz_class& b) const {
      return a + b;
    }
  };
}

int main(int argc, char *argv[]) {
  Z3_context ctx = MkContext();
  Z3_theory theory = MkTheory<UtvpiGraphZ, int>(ctx);
  // Z3_theory theory = MkTheory<UtvpiGraphZ, mpz_class>(ctx);
  // Z3_theory theory = MkTheory<UtvpiGraphQ, mpq_class>(ctx);

  std::cout << "Parsing..." << std::endl;
  // FIXME: don't pass only NULL here --- may cause segfault!
  Z3_ast formula = Z3_parse_smtlib2_file(ctx, "test.smt", 0, 0, 0, 0, 0, 0);
  // Z3_ast formula = Z3_parse_smtlib2_file(ctx, "tightening_example.smt", 0, 0, 0, 0, 0, 0);
  if (formula == NULL) {
    std::cout << "Parsing failed!!!" << std::endl;
  }
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

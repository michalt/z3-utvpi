#include <iostream>
#include <cstdlib>

#include <boost/graph/graph_traits.hpp>

#include <gmpxx.h>
#include <z3.h>

#include "utvpi_graph_z.h"
#include "utvpi_graph_q.h"
#include "utvpi_theory.h"

void UsageAndDie() {
  std::cerr << "Usage: test [option] [file]"
            << std::endl
            << "[option] is either -i, -ia or -r"
            << std::endl;
  exit(EXIT_FAILURE);
}

template <template <typename> class Utvpi, typename T>
void ParseAndRun(const char *file) {
  Z3_context context = MkContext();
  MkTheory<Utvpi, T>(context);

  std::cout << "Parsing file: " << file << std::endl;

  Z3_ast formula = Z3_parse_smtlib2_file(context, file, 0, 0, 0, 0, 0, 0);

  if (formula == NULL) {
    std::cerr << "Parsing failed!" << std::endl;
    return;
  }
  std::cout << "Parsed the following formula:"
            << std::endl
            << Z3_ast_to_string(context, formula)
            << std::endl;

  Z3_assert_cnstr(context, formula);

  switch (Z3_check(context)) {
    case Z3_L_FALSE:
      std::cout << "Z3: UNSAT" << std::endl;
      break;
    case Z3_L_UNDEF:
      std::cout << "Z3: UNKNOWN" << std::endl;
      break;
    case Z3_L_TRUE:
      std::cout << "Z3: SAT" << std::endl;
      break;
  }

  Z3_del_context(context);
}

int main(int argc, char *argv[]) {
  if (argc != 3) {
    UsageAndDie();
  }

  const char *file = argv[2];

  /* Should probably use getopt.. */
  if (std::string(argv[1]) == "-i") {
    ParseAndRun<UtvpiGraphZ, int>(file);
  } else if (std::string(argv[1]) == "-ia") {
    ParseAndRun<UtvpiGraphZ, mpz_class>(file);
  } else if (std::string(argv[1]) == "-r") {
    ParseAndRun<UtvpiGraphQ, mpq_class>(file);
  } else {
    UsageAndDie();
  }
}

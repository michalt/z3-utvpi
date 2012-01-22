#ifndef UTVPI_THEORY_H
#define UTVPI_THEORY_H

/*
 * Integration with Z3.
 */

#include <cstdlib>
#include <cstdint>

#include <gmpxx.h>
#include <z3.h>

#include "reason.h"

/*
 * A few helper functions.
 */

void Die(const char *msg) {
  std::cerr << "Died because of: " << msg << std::endl;
  exit(EXIT_FAILURE);
}

void ErrorHandler(Z3_error_code e) {
  std::cerr << "Error code: " << e << std::endl;
  Die("incorrect use of Z3.");
}


Z3_context MkContext() {
  Z3_config  cfg = Z3_mk_config();

  Z3_context context = Z3_mk_context(cfg);

#ifdef TRACING
  Z3_trace_to_stderr(context);
#endif

  Z3_set_error_handler(context, ErrorHandler);

  Z3_del_config(cfg);
  return context;
}

Sign ArgToSign(Z3_ast &minus, Z3_ast &plus, Z3_ast &ast) {
  if (ast == plus)
    return Pos;
  assert(ast == minus);
  return Neg;
}


/*
 * Magic constants that are used by the theory.
 */

/* Should we put them into UtvpiData? */
static const char        *utvpi_name  = "Utvpi";
static const char        *svpi_name   = "Svpi";
static const unsigned int utvpi_arity = 5;
static const unsigned int svpi_arity  = 3;
static const char        *sign_name   = "Sign";
static const char        *minus_name  = "Minus";
static const char        *plus_name   = "Plus";

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

template <template <typename > class Utvpi, typename T>
void NewEquality(Z3_theory theory, Z3_ast ast1, Z3_ast ast2) {

  Z3_context context = Z3_theory_get_context(theory);

#ifdef VERBOSE
  std::cout << "Z3: NewEquality: "
            << Z3_ast_to_string(context, ast1)
            << " == "
            << Z3_ast_to_string(context, ast2)
            << std::endl;
#endif

  UtvpiData<Utvpi, T> *data =
    static_cast<UtvpiData<Utvpi, T>*>(Z3_theory_get_ext_data(theory));

  VarId x = Z3_get_ast_id(context, ast1);
  VarId y = Z3_get_ast_id(context, ast2);
  data->id_to_ast[x] = ast1;
  data->id_to_ast[y] = ast2;

  data->graph.AddEquality(x, y);
}

template <template <typename > class Utvpi, typename T>
void Push(Z3_theory theory) {

#ifdef VERBOSE
  std::cout << "Z3: Push" << std::endl;
#endif

  UtvpiData<Utvpi, T> *data =
    static_cast<UtvpiData<Utvpi, T>*>(Z3_theory_get_ext_data(theory));
  data->graph.Push();
}

template <template <typename > class Utvpi, typename T>
void Pop(Z3_theory theory) {

#ifdef VERBOSE
  std::cout << "Z3: Pop" << std::endl;
#endif

  UtvpiData<Utvpi, T> *data =
    static_cast<UtvpiData<Utvpi, T>*>(Z3_theory_get_ext_data(theory));
  data->graph.Pop();
}

template <template <typename > class Utvpi, typename T>
void Reset(Z3_theory theory) {

#ifdef VERBOSE
  std::cout << "Z3: Reset" << std::endl;
#endif

  UtvpiData<Utvpi, T> *data =
    static_cast<UtvpiData<Utvpi, T>*>(Z3_theory_get_ext_data(theory));
  data->graph.Reset();
}

template <template <typename > class Utvpi, typename T>
void Restart(Z3_theory theory) {

#ifdef VERBOSE
  std::cout << "Z3: Restart" << std::endl;
#endif

  UtvpiData<Utvpi, T> *data =
    static_cast<UtvpiData<Utvpi, T>*>(Z3_theory_get_ext_data(theory));
  data->graph.Reset();
}

template <template <typename > class Utvpi, typename T>
Z3_bool SatCheck(Z3_theory theory, bool is_final) {

  Z3_context context = Z3_theory_get_context(theory);
  UtvpiData<Utvpi, T> *data =
    static_cast<UtvpiData<Utvpi, T>*>(Z3_theory_get_ext_data(theory));

#ifdef DEBUG
  data->graph.Print();
#endif

  bool sat;
  std::list<ReasonPtr> *cycle;
  std::tie(sat, cycle) = data->graph.Satisfiable();

  if (sat)
    return Z3_TRUE;

#ifdef VERBOSE
  std::cout << "SatCheck: UNSAT." << std::endl;
#endif

  /* Current context is unsatisfiable. If this is the final check just return
   * UNSAT. */

  if (is_final)
    return Z3_FALSE;

  /* Otherwise get the explanations and assert it.*/

  /* There must be at least one edge cause the negative cycle. */
  assert(cycle->size() > 0);

  Z3_ast conflict = cycle->front()->MkAst(context, data->id_to_ast, data->utvpi,
      data->svpi, data->minus, data->plus);
  cycle->pop_front();

  Z3_ast tmp[2];
  for (auto r : *cycle) {
    tmp[0] = conflict;
    tmp[1] = r->MkAst(context, data->id_to_ast, data->utvpi, data->svpi,
        data->minus, data->plus);
    conflict = Z3_mk_and(context, 2, tmp);

  }
  conflict = Z3_mk_not(context, conflict);

#ifdef VERBOSE
  std::cout << "SatCheck: asserting theory axiom:" << std::endl;
  std::cout << Z3_ast_to_string(context, conflict) << std::endl;
#endif

  Z3_theory_assert_axiom(theory, conflict);
  return Z3_FALSE;
}

template <template <typename > class Utvpi, typename T>
Z3_bool FinalCheck(Z3_theory theory) {
  return SatCheck<Utvpi, T>(theory, true);
}


template <template <typename> class Utvpi, typename T>
Z3_ast SignToArg(UtvpiData<Utvpi, T> *data, Sign sign) {
  if (sign == Pos)
    return data->plus;
  return data->minus;
}

/*
 * We can get a numeral from Z3 in a number of ways. This allows us to make the
 * choce depending on the type of our domain.
 */
template <typename T>
Z3_bool GetNumeral(Z3_context context, Z3_ast ast, T& result);

template <>
Z3_bool GetNumeral(Z3_context context, Z3_ast ast, int& result) {
  return Z3_get_numeral_int(context, ast, &result);
}

template <>
Z3_bool GetNumeral(Z3_context context, Z3_ast ast, long long& result) {
  return Z3_get_numeral_int64(context, ast, &result);
}

template <>
Z3_bool GetNumeral(Z3_context context, Z3_ast ast, mpz_class& result) {
  Z3_string str = Z3_get_numeral_string(context, ast);
  result = mpz_class(static_cast<const char*>(str));
  return Z3_TRUE;
}

template <>
Z3_bool GetNumeral(Z3_context context, Z3_ast ast, mpq_class& result) {
  Z3_string str = Z3_get_numeral_string(context, ast);
  result = mpq_class(static_cast<const char*>(str));
  return Z3_TRUE;
}

/*
 * Callbacks.
 */

template <template <typename > class Utvpi, typename T>
void NewAssignment(Z3_theory theory, Z3_ast ast, Z3_bool value) {

  Z3_context context = Z3_theory_get_context(theory);

#ifdef VERBOSE
  std::cout << "Z3: Assigned "
            << Z3_ast_to_string(context, ast)
            << " to "
            << static_cast<int>(value)
            << std::endl;
#endif

  if (value != Z3_TRUE) {
#ifdef DEBUG
    std::cout << "NewAssignment: something is not true. Ignoring.."
              << std::endl;
#endif
    return;
  }

  Z3_app app_ast = Z3_to_app(context, ast);

  Z3_func_decl predicate_decl = Z3_get_app_decl(context, app_ast);

  assert(predicate_decl != NULL);

  UtvpiData<Utvpi, T> *data =
    static_cast<UtvpiData<Utvpi, T>*>(Z3_theory_get_ext_data(theory));

  unsigned int num_args = Z3_get_app_num_args(context, app_ast);
  if (predicate_decl == data->utvpi && num_args == utvpi_arity) {

    Z3_ast arg0 = Z3_get_app_arg (context, app_ast, 0);
    Z3_ast arg1 = Z3_get_app_arg (context, app_ast, 1);
    Z3_ast arg2 = Z3_get_app_arg (context, app_ast, 2);
    Z3_ast arg3 = Z3_get_app_arg (context, app_ast, 3);
    Z3_ast arg4 = Z3_get_app_arg (context, app_ast, 4);

    Sign a = ArgToSign(data->minus, data->plus, arg0);
    Sign b = ArgToSign(data->minus, data->plus, arg2);
    VarId x = Z3_get_ast_id(context, arg1);
    VarId y = Z3_get_ast_id(context, arg3);
    data->id_to_ast[x] = arg1;
    data->id_to_ast[y] = arg3;

    T c;
    bool success = GetNumeral(context, arg4, c);

    if (!success)
      Die("Failed to get the numeral!");

    data->graph.AddInequality(a, x, b, y, c);
  } else if (predicate_decl == data->svpi && num_args == svpi_arity) {

    Z3_ast arg0 = Z3_get_app_arg (context, app_ast, 0);
    Z3_ast arg1 = Z3_get_app_arg (context, app_ast, 1);
    Z3_ast arg2 = Z3_get_app_arg (context, app_ast, 2);

    Sign a = ArgToSign(data->minus, data->plus, arg0);
    VarId x = Z3_get_ast_id(context, arg1);
    data->id_to_ast[x] = arg1;

    T c;
    bool success = GetNumeral(context, arg2, c);

    if (!success)
      Die("Failed to get the numeral!");

    data->graph.AddInequality(a, x, c);
  } else {
    /* We should never get here.. */
    assert(false);
  }

  SatCheck<Utvpi, T>(theory, false);
}


template <typename T>
struct CustomSort {
  CustomSort(Z3_context context) : sort(Z3_mk_int_sort(context)) { }
  Z3_sort sort;
};

template <>
struct CustomSort<mpq_class> {
  CustomSort(Z3_context context) : sort(Z3_mk_real_sort(context)) { }
  Z3_sort sort;
};

template <template <typename > class Utvpi, typename T>
Z3_theory MkTheory(Z3_context context) {
  UtvpiData<Utvpi, T> *data = new UtvpiData<Utvpi, T>();
  Z3_theory theory = Z3_mk_theory(context, "UTVPI", data);

  Z3_symbol utvpi_sym = Z3_mk_string_symbol(context, utvpi_name);
  Z3_symbol svpi_sym = Z3_mk_string_symbol(context, svpi_name);
  Z3_symbol sign_sym = Z3_mk_string_symbol(context, sign_name);
  Z3_symbol plus_sym = Z3_mk_string_symbol(context, plus_name);
  Z3_symbol minus_sym = Z3_mk_string_symbol(context, minus_name);

  Z3_sort domain_sort = CustomSort<T>(context).sort;
  Z3_sort bool_sort = Z3_mk_bool_sort(context);
  Z3_sort sign_sort = Z3_theory_mk_sort(context, theory, sign_sym);

  Z3_sort utvpi_domain[5] = { sign_sort, domain_sort, sign_sort, domain_sort,
    domain_sort };
  Z3_sort svpi_domain[3] = { sign_sort, domain_sort, domain_sort };

  data->utvpi = Z3_theory_mk_func_decl(context, theory, utvpi_sym, 5,
      utvpi_domain, bool_sort);
  data->svpi = Z3_theory_mk_func_decl(context, theory, svpi_sym, 3,
      svpi_domain, bool_sort);
  data->minus = Z3_theory_mk_constant(context, theory, minus_sym, sign_sort);
  data->plus = Z3_theory_mk_constant(context, theory, plus_sym, sign_sort);

  Z3_set_new_assignment_callback(theory, NewAssignment<Utvpi, T>);
  Z3_set_push_callback(theory, Push<Utvpi, T>);
  Z3_set_pop_callback(theory, Pop<Utvpi, T>);
  Z3_set_final_check_callback(theory, FinalCheck<Utvpi, T>);
  Z3_set_new_eq_callback(theory, NewEquality<Utvpi, T>);
  Z3_set_reset_callback(theory, Reset<Utvpi, T>);
  Z3_set_restart_callback(theory, Restart<Utvpi, T>);

  return theory;
}

#endif /* UTVPI_THEORY_H */

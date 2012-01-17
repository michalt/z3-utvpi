#ifndef UTVPI_THEORY_H
#define UTVPI_THEORY_H

/*
 * Integration with Z3.
 */

#include <cstdlib>
#include <cstdint>

#include <gmpxx.h>

#include <z3.h>

/*
 * A few helper functions.
 */

void Die(const char *msg) {
  std::cerr << "Died because of: " << msg << std::endl;
  exit(EXIT_FAILURE);
}

void ErrorHandler(Z3_error_code e) {
  std::cout << "Error code: " << e << std::endl;
  Die("incorrect use of Z3.");
}


Z3_context MkContext() {
  Z3_config  cfg = Z3_mk_config();
  Z3_set_param_value(cfg, "MODEL", "true");

  Z3_context ctx = Z3_mk_context(cfg);

#ifdef TRACING
  Z3_trace_to_stderr(ctx);
#endif

  Z3_set_error_handler(ctx, ErrorHandler);

  Z3_del_config(cfg);
  return ctx;
}


/*
 * Magic constants that are used by the theory.
 */

/* Should we put them into UtvpiData? */
static const char *predicate_name = "Utvpi";
static const unsigned int predicate_arity = 5;
static const char *sign_name = "Sign";
static const char *minus_name = "Minus";
static const char *plus_name = "Plus";

/*
 * Stores all the necessary data for the theory. Note that predicate, minus and
 * plus should really be constants, however, this seems problematic. To create
 * them using Z3 API we need to create UtvpiData object first, but that would
 * require creating the predicate, minus and plus...
 */
template <template <typename> class Utvpi, typename T>
struct UtvpiData {
  Z3_func_decl predicate;
  Z3_ast plus, minus;
  Utvpi<T> graph;
};


template <template <typename > class Utvpi, typename T>
void Push(Z3_theory theory) {
  std::cout << "Z3: Push" << std::endl;
  UtvpiData<Utvpi, T> *data =
    static_cast<UtvpiData<Utvpi, T>*>(Z3_theory_get_ext_data(theory));
  data->graph.Push();
}

template <template <typename > class Utvpi, typename T>
void Pop(Z3_theory theory) {
  std::cout << "Z3: Pop" << std::endl;
  UtvpiData<Utvpi, T> *data =
    static_cast<UtvpiData<Utvpi, T>*>(Z3_theory_get_ext_data(theory));
  data->graph.Pop();
}

template <template <typename > class Utvpi, typename T>
void Reset(Z3_theory theory) {
  std::cout << "Z3: Reset" << std::endl;
  UtvpiData<Utvpi, T> *data =
    static_cast<UtvpiData<Utvpi, T>*>(Z3_theory_get_ext_data(theory));
  data->graph.Reset();
}

template <template <typename > class Utvpi, typename T>
void Restart(Z3_theory theory) {
  std::cout << "Z3: Restart" << std::endl;
  UtvpiData<Utvpi, T> *data =
    static_cast<UtvpiData<Utvpi, T>*>(Z3_theory_get_ext_data(theory));
  /* FIXME: check what exactly is the differenc between Reset and Restart.. */
  data->graph.Reset();
}

template <template <typename > class Utvpi, typename T>
Z3_bool FinalCheck(Z3_theory theory) {
  std::cout << "Z3: Final check" << std::endl;
  UtvpiData<Utvpi, T> *data =
    static_cast<UtvpiData<Utvpi, T>*>(Z3_theory_get_ext_data(theory));
  if (data->graph.Satisfiable())
    return Z3_TRUE;
  return Z3_FALSE;
}

Sign ArgToSign(Z3_ast &minus, Z3_ast &plus, Z3_ast &ast) {
  if (ast == plus)
    return Pos;
  assert(ast == minus);
  return Neg;
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

  std::cout << "Z3: Assigned " << Z3_ast_to_string(context, ast)
            << " to " << static_cast<int>(value) << std::endl
            << "Updating graph." << std::endl;

  if (value != Z3_TRUE) {
    std::cout << "NewAssignment: something is not true. Ignoring.."
              << std::endl;
    return;
  }

  Z3_app app_ast = Z3_to_app(context, ast);

  Z3_func_decl predicate_decl = Z3_get_app_decl(context, app_ast);

  assert(predicate_decl != NULL);

  unsigned int num_args = Z3_get_app_num_args(context, app_ast);

  UtvpiData<Utvpi, T> *data =
    static_cast<UtvpiData<Utvpi, T>*>(Z3_theory_get_ext_data(theory));

  if (predicate_decl != data->predicate || num_args != predicate_arity) {
    std::cout << "NewAssignment: unknown format of predicate." << std::endl;
    return;
  }

  Z3_ast arg0 = Z3_get_app_arg (context, app_ast, 0);
  Z3_ast arg1 = Z3_get_app_arg (context, app_ast, 1);
  Z3_ast arg2 = Z3_get_app_arg (context, app_ast, 2);
  Z3_ast arg3 = Z3_get_app_arg (context, app_ast, 3);
  Z3_ast arg4 = Z3_get_app_arg (context, app_ast, 4);

  Sign a = ArgToSign(data->minus, data->plus, arg0);
  Sign b = ArgToSign(data->minus, data->plus, arg2);
  VarId x = Z3_get_ast_id(context, arg2);
  VarId y = Z3_get_ast_id(context, arg4);

  // std::cout << "Argument: " << Z3_ast_to_string(context, arg0) << std::endl;
  // std::cout << "Argument: " << Z3_ast_to_string(context, arg1) << std::endl;
  // std::cout << "Argument: " << Z3_ast_to_string(context, arg2) << std::endl;
  // std::cout << "Argument: " << Z3_ast_to_string(context, arg3) << std::endl;
  // std::cout << "Argument: " << Z3_ast_to_string(context, arg4) << std::endl;

  T c;
  bool success = GetNumeral(context, arg4, c);

  if (!success)
    Die("Failed to get the numeral!");

  data->graph.AddInequality(a, x, b, y, c);
}




template <template <typename > class Utvpi, typename T>
Z3_theory MkTheory(Z3_context ctx) {
  UtvpiData<Utvpi, T> *data = new UtvpiData<Utvpi, T>();
  Z3_theory theory = Z3_mk_theory(ctx, "UTVPI", data);

  Z3_symbol utvpi_sym = Z3_mk_string_symbol(ctx, "Utvpi");
  Z3_symbol sign_sym = Z3_mk_string_symbol(ctx, "Sign");
  Z3_symbol plus_sym = Z3_mk_string_symbol(ctx, "Plus");
  Z3_symbol minus_sym = Z3_mk_string_symbol(ctx, "Minus");

  Z3_sort int_sort = Z3_mk_int_sort(ctx);
  Z3_sort bool_sort = Z3_mk_bool_sort(ctx);
  Z3_sort sign_sort = Z3_theory_mk_sort(ctx, theory, sign_sym); 

  Z3_sort domain[5] = { sign_sort, int_sort, sign_sort, int_sort, int_sort };

  data->predicate = Z3_theory_mk_func_decl(ctx, theory, utvpi_sym, 5, domain, bool_sort);
  data->minus = Z3_theory_mk_constant(ctx, theory, plus_sym, sign_sort);
  data->plus = Z3_theory_mk_constant(ctx, theory, minus_sym, sign_sort);

  Z3_set_new_assignment_callback(theory, NewAssignment<Utvpi, T>);
  Z3_set_push_callback(theory, Push<Utvpi, T>);
  Z3_set_pop_callback(theory, Pop<Utvpi, T>);
  Z3_set_final_check_callback(theory, FinalCheck<Utvpi, T>);

  // probably needed later on:
  // Z3_set_init_search_callback(theory, InitSearch<T>);
  // Z3_set_new_eq_callback(theory, Th_new_eq);
  // Z3_set_reset_callback(theory, Th_reset);
  // Z3_set_restart_callback(theory, Th_restart);
  // Z3_set_new_diseq_callback(theory, Th_new_diseq);
  //
  // probably not needed:
  // Z3_set_new_relevant_callback(theory, Th_new_relevant);
  // Z3_set_delete_callback(theory, Th_delete);
  // Z3_set_reduce_app_callback(theory, Th_reduce_app);
  // Z3_set_new_app_callback(theory, Th_new_app);
  // Z3_set_new_elem_callback(theory, Th_new_elem);

  return theory;
}

#endif /* UTVPI_THEORY_H */

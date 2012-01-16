/*
 * Integration with Z3.
 */

#include <z3.h>

void exitf(const char* message) 
{
  fprintf(stderr,"BUG: %s.\n", message);
  exit(1);
}

void error_handler(Z3_error_code e) 
{
	printf("Error code: %d\n", e);
    exitf("incorrect use of Z3");
}

Z3_context mk_context_custom(Z3_config cfg, Z3_error_handler err) 
{
    Z3_context ctx;
    
    Z3_set_param_value(cfg, "MODEL", "true");
    // Z3_set_param_value(cfg, "ARITH_SOLVER", "0");
    ctx = Z3_mk_context(cfg);
#ifdef TRACING
    Z3_trace_to_stderr(ctx);
#endif
    Z3_set_error_handler(ctx, err);
    
    return ctx;
}


Z3_context mk_context() 
{
    Z3_config  cfg;
    Z3_context ctx;
    cfg = Z3_mk_config();
    ctx = mk_context_custom(cfg, error_handler);
    Z3_del_config(cfg);
    return ctx;
}


/*
 * Magic constants that are used below.
 */

static const char* predicate_name = "Utvpi";
static const unsigned int predicate_arity = 5;

static Z3_func_decl predicate;
static Z3_ast plus, minus;

template <typename T>
void Push(Z3_theory t) {
  printf("Push\n");
}

template <typename T>
void Pop(Z3_theory t) {
  printf("Pop\n");
}

template <typename T>
void Reset(Z3_theory t) {
  printf("Reset\n");
}

template <typename T>
void Restart(Z3_theory t) {
  printf("Restart\n");
}

template <typename T>
Z3_bool FinalCheck(Z3_theory t) {
  printf("Final check\n");
  return Z3_TRUE;
}

Sign ArgToSign(Z3_ast ast) {
  if (ast == plus)
    return Pos;
  return Neg;
}

template <typename T>
void NewAssignment(Z3_theory theory, Z3_ast ast, Z3_bool value) {

  Z3_context context = Z3_theory_get_context(theory);

  std::cout << "Assigned " << Z3_ast_to_string(context, ast)
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

  if (predicate_decl != predicate || num_args != predicate_arity) {
    std::cout << "NewAssignment: unknown format of predicate." << std::endl;
    return;
  }

  Z3_ast arg0 = Z3_get_app_arg (context, app_ast, 0);
  Z3_ast arg1 = Z3_get_app_arg (context, app_ast, 1);
  Z3_ast arg2 = Z3_get_app_arg (context, app_ast, 2);
  Z3_ast arg3 = Z3_get_app_arg (context, app_ast, 3);
  Z3_ast arg4 = Z3_get_app_arg (context, app_ast, 4);

  T *graph = static_cast<T*>(Z3_theory_get_ext_data(theory));
  Sign a = ArgToSign(arg0);
  Sign b = ArgToSign(arg2);
  VarId x = Z3_get_ast_id(context, arg2);
  VarId y = Z3_get_ast_id(context, arg4);

  // std::cout << "Argument: " << Z3_ast_to_string(context, arg0) << std::endl;
  // std::cout << "Argument: " << Z3_ast_to_string(context, arg1) << std::endl;
  // std::cout << "Argument: " << Z3_ast_to_string(context, arg2) << std::endl;
  // std::cout << "Argument: " << Z3_ast_to_string(context, arg3) << std::endl;
  // std::cout << "Argument: " << Z3_ast_to_string(context, arg4) << std::endl;

  // There's no different kind for constants and applications so checking here
  // is probably not the best idea!
  // if (Z3_get_ast_kind(context, arg1) != Z3_VAR_AST) {
    // std::cout << "NewAssignment: " << Z3_ast_to_string(context, arg1)
              // << " is not a variable!" << std::endl;
    // return;
  // }
  // if (Z3_get_ast_kind(context, arg0) != Z3_VAR_AST) {

  int c;
  /* FIXME: this is obviously wrong.. */
  bool success = Z3_get_numeral_int(context, arg4, &c);

  graph->AddInequality(a, x, b, y, c);
}




template <typename T>
Z3_theory MkTheory(Z3_context ctx) {
  T *graph = new T();

  Z3_theory theory = Z3_mk_theory(ctx, "UTVPI", graph);

  Z3_symbol utvpi_sym = Z3_mk_string_symbol(ctx, "Utvpi");
  Z3_symbol sign_sym = Z3_mk_string_symbol(ctx, "Sign");
  Z3_symbol plus_sym = Z3_mk_string_symbol(ctx, "Plus");
  Z3_symbol minus_sym = Z3_mk_string_symbol(ctx, "Minus");

  // td->S                 = Z3_theory_mk_sort(ctx, Th, name); 

  Z3_sort int_sort = Z3_mk_int_sort(ctx);
  Z3_sort bool_sort = Z3_mk_bool_sort(ctx);
  Z3_sort sign_sort = Z3_theory_mk_sort(ctx, theory, sign_sym); 

  Z3_sort domain[5] = { sign_sort, int_sort, sign_sort, int_sort, int_sort };

  // td->f       =
  predicate = Z3_theory_mk_func_decl(ctx, theory, utvpi_sym, 5, domain, bool_sort);

  minus = Z3_theory_mk_constant(ctx, theory, plus_sym, sign_sort);
  plus = Z3_theory_mk_constant(ctx, theory, minus_sym, sign_sort);

  // Z3_sort B             = Z3_mk_bool_sort(ctx);

  Z3_set_new_assignment_callback(theory, NewAssignment<T>);
  Z3_set_push_callback(theory, Push<T>);
  Z3_set_pop_callback(theory, Pop<T>);
  Z3_set_final_check_callback(theory, FinalCheck<T>);

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

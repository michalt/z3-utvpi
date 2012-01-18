#ifndef REASON_H
#define REASON_H

#include <z3.h>
#include <gmpxx.h>

#include "common.h"
#include "utvpi_theory_data.h"

/*
 * We can create a numeral in Z3 in a number of ways. This allows us to make the
 * choce depending on the type of our domain.
 */
template <typename T>
Z3_ast MkNumeral(Z3_context context, const T &num);

template <>
Z3_ast MkNumeral(Z3_context context, const int &num) {
  return Z3_mk_int(context, num, Z3_mk_int_sort(context));
}

template <>
Z3_ast MkNumeral(Z3_context context, const long long &num) {
  return Z3_mk_int64(context, num, Z3_mk_int_sort(context));
}

template <>
Z3_ast MkNumeral(Z3_context context, const mpz_class &num) {
  return Z3_mk_numeral(context, num.get_str().c_str(), Z3_mk_int_sort(context));
}

template <>
Z3_ast MkNumeral(Z3_context context, const mpq_class &num) {
  return Z3_mk_numeral(context, num.get_str().c_str(),
      Z3_mk_real_sort(context));
}

/*
 * Reason for adding some edge
 */

class Reason {
  public:
    virtual Z3_ast MkAst(Z3_context, std::unordered_map<VarId, Z3_ast> &id_to_ast,
        Z3_func_decl utvpi, Z3_func_decl svpi, Z3_ast minus, Z3_ast plus) const = 0;
    virtual std::ostream& Print(std::ostream& out) const = 0;
};

typedef typename std::shared_ptr<Reason> ReasonPtr;


std::ostream& operator<<(std::ostream& out, const Reason &s) {
  s.Print(out);
  return out;
}


Z3_ast SignToArg2(Z3_ast minus, Z3_ast plus, Sign sign) {
  if (sign == Pos)
    return plus;
  return minus;
}


class Equality : public Reason {
  public:
    Equality(VarId x, VarId y) : x_(x), y_(y) { }

    Z3_ast MkAst(Z3_context context, std::unordered_map<VarId, Z3_ast> &id_to_ast,
        Z3_func_decl utvpi, Z3_func_decl svpi, Z3_ast minus, Z3_ast plus) const {
      Z3_ast ast_x = id_to_ast[x_];
      Z3_ast ast_y = id_to_ast[y_];
      return Z3_mk_eq(context, ast_x, ast_y);
    }

    std::ostream& Print(std::ostream &out) const {
      out << x_ << " = " << y_;
      return out;
    }

  private:
    VarId x_;
    VarId y_;
};


template <typename T>
class Inequality1 : public Reason {
  public:
    Inequality1(Sign a, VarId x, T c) : a_(a), x_(x), c_(c) { }

    Z3_ast MkAst(Z3_context context, std::unordered_map<VarId, Z3_ast> &id_to_ast,
        Z3_func_decl utvpi, Z3_func_decl svpi, Z3_ast minus, Z3_ast plus) const {
      Z3_ast args[] = { SignToArg2(minus, plus, a_)
                      , id_to_ast[x_]
                      , MkNumeral(context, c_)
                      };
      Z3_ast ast = Z3_mk_app(context, svpi, 3, args);
#ifdef DEBUG
      std::cout << "Created: " << Z3_ast_to_string(context, ast) << std::endl;
#endif
      return ast;
    }

    std::ostream& Print(std::ostream &out) const {
      out << a_ << x_ << " <= " << c_;
      return out;
    }

  private:
    Sign a_;
    VarId x_;
    T c_;
};

template <typename T>
class Inequality2 : public Reason {
  public:
    Inequality2(Sign a, VarId x, Sign b, VarId y, T c)
      : a_(a), b_(b), x_(x), y_(y), c_(c) { }

    Z3_ast MkAst(Z3_context context, std::unordered_map<VarId, Z3_ast> &id_to_ast,
        Z3_func_decl utvpi, Z3_func_decl svpi, Z3_ast minus, Z3_ast plus) const {

      Z3_ast args[] = { SignToArg2(minus, plus, a_)
                      , id_to_ast[x_]
                      , SignToArg2(minus, plus, b_)
                      , id_to_ast[y_]
                      , MkNumeral(context, c_)
                      };
      Z3_ast ast = Z3_mk_app(context, utvpi, 5, args);
#ifdef DEBUG
      std::cout << "Created: " << Z3_ast_to_string(context, ast) << std::endl;
#endif
      return ast;
    }

    std::ostream& Print(std::ostream &out) const {
      out << a_ << x_  << " + " << b_ << y_ << " <= " << c_;
      return out;
    }

  private:
    Sign a_, b_;
    VarId x_, y_;
    T c_;
};


#endif /* REASON_H */

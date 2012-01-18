#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <unordered_map>
#include <utility>

#include <gmpxx.h>

enum Sign { Pos = 0, Neg = 1 };

inline Sign negate(Sign s) {
  if (s == Neg)
    return Pos;
  return Neg;
}

std::ostream& operator<<(std::ostream& out, const Sign& s) {
  if (s == Neg)
    out << "-";
  else
    out << "+";
  return out;
}

typedef unsigned int VarId;

typedef std::pair<VarId, Sign> SignedVarId;

inline SignedVarId negate(SignedVarId s_varid) {
  return SignedVarId(s_varid.first, negate(s_varid.second));
}

std::ostream& operator<<(std::ostream& out, const SignedVarId& pair) {
  out << pair.first << pair.second;
  return out;
}

namespace std {
  template<>
  struct hash<SignedVarId> {
    std::size_t operator()(SignedVarId const &var) const {
      return  var.second << var.first;
    }
  };
}

/*
 * This is necessary for Boost's Bellman-Ford to work correctly --- closed_plus
 * is used for addition. The problem is that with mpz_class that has no
 * std::numeric_limits::max it assumes that zero is max value and no matter what
 * is added to zero the result is zero.. This makes sense in case of fixed size
 * integers but not for e.g. mpz_class..
 */
template <typename T>
struct custom_plus : public boost::closed_plus<T> { };

template <>
struct custom_plus<mpz_class> {
  mpz_class operator()(const mpz_class& a, const mpz_class& b) const {
    return a + b;
  }
};

template <>
struct custom_plus<mpq_class> {
  mpq_class operator()(const mpq_class& a, const mpq_class& b) const {
    return a + b;
  }
};

#endif /* COMMON_H */

#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <unordered_map>
#include <utility>

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

#endif /* COMMON_H */

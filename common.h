#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <unordered_map>
#include <utility>

enum Sign { Pos, Neg };

std::ostream& operator<<(std::ostream& out, const Sign& s) {
  if (s == Neg)
    out << "-";
  return out;
}

typedef unsigned int VarId;

typedef std::pair<Sign, unsigned int> SignedVarId;

inline Sign negate(Sign s) {
  if (s == Neg)
    return Pos;
  return Neg;
}

namespace std {
  template<>
    struct hash<SignedVarId> {
      std::size_t operator()(SignedVarId const &var) const {
        return static_cast<size_t>(var.first) + static_cast<size_t>(var.second);
      }
    };
}

#endif /* COMMON_H */

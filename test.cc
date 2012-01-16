#include <iostream>
#include <utility>
#include <unordered_map>

// FIXME: remove
// #include <boost/graph/adjacency_list.hpp>
// #include <boost/graph/bellman_ford_shortest_paths.hpp>

#include <boost/graph/graph_traits.hpp>
// #include <boost/graph/adjacency_list.hpp>

#include <limits>

#include <gmpxx.h>

#include "z3.h"
#include "utvpi_graph_z.h"
#include "utvpi_graph_q.h"

// using namespace boost;
// struct EdgeProperties {
  // int weight;
// };



void die(const char *msg) {
  std::cerr << "Cause of death: " << std:: endl
            << msg << std::endl;
}

void error_handler(Z3_error_code e) {
  std::cout << "Error code: " << e << std::endl;
  die("Incorrect use of Z3.");
}



namespace std {

template <>
class numeric_limits<mpz_class> {
private:
   typedef mpz_class Type;

public:
   static const bool is_specialized = false;
   static const int digits = 0;
   static const int digits10 = 0;
   static const bool is_signed = true;
   static const bool is_integer = true;
   static const bool is_exact = true;
   static const int radix = 2;
   static const int min_exponent = 0;
   static const int min_exponent10 = 0;
   static const int max_exponent = 0;
   static const int max_exponent10 = 0;
   static const bool has_infinity = false;
   static const bool has_quiet_NaN =  false;
   static const bool has_signaling_NaN = false;
   static const float_denorm_style has_denorm = denorm_absent;
   static const bool has_denorm_loss = false;
   static const bool is_iec559 = false;
   static const bool is_bounded = false;
   static const bool is_modulo = false;
   static const bool traps = false;
   static const bool tinyness_before = false;
   static const float_round_style round_style = round_toward_zero;

   static Type min() {
     return static_cast<Type>(0);
   }

   static Type max() {
     return static_cast<Type>(0);
   }

   static Type epsilon() {
     return static_cast<Type>(1);
   }

   static Type round_error() {
     return static_cast<Type>(1);
   }

   static Type infinity() {
     return static_cast<Type>(0);
   }

   static Type quiet_NaN() {
     return static_cast<Type>(0);
   }

   static Type denorm_min() {
     return static_cast<Type>(1);
   }
};
}

/*
 * Template specialization to work around a bug in BGL that defines closed_plus
 * in the same way no matter if std::numeric_limits::has_infinity is true or
 * false. This causes BGL to assume that mpz_class to have infinity value equal
 * to 0, which obviously results in wrong results for some algorithms.
 */
namespace boost {
  template <>
  struct closed_plus<mpz_class> {
    mpz_class operator()(const mpz_class& a, const mpz_class& b) const {
      return a + b;
    }
  };
}

namespace boost {
  template <>
  struct closed_plus<mpq_class> {
    mpq_class operator()(const mpq_class& a, const mpq_class& b) const {
      return a + b;
    }
  };
}


class Foo {
  public:
    Foo() : v_(0) { }

    // Foo(mpz_class v) : v_(v) { }

    Foo(int x) : v_(x) { }

    // Foo(Foo x) : v_(x.v_) { }

    Foo(const Foo& x) : v_(x.v_) { }

    // Foo operator=(const Foo& x) {

    // }

    Foo operator+(const Foo& x) const {
      // return Foo(mpz_class(v_ + x.v_));
      return Foo((v_ + x.v_));
    }

    Foo operator-(const Foo& x) const {
      return Foo((v_ - x.v_));
      // return Foo(mpz_class(v_ - x.v_));
    }

    bool operator!=(const Foo& x) const {
      return v_ != x.v_;
    }

    bool operator==(const Foo& x) const {
      return v_ == x.v_;
    }

    Foo operator%(const Foo& x) const {
      return Foo((v_ % x.v_));
      // return Foo(mpz_class(v_ % x.v_));
    }

    // Foo operator*(Foo x) {
      // return Foo(mpz_class(v_ * x.v_));
    // }

    Foo operator*(const Foo& x) const {
      // return Foo(mpz_class(v_ * x.v_));
      return Foo((v_ * x.v_));
    }

    bool operator<(const Foo& x) const {
      return v_ < x.v_;
    }

    bool operator<=(const Foo& x) const {
      return v_ <= x.v_;
    }

    friend std::ostream& operator<<(std::ostream& out, const Foo& x);

  private:
    int v_;
};

std::ostream& operator<<(std::ostream& out, const Foo& x) {
  out << x.v_;
  return out;
}


int main(int argc, char *argv[]) {
  // UtvpiGraphZ<mpz_class> graph;
  // UtvpiGraphZ<mpq_class> graph;
  Foo x, y;
  x = -10;
  y = Foo(2) * x;

  if (x <= y)
    std::cout << "FOOBAR 1" << std::endl;
  if (x < y)
    std::cout << "FOOBAR 2" << std::endl;
  if (y <= x)
    std::cout << "FOOBAR 3" << std::endl;
  if (y < x)
    std::cout << "FOOBAR 4" << std::endl;


  // UtvpiGraphZ<int> graph;
  // UtvpiGraphZ<Foo> graph;
  UtvpiGraphQ<mpq_class> graph;
  // UtvpiGraphQ<double> graph;
  graph.AddInequality(Pos, 2, Pos, 1, -5);
  graph.AddInequality(Pos, 4, Neg, 1, 4);
  graph.AddInequality(Neg, 4, Neg, 1, 3);
  graph.AddInequality(Pos, 3, Neg, 2, 2);
  graph.AddInequality(Neg, 3, Neg, 2, 1);
  // graph.AddInequality(Pos, 1, 100);
  graph.Print();
  std::cout << graph.CheckSat() << std::endl;



  /*
  typedef typename boost::adjacency_list<boost::vecS,
          boost::vecS,
          boost::directedS,
          SignedVarId,
          // boost::property<edge_weight_t, int> > Graph;
          int > Graph;

  typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
  typedef typename boost::graph_traits<Graph>::edge_descriptor   Edge;
  typedef typename boost::graph_traits<Graph>::vertex_iterator   VertexIter;
  typedef typename boost::graph_traits<Graph>::edge_iterator     EdgeIter;
  typedef typename boost::graph_traits<Graph>::out_edge_iterator OutEdgeIter;

  typedef std::unordered_map<SignedVarId, Vertex> VertexMap;



  std::size_t size = 10;

  std::vector<int> distance(size);
  std::vector<Vertex> parent(size);

  Graph graph_(size);

  Vertex special = boost::add_vertex(graph_);
  // property_map<graph_t, edge_weight_t>::type weightmap = get(edge_weight, g); 
  // auto weightmap = get(edge_bundle, g);

  bool r = boost::bellman_ford_shortest_paths
    (graph_, size, boost::root_vertex(special).
                      weight_map(boost::get(boost::edge_bundle, graph_)).
                      distance_map(&distance[0]).
                      predecessor_map(&parent[0]));
  */

  /*
  enum { u, v, x, y, z, N };
  char name[] = { 'u', 'v', 'x', 'y', 'z' };
  typedef std::pair < int, int >E;
  const int n_edges = 10;
  E edge_array[] = { E(u, y), E(u, x), E(u, v), E(v, u),
      E(x, y), E(x, v), E(y, v), E(y, z), E(z, u), E(z,x) };
  int weight[n_edges] = { -4, 8, 5, -2, 9, -3, 7, 2, 6, 7 };

  typedef adjacency_list < vecS, vecS, directedS,
    no_property, EdgeProperties> Graph;
  Graph g(edge_array, edge_array + n_edges, N);
  graph_traits < Graph >::edge_iterator ei, ei_end;
  property_map<Graph, int EdgeProperties::*>::type 
    weight_pmap = get(&EdgeProperties::weight, g);
  int i = 0;
  for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei, ++i)
    weight_pmap[*ei] = weight[i];

  std::vector<int> distance(N, (std::numeric_limits < short >::max)());
  std::vector<std::size_t> parent(N);
  for (i = 0; i < N; ++i)
    parent[i] = i;
  distance[z] = 0;

  bool r = bellman_ford_shortest_paths
    (g, int (N), weight_map(weight_pmap).distance_map(&distance[0]).
     predecessor_map(&parent[0]));
  */
}

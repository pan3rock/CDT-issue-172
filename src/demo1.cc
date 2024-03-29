#include "predicates.hpp"
#include <CDT.h>
#include <cmath>
#include <fmt/format.h>
#include <vector>

using V2d = CDT::V2d<double>;

int main() {
  std::vector<double> x{1.000000000000, 0.000000000000, 0.500000000000,
                        0.750000000000, 0.750000000000, 0.750000000000,
                        0.875000000000, 0.812500000000, 0.750000000000,
                        0.781250000000, 0.796875000000, 0.789062500000,
                        0.789062500000};
  std::vector<double> y{0.000000000000, 0.100000000000, 0.050000000000,
                        0.000000000000, 0.025000000000, 0.012500000000,
                        0.012500000000, 0.018750000000, 0.018750000000,
                        0.015625000000, 0.017187500000, 0.014843750000,
                        0.016406250000};

  std::vector<V2d> nodes;
  for (size_t i = 0; i < x.size(); ++i) {
    nodes.push_back(V2d::make(x[i], y[i]));
  }

  CDT::Triangulation<double> cdt;
  cdt.insertVertices(nodes);
  cdt.eraseSuperTriangle();
  auto elements = cdt.triangles;
  auto edges_us = CDT::extractEdgesFromTriangles(elements);

  for (size_t i = 0; i < elements.size(); ++i) {
    auto v = elements[i].vertices;
    double x1 = nodes[v[0]].x;
    double y1 = nodes[v[0]].y;
    double x2 = nodes[v[1]].x;
    double y2 = nodes[v[1]].y;
    double x3 = nodes[v[2]].x;
    double y3 = nodes[v[2]].y;
    auto value = predicates::adaptive::orient2d(x1, y1, x2, y2, x3, y3);
    if (std::abs(value) < 1.0e-10) {
      auto v1 = predicates::adaptive::orient2d(x1, y1, x2, y2, x3, y3);
      auto v2 = predicates::adaptive::orient2d(x1, y1, x3, y3, x2, y2);
      auto v3 = predicates::adaptive::orient2d(x3, y3, x2, y2, x1, y1);
      fmt::print("({:d}, {:d}, {:d}): {:16.5e}\n", v[0], v[1], v[2], v1);
      fmt::print("({:d}, {:d}, {:d}): {:16.5e}\n", v[0], v[2], v[1], v2);
      fmt::print("({:d}, {:d}, {:d}): {:16.5e}\n", v[2], v[1], v[0], v3);
      double z1[] = {x1, y1};
      double z2[] = {x2, y2};
      double z3[] = {x3, y3};
      fmt::print("another predicates: {:16.5e}\n",
                 predicates::orient2d(z1, z2, z3));
      fmt::print("another predicates: {:16.5e}\n",
                 predicates::orient2d(z1, z3, z2));
      fmt::print("another predicates: {:16.5e}\n",
                 predicates::orient2d(z3, z2, z1));
    }
  }

  // double x1 = 0.5000000000000000000000000000000000000000;
  // double y1 = 0.0500000000000000027755575615628913510591;
  // double x2 = 0.7500000000000000000000000000000000000000;
  // double y2 = 0.0250000000000000013877787807814456755295;
  // double x3 = 0.8125000000000000000000000000000000000000;
  // double y3 = 0.0187500000000000027755575615628913510591;
  double x1 = 0.50000;
  double y1 = 0.05000;
  double x2 = 0.75000;
  double y2 = 0.02500;
  double x3 = 0.81250;
  double y3 = 0.01875;
  auto v1 = predicates::adaptive::orient2d(x1, y1, x2, y2, x3, y3);
  auto v2 = predicates::adaptive::orient2d(x1, y1, x3, y3, x2, y2);
  auto v3 = predicates::adaptive::orient2d(x3, y3, x2, y2, x1, y1);
  fmt::print("({:d}, {:d}, {:d}): {:16.5e}\n", 1, 2, 3, v1);
  fmt::print("({:d}, {:d}, {:d}): {:16.5e}\n", 1, 3, 2, v2);
  fmt::print("({:d}, {:d}, {:d}): {:16.5e}\n", 3, 2, 1, v3);

  // double z1[] = {0.5000000000000000000000000000000000000000,
  //                0.0500000000000000027755575615628913510591};
  // double z2[] = {0.7500000000000000000000000000000000000000,
  //                0.0250000000000000013877787807814456755295};
  // double z3[] = {0.8125000000000000000000000000000000000000,
  //                0.0187500000000000027755575615628913510591};
  double z1[] = {0.50000, 0.05000};
  double z2[] = {0.75000, 0.02500};
  double z3[] = {0.81250, 0.01875};
  fmt::print("another predicates: {:16.5e}\n",
             predicates::orient2d(z1, z2, z3));
  fmt::print("another predicates: {:16.5e}\n",
             predicates::orient2d(z1, z3, z2));
  fmt::print("another predicates: {:16.5e}\n",
             predicates::orient2d(z3, z2, z1));

  return 0;
}
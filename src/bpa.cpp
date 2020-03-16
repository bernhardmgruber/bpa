#include "bpa.h"

#include <boost/math/constants/constants.hpp>
#include <algorithm>
#include <deque>
#include <optional>
#include <string>
#include <iostream>

#include <glm/gtx/io.hpp>

#include "IO.h"

using glm::vec3;
using glm::ivec3;

namespace bpa {
	namespace {
		constexpr auto debug = false;
		constexpr auto pi = boost::math::constants::pi<float>();

		struct Point {
			vec3 pos;
			bool used = false;
		};

		struct Edge {
			Point* a;
			Point* b;
			Point* opposite;
			vec3 center;
			Edge* prev;
			Edge* next;
			bool active = true;
		};

		struct Face : std::array<Point*, 3>{
			auto normal() const {
				return glm::normalize(glm::cross((*this)[0]->pos - (*this)[1]->pos, (*this)[0]->pos - (*this)[2]->pos));
			}
		};

		struct Cell {
			std::vector<Point> points;
		};

		struct Grid {
			Grid(vec3 lower, vec3 upper, float cellSize)
				: lower(lower), upper(upper), cellSize(cellSize) {

				const auto sizes = upper - lower;
				dims = ivec3{glm::ceil(sizes / cellSize)};
				dims = glm::max(dims, ivec3{1}); // for planar clouds
				cells.resize(dims.x * dims.y * dims.z);
			}

			auto cellIndex(vec3 point) -> ivec3 {
				const auto index = ivec3{(point - lower) / cellSize};
				return glm::clamp(index, ivec3{}, dims - 1);
			}

			auto cell(ivec3 index) -> Cell& {
				return cells[index.z * dims.x * dims.y + index.y * dims.x + index.x];
			}

			auto neighborhood27(vec3 point) -> std::vector<Point*> {
				std::vector<Point*> result;
				const auto centerIndex = cellIndex(point);
				for (auto xOff : {-1, 0, 1}) {
					for (auto yOff : {-1, 0, 1}) {
						for (auto zOff : {-1, 0, 1}) {
							const auto index = centerIndex + ivec3{xOff, yOff, zOff};
							if (index.x < 0 || index.x >= dims.x) continue;
							if (index.y < 0 || index.y >= dims.y) continue;
							if (index.z < 0 || index.z >= dims.z) continue;
							auto& c = cell(index);
							std::transform(begin(c.points), end(c.points), std::back_inserter(result), [](auto& point) { return &point; });
						}
					}
				}
				return result;
			}

			auto sphericalNeighborhood(vec3 point, std::initializer_list<vec3> ignore) -> std::vector<Point*> {
				auto result = neighborhood27(point);
				result.erase(std::remove_if(begin(result), end(result), [&](Point* p) {
					return glm::length(p->pos - point) > cellSize || std::find(begin(ignore), end(ignore), p->pos) != end(ignore);
				}), end(result));
				return result;
			}

			vec3 lower;
			vec3 upper;
			float cellSize;
			ivec3 dims;
			std::vector<Cell> cells;
		};

		auto buildGrid(const std::vector<vec3>& points, float radius) {
			vec3 lower = points.front();
			vec3 upper = points.front();

			for (const auto& p : points) {
				for (auto i = 0; i < 3; i++) {
					lower[i] = std::min(lower[i], p[i]);
					upper[i] = std::max(upper[i], p[i]);
				}
			}

			Grid g(lower, upper, radius * 2);
			for (const auto& p : points)
				g.cell(g.cellIndex(p)).points.push_back({p});
			return g;
		}

		auto computeBallCenter(Face f, float radius) -> std::optional<vec3> {
			// from https://gamedev.stackexchange.com/questions/60630/how-do-i-find-the-circumcenter-of-a-triangle-in-3d
			const vec3 ac = f[2]->pos - f[0]->pos;
			const vec3 ab = f[1]->pos - f[0]->pos;
			const vec3 abXac = glm::cross(ab, ac);
			const vec3 toCircumCircleCenter = (glm::cross(abXac, ab) * glm::dot(ac, ac) + glm::cross(ac, abXac) * glm::dot(ab, ab)) / (2 * glm::dot(abXac, abXac));
			const vec3 circumCircleCenter = f[0]->pos + toCircumCircleCenter;

			const auto heightSquared = radius * radius - glm::dot(toCircumCircleCenter, toCircumCircleCenter);
			if (heightSquared < 0)
				return {};
			auto ballCenter = circumCircleCenter + f.normal() * std::sqrt(heightSquared);
			return ballCenter;
		}

		auto ballIsEmpty(vec3 ballCenter, const std::vector<Point*>& points, float radius) -> bool {
			return std::all_of(begin(points), end(points), [&](Point* p) {
				return glm::dot(p->pos - ballCenter, p->pos - ballCenter) > radius * radius - 1e-4f; // TODO epsilon
			});
		}

		struct SeedResult {
			Face f;
			vec3 ballCenter;
		};

		auto findSeedTriangle(Grid& grid, float radius) -> std::optional<SeedResult> {
			for (auto& cell : grid.cells) {
				for (auto& p1 : cell.points) {
					auto neighborhood = grid.sphericalNeighborhood(p1.pos, {p1.pos});
					std::sort(begin(neighborhood), end(neighborhood), [&](Point* a, Point* b) {
						return glm::length(a->pos - p1.pos) < glm::length(b->pos - p1.pos);
					});

					for (auto& p2 : neighborhood) {
						for (auto& p3 : neighborhood) {
							if (p2 == p3) continue;
							Face f{{&p1, p2, p3}};
							const auto ballCenter = computeBallCenter(f, radius);
							if (ballCenter && ballIsEmpty(ballCenter.value(), neighborhood, radius)) {
								p1.used = true;
								p2->used = true;
								p3->used = true;
								return SeedResult{f, ballCenter.value()};
							}
						}
					}
				}
			}
			return {};
		}

		auto getActiveEdge(std::deque<Edge>& front) -> std::optional<Edge*> {
			for (auto& e : front)
				if (e.a && e.b && e.active)
					return &e;
			return {};
		}

		struct PivotResult {
			Point* p;
			vec3 center;
		};

		auto ballPivot(const Edge* e, Grid& grid, float radius) -> std::optional<PivotResult> {
			const auto m = (e->a->pos + e->b->pos) / 2.0f;
			const auto oldCenterVec = glm::normalize(e->center - m);
			auto neighborhood = grid.sphericalNeighborhood(m, {e->a->pos, e->b->pos, e->opposite->pos});

			static auto counter = 0;
			counter++;
			if (debug) {
				saveTriangles(std::to_string(counter) + "_pivot_edge.stl", std::vector<Triangle>{Triangle{e->a->pos, e->a->pos, e->b->pos}});

				std::vector<vec3> points(neighborhood.size());
				std::transform(begin(neighborhood), end(neighborhood), begin(points), [](const Point* p) { return p->pos; });
				savePoints(std::to_string(counter) + "_neighborhood.ply", points);

				if (debug) std::cout << "pivoting " << counter << "\n";
			}

			auto smallestAngle = std::numeric_limits<float>::max();
			Point* pointWithSmallestAngle = nullptr;
			vec3 centerOfSmallest{};
			std::cout << "pivoting edge a=" << e->a->pos << " b=" << e->b->pos << " op=" << e->opposite->pos << ". testing " << neighborhood.size() << " neighbors\n";
			for (const auto& p : neighborhood) {
				auto newFaceNormal = Triangle{e->b->pos, e->a->pos, p->pos}.normal();

				const auto c = computeBallCenter(Face{{e->b, e->a, p}}, radius);
				if (!c) {
					std::cout << "    " << p->pos << " center computation failed\n";
					continue;
				}

				if (debug) {
					static auto counter2 = 0;
					counter2++;
					saveTriangles(std::to_string(counter) + "_" + std::to_string(counter2) + "_face.stl", std::vector<Triangle>{Triangle{e->a->pos, e->b->pos, p->pos}});
					savePoints(std::to_string(counter) + "_" + std::to_string(counter2) + "_ballcenter.ply", {c.value()});
				}

				// this check is not in the paper: the ball center must always be above the triangle
				const auto newCenterVec = glm::normalize(c.value() - m);
				if (glm::dot(newCenterVec, newFaceNormal) < 0) {
					std::cout << "    " << p->pos << " ball center " << c.value() << " underneath triangle\n";
					continue;
				}

				auto angle = std::acos(std::clamp(glm::dot(oldCenterVec, newCenterVec), -1.0f, 1.0f));
				if (glm::dot(glm::cross(newCenterVec, oldCenterVec), e->b->pos - e->a->pos) < 0)
					angle += pi;
				std::cout << "    " << p->pos << " center " << c.value() << " angle " << angle << "\n";

				if (angle < smallestAngle && ballIsEmpty(c.value(), neighborhood, radius)) {
					smallestAngle = angle;
					pointWithSmallestAngle = p;
					centerOfSmallest = c.value();
				}
			}

			if (smallestAngle != std::numeric_limits<float>::max()) {
				if (debug) savePoints(std::to_string(counter) + "_candidate.ply", {pointWithSmallestAngle->pos});
				return PivotResult{pointWithSmallestAngle, centerOfSmallest};
			}
			return {};
		}

		auto notUsed(const Point* p) -> bool {
			return !p->used;
		}

		auto onFront(const Point* p, const std::deque<Edge>& front) -> bool {
			return std::any_of(begin(front), end(front), [&](const Edge& e) {
				return p == e.a || p == e.b;
			});
		}

		void remove(Edge* edge, std::deque<Edge>& front) {
			const auto it = std::find_if(begin(front), end(front), [&](const Edge& e) {
				return edge == &e;
			});
			assert(it != end(front));
			// we cannot erase because this moves all the Edges in memory, so we mark it
			*it = Edge{};
		}

		void outputTriangle(Face f, std::vector<Triangle>& triangles) {
			triangles.push_back({f[0]->pos, f[1]->pos, f[2]->pos});
		}

		auto join(Edge* e_ij, Point* o_k, vec3 o_k_ballCenter, std::deque<Edge>& front, std::vector<Triangle>& triangles) -> std::tuple<Edge*, Edge*> {
			outputTriangle({e_ij->a, o_k, e_ij->b}, triangles);

			auto& e_ik = front.emplace_back(Edge{e_ij->a, o_k, e_ij->b, o_k_ballCenter});
			auto& e_kj = front.emplace_back(Edge{o_k, e_ij->b, e_ij->a, o_k_ballCenter});

			e_ik.next = &e_kj;
			e_ik.prev = e_ij->prev;
			e_ij->prev->next = &e_ik;

			e_kj.prev = &e_ik;
			e_kj.next = e_ij->next;
			e_ij->next->prev = &e_kj;

			remove(e_ij, front);

			return {&e_ik, &e_kj};
		}

		void glue(Edge* a, Edge* b, std::deque<Edge>& front) {
			// case 1
			if (a->next == b && a->prev == b && b->next == a && b->prev == a) {
				remove(a, front);
				remove(b, front);
				return;
			}
			// case 2
			if (a->next == b && b->prev == a) {
				a->prev->next = b->next;
				b->next->prev = a->prev;
				remove(a, front);
				remove(b, front);
				return;
			}
			if (a->prev == b && b->next == a) {
				a->next->prev = b->prev;
				b->prev->next = a->next;
				remove(a, front);
				remove(b, front);
				return;
			}
			// case 3
			a->prev->next = b->next;
			b->next->prev = a->prev;
			a->next->prev = b->prev;
			b->prev->next = a->next;
			remove(a, front);
			remove(b, front);

			std::cerr << "unsuccessful glue\n";
			assert(false);
		}
	}

	auto reconstruct(const std::vector<vec3>& points, float radius) -> std::vector<Triangle> {
		auto grid = buildGrid(points, radius);

		const auto seedResult = findSeedTriangle(grid, radius);
		if (!seedResult) {
			std::cerr << "No seed triangle found\n";
			return {};
		}

		std::vector<Triangle> triangles;
		std::deque<Edge> front;

		auto [seed, ballCenter] = *seedResult;
		outputTriangle(seed, triangles);
		front.push_back({seed[0], seed[1], seed[2], ballCenter});
		front.push_back({seed[1], seed[2], seed[0], ballCenter});
		front.push_back({seed[2], seed[0], seed[1], ballCenter});
		front[0].prev = &front[2];
		front[0].next = &front[1];
		front[1].prev = &front[0];
		front[1].next = &front[2];
		front[2].prev = &front[1];
		front[2].next = &front[0];

		while (auto e_ij = getActiveEdge(front)) {
			if (const auto o_k = ballPivot(e_ij.value(), grid, radius); o_k && (notUsed(o_k->p) || onFront(o_k->p, front))) {
				o_k->p->used = true;
				outputTriangle({{e_ij.value()->a, o_k->p, e_ij.value()->b}}, triangles);
				auto [e_ik, e_kj] = join(e_ij.value(), o_k->p, o_k->center, front, triangles);
				if (auto e_ki = std::find_if(begin(front), end(front), [&](const Edge& e) { return e_ik->a == e.b && e_ik->b == e.a; }); e_ki != end(front)) glue(e_ik, &*e_ki, front);
				if (auto e_jk = std::find_if(begin(front), end(front), [&](const Edge& e) { return e_kj->a == e.b && e_kj->b == e.a; }); e_jk != end(front)) glue(e_kj, &*e_jk, front);
			} else
				e_ij.value()->active = false;
		}

		if (debug) {
			std::vector<Triangle> boundaryEdges;
			for (const auto& e : front) {
				if (e.a != nullptr)
					boundaryEdges.push_back({e.a->pos, e.a->pos, e.b->pos});
			}
			saveTriangles("boundaryEdges.stl", boundaryEdges);

		}

		return triangles;
	}
}
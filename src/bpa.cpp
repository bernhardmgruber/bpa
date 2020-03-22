#include "bpa.h"

#include <boost/math/constants/constants.hpp>
#include <algorithm>
#include <deque>
#include <optional>
#include <string>
#include <iostream>
#include <sstream>
#include <numeric>

#include <glm/gtx/io.hpp>

#include "IO.h"

using glm::vec3;
using glm::ivec3;

namespace bpa {
	namespace {
		constexpr auto debug = false;
		constexpr auto pi = boost::math::constants::pi<float>();

		struct MeshEdge;

		struct MeshPoint {
			vec3 pos;
			vec3 normal;
			bool used = false;
			std::vector<MeshEdge*> edges;
		};

		enum class EdgeStatus {
			active,
			inner,
			boundary
		};

		struct MeshEdge {
			MeshPoint* a;
			MeshPoint* b;
			MeshPoint* opposite;
			vec3 center;
			MeshEdge* prev;
			MeshEdge* next;
			EdgeStatus status = EdgeStatus::active;
		};

		struct MeshFace : std::array<MeshPoint*, 3>{
			auto normal() const {
				return glm::normalize(glm::cross((*this)[0]->pos - (*this)[1]->pos, (*this)[0]->pos - (*this)[2]->pos));
			}
		};

		struct Cell {
			std::vector<MeshPoint> points;
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

			auto neighborhood27(vec3 point) -> std::vector<MeshPoint*> {
				std::vector<MeshPoint*> result;
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

			auto sphericalNeighborhood(vec3 point, std::initializer_list<vec3> ignore) -> std::vector<MeshPoint*> {
				auto result = neighborhood27(point);
				result.erase(std::remove_if(begin(result), end(result), [&](MeshPoint* p) {
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

		auto buildGrid(const std::vector<Point>& points, float radius) {
			vec3 lower = points.front().pos;
			vec3 upper = points.front().pos;

			for (const auto& p : points) {
				for (auto i = 0; i < 3; i++) {
					lower[i] = std::min(lower[i], p.pos[i]);
					upper[i] = std::max(upper[i], p.pos[i]);
				}
			}

			Grid g(lower, upper, radius * 2);
			for (const auto& p : points)
				g.cell(g.cellIndex(p.pos)).points.push_back({p.pos, p.normal});
			return g;
		}

		auto computeBallCenter(MeshFace f, float radius) -> std::optional<vec3> {
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

		auto ballIsEmpty(vec3 ballCenter, const std::vector<MeshPoint*>& points, float radius) -> bool {
			return std::all_of(begin(points), end(points), [&](MeshPoint* p) {
				return glm::dot(p->pos - ballCenter, p->pos - ballCenter) > radius * radius - 1e-4f; // TODO epsilon
			});
		}

		struct SeedResult {
			MeshFace f;
			vec3 ballCenter;
		};

		auto findSeedTriangle(Grid& grid, float radius) -> std::optional<SeedResult> {
			for (auto& cell : grid.cells) {
				const auto avgNormal = normalize(std::reduce(begin(cell.points), end(cell.points), vec3{}, [](vec3 acc, const MeshPoint& p) {
					return acc + p.normal;
				}));
				for (auto& p1 : cell.points) {
					auto neighborhood = grid.sphericalNeighborhood(p1.pos, {p1.pos});
					std::sort(begin(neighborhood), end(neighborhood), [&](MeshPoint* a, MeshPoint* b) {
						return glm::length(a->pos - p1.pos) < glm::length(b->pos - p1.pos);
					});

					for (auto& p2 : neighborhood) {
						for (auto& p3 : neighborhood) {
							if (p2 == p3) continue;
							MeshFace f{{&p1, p2, p3}};
							if (dot(f.normal(), avgNormal) < 0) // only accept triangles which's normal points into the same half-space as the average normal of this cell's points
								continue;
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

		auto getActiveEdge(std::vector<MeshEdge*>& front) -> std::optional<MeshEdge*> {
			while (!front.empty()) {
				auto* e = front.back();
				if (e->status == EdgeStatus::active)
					return e;
				front.pop_back(); // cleanup non-active edges from front
			}
			return {};
		}

		struct PivotResult {
			MeshPoint* p;
			vec3 center;
		};

		auto ballPivot(const MeshEdge* e, Grid& grid, float radius) -> std::optional<PivotResult> {
			const auto m = (e->a->pos + e->b->pos) / 2.0f;
			const auto oldCenterVec = glm::normalize(e->center - m);
			auto neighborhood = grid.sphericalNeighborhood(m, {e->a->pos, e->b->pos, e->opposite->pos});

			static auto counter = 0;
			counter++;
			if (debug) {
				saveTriangles(std::to_string(counter) + "_pivot_edge.stl", std::vector<Triangle>{Triangle{e->a->pos, e->a->pos, e->b->pos}});

				std::vector<vec3> points(neighborhood.size());
				std::transform(begin(neighborhood), end(neighborhood), begin(points), [](const MeshPoint* p) { return p->pos; });
				savePoints(std::to_string(counter) + "_neighborhood.ply", points);
			}

			auto smallestAngle = std::numeric_limits<float>::max();
			MeshPoint* pointWithSmallestAngle = nullptr;
			vec3 centerOfSmallest{};
			std::stringstream ss;
			if (debug) ss << counter << ". pivoting edge a=" << e->a->pos << " b=" << e->b->pos << " op=" << e->opposite->pos << ". testing " << neighborhood.size() << " neighbors\n";
			auto i = 0;
			int smallestNumber = 0;
			for (const auto& p : neighborhood) {
				i++;
				auto newFaceNormal = Triangle{e->b->pos, e->a->pos, p->pos}.normal();

				// this check is not in the paper: all points' normals must point into the same half-space
				if (dot(newFaceNormal, p->normal) < 0)
					continue;

				const auto c = computeBallCenter(MeshFace{{e->b, e->a, p}}, radius);
				if (!c) {
					if (debug) ss << i << ".    " << p->pos << " center computation failed\n";
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
				const auto newCenterFaceDot = glm::dot(newCenterVec, newFaceNormal);
				if (newCenterFaceDot < 0) {
					if (debug) ss << i << ".    " << p->pos << " ball center " << c.value() << " underneath triangle\n";
					continue;
				}

				// this check is not in the paper: points to which we already have an inner edge are not considered
				for (const auto* ee : p->edges) {
					const auto* otherPoint = ee->a == p ? ee->b : ee->a;
					if (ee->status == EdgeStatus::inner && (otherPoint == e->a || otherPoint == e->b)) {
						if (debug) ss << i << ".    " << p->pos << " inner edge exists\n";
						goto nextneighbor;
					}
				}

				auto angle = std::acos(std::clamp(glm::dot(oldCenterVec, newCenterVec), -1.0f, 1.0f));
				if (glm::dot(glm::cross(newCenterVec, oldCenterVec), e->a->pos - e->b->pos) < 0)
					angle += pi;
				if (angle < smallestAngle) {
					smallestAngle = angle;
					pointWithSmallestAngle = p;
					centerOfSmallest = c.value();
					smallestNumber = i;
				}
				if (debug) ss << i << ".    " << p->pos << " center " << c.value() << " angle " << angle << " newCenterFaceDot " << newCenterFaceDot << "\n";
			nextneighbor:;
			}

			if (smallestAngle != std::numeric_limits<float>::max()) {
				if (ballIsEmpty(centerOfSmallest, neighborhood, radius)) {
					if (debug) {
						ss << "        picking point " << smallestNumber << "\n";
						savePoints(std::to_string(counter) + "_candidate.ply", {pointWithSmallestAngle->pos});
					}
					return PivotResult{pointWithSmallestAngle, centerOfSmallest};
				} else if(debug)
					ss << "        found candidate " << smallestNumber << " but ball is not empty\n";
			}
			if (debug) std::cout << ss.str();

			return {};
		}

		auto notUsed(const MeshPoint* p) -> bool {
			return !p->used;
		}

		auto onFront(const MeshPoint* p) -> bool {
			return std::any_of(begin(p->edges), end(p->edges), [&](const MeshEdge* e) {
				return e->status == EdgeStatus::active;
			});
		}

		void remove(MeshEdge* edge) {
			// just mark the edge as inner. The edge will be removed later in getActiveEdge()
			edge->status = EdgeStatus::inner;
		}

		void outputTriangle(MeshFace f, std::vector<Triangle>& triangles) {
			triangles.push_back({f[0]->pos, f[1]->pos, f[2]->pos});
		}

		auto join(MeshEdge* e_ij, MeshPoint* o_k, vec3 o_k_ballCenter, std::vector<MeshEdge*>& front, std::deque<MeshEdge>& edges) -> std::tuple<MeshEdge*, MeshEdge*> {
			auto& e_ik = edges.emplace_back(MeshEdge{e_ij->a, o_k, e_ij->b, o_k_ballCenter});
			auto& e_kj = edges.emplace_back(MeshEdge{o_k, e_ij->b, e_ij->a, o_k_ballCenter});

			e_ik.next = &e_kj;
			e_ik.prev = e_ij->prev;
			e_ij->prev->next = &e_ik;
			e_ij->a->edges.push_back(&e_ik);

			e_kj.prev = &e_ik;
			e_kj.next = e_ij->next;
			e_ij->next->prev = &e_kj;
			e_ij->b->edges.push_back(&e_kj);

			o_k->used = true;
			o_k->edges.push_back(&e_ik);
			o_k->edges.push_back(&e_kj);

			front.push_back(&e_ik);
			front.push_back(&e_kj);
			remove(e_ij);

			return {&e_ik, &e_kj};
		}

		void glue(MeshEdge* a, MeshEdge* b, std::vector<MeshEdge*>& front) {
			if (debug) {
				std::vector<Triangle> frontTriangles;
				for (const auto* e : front)
					if (e->status == EdgeStatus::active)
						frontTriangles.push_back(Triangle{e->a->pos, e->a->pos, e->b->pos});
				saveTriangles("glue_front.stl", frontTriangles);
				saveTriangles("glue_edges.stl", std::vector<Triangle>{Triangle{a->a->pos, a->a->pos, a->b->pos}, Triangle{b->a->pos, b->a->pos, b->b->pos}});
			}

			// case 1
			if (a->next == b && a->prev == b && b->next == a && b->prev == a) {
				remove(a);
				remove(b);
				return;
			}
			// case 2
			if (a->next == b && b->prev == a) {
				a->prev->next = b->next;
				b->next->prev = a->prev;
				remove(a);
				remove(b);
				return;
			}
			if (a->prev == b && b->next == a) {
				a->next->prev = b->prev;
				b->prev->next = a->next;
				remove(a);
				remove(b);
				return;
			}
			// case 3/4
			a->prev->next = b->next;
			b->next->prev = a->prev;
			a->next->prev = b->prev;
			b->prev->next = a->next;
			remove(a);
			remove(b);
		}

		auto findReverseEdgeOnFront(MeshEdge* edge) -> MeshEdge* {
			for (auto& e : edge->a->edges)
				if (e->a == edge->b)
					return e;
			return nullptr;
		}
	}

	auto reconstruct(const std::vector<Point>& points, float radius) -> std::vector<Triangle> {
		auto grid = buildGrid(points, radius);

		const auto seedResult = findSeedTriangle(grid, radius);
		if (!seedResult) {
			std::cerr << "No seed triangle found\n";
			return {};
		}

		std::vector<Triangle> triangles;
		std::deque<MeshEdge> edges;

		auto [seed, ballCenter] = *seedResult;
		outputTriangle(seed, triangles);
		auto& e0 = edges.emplace_back(MeshEdge{seed[0], seed[1], seed[2], ballCenter});
		auto& e1 = edges.emplace_back(MeshEdge{seed[1], seed[2], seed[0], ballCenter});
		auto& e2 = edges.emplace_back(MeshEdge{seed[2], seed[0], seed[1], ballCenter});
		e0.prev = e1.next = &e2;
		e0.next = e2.prev = &e1;
		e1.prev = e2.next = &e0;
		seed[0]->edges = { &e0, &e2 };
		seed[1]->edges = { &e0, &e1 };
		seed[2]->edges = { &e1, &e2 };
		std::vector<MeshEdge*> front{&e0, &e1, &e2};

		if (debug) saveTriangles("seed.stl", triangles);

		auto counter = 0;
		while (auto e_ij = getActiveEdge(front)) {
			if (debug) saveTriangles("current_active_edge.stl", std::vector<Triangle>{Triangle{e_ij.value()->a->pos, e_ij.value()->a->pos, e_ij.value()->b->pos}});
			const auto o_k = ballPivot(e_ij.value(), grid, radius);
			if (debug) saveTriangles("current_mesh.stl", triangles);
			if (o_k && (notUsed(o_k->p) || onFront(o_k->p))) {
				outputTriangle({{e_ij.value()->a, o_k->p, e_ij.value()->b}}, triangles);
				auto [e_ik, e_kj] = join(e_ij.value(), o_k->p, o_k->center, front, edges);
				if (auto* e_ki = findReverseEdgeOnFront(e_ik)) glue(e_ik, e_ki, front);
				if (auto* e_jk = findReverseEdgeOnFront(e_kj)) glue(e_kj, e_jk, front);
			} else {
				if (debug) savePoints("current_boundary_point.ply", {o_k->p->pos});
				e_ij.value()->status = EdgeStatus::boundary;
			}
		}

		if (debug) {
			std::vector<Triangle> boundaryEdges;
			for (const auto* e : front) {
				if (e->status == EdgeStatus::boundary)
					boundaryEdges.push_back({e->a->pos, e->a->pos, e->b->pos});
			}
			saveTriangles("boundaryEdges.stl", boundaryEdges);
		}

		return triangles;
	}
}
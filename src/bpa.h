#pragma once

#include <array>
#include <glm/glm.hpp>
#include <vector>

namespace bpa {
	struct Triangle : std::array<glm::vec3, 3> {
		auto normal() const { return glm::normalize(glm::cross((*this)[0] - (*this)[1], (*this)[0] - (*this)[2])); }
	};

	struct Point {
		glm::vec3 pos;
		glm::vec3 normal;
	};

	auto reconstruct(const std::vector<Point>& points, float radius) -> std::vector<Triangle>;
} // namespace bpa

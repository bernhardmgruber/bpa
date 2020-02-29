#pragma once

#include <array>
#include <vector>

#include <glm/vec3.hpp>

namespace bpa {
	using Triangle = std::array<glm::vec3, 3>;

	auto reconstruct(const std::vector<glm::vec3>& points)->std::vector<Triangle>;
}

#pragma once

#include <stdexcept>
#include <string>
#include <utility>

// Converts an integer to another integer type and throws std::range_error when the value does not fit.
// A static_cast would silently drop the bits that do not fit.
template <typename To, typename From>
To checked_cast(From value) {
	if (!std::in_range<To>(value))
		throw std::range_error("value " + std::to_string(value) + " does not fit the target type");
	return static_cast<To>(value);
}

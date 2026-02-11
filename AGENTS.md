# AGENTS.md - Repository Guidelines

This document provides guidelines for AI agents working on this Visual-Inertial Odometry (VIO) C++ codebase.

## Build Commands

```bash
# Create build directory and build
mkdir -p build && cd build
cmake ..
make -j$(nproc)

# Build the main executable
cd build && make vioodometry

# Clean build
rm -rf build && mkdir build && cd build && cmake .. && make
```

## Project Structure

- **src/**: Implementation files (.cpp)
- **include/**: Header files (.h)
- **config/**: YAML configuration files
- **CMakeLists.txt**: Build configuration

## Dependencies

- OpenCV (required)
- Ceres Solver (required)
- Sophus (required)
- Pangolin (required)
- Eigen3 (required)

## Code Style Guidelines

### Naming Conventions

- **Classes**: PascalCase (e.g., `VioOdometry`, `FeaturePoints`)
- **Functions**: camelCase (e.g., `setPose()`, `addFeaturePoints()`)
- **Member variables**: snake_case (e.g., `acc_bias`, `feature_points`)
- **Private members**: Use trailing underscore (e.g., `private_var_`) or prefix (project uses mixed style)
- **Constants**: UPPER_SNAKE_CASE (e.g., `MAX_CNT`, `NUMOFFRAMESTOINIT`)
- **Global variables**: Prefix with g_ or use UPPER_CASE in parameters.cpp

### Formatting

- **Indentation**: 4 spaces (no tabs)
- **Braces**: Opening brace on same line for functions and classes
- **Line length**: Keep under 120 characters
- **Spacing**: Space after keywords (if, for, while), spaces around operators

### Header Files

Always use both `#pragma once` AND traditional include guards:

```cpp
#pragma once
#ifndef CLASSNAME_H
#define CLASSNAME_H

// ... content ...

#endif // CLASSNAME_H
```

Include order in .cpp files:
1. Corresponding header
2. System headers
3. Library headers (OpenCV, Eigen, etc.)
4. Project headers

### Types and Math

- Use Eigen for matrix/vector operations (`Vector3d`, `Matrix3d`, `Quaterniond`)
- Use OpenCV types for image processing (`Mat`, `Point2f`)
- Maintain both Eigen and OpenCV representations where needed (see Frame class)
- Use `double` for precision in optimization contexts

### Error Handling

- Use `cerr` for error messages to stderr
- Check file openings before operations
- Use `assert` for debug checks where appropriate

### Code Patterns

When adding features, follow these patterns:

1. **Class Structure**: Public interface first, then member variables
2. **Constructors**: Provide default and parameterized constructors
3. **Pose Representation**: Store both rotation matrix and quaternion forms
4. **Feature Management**: Use `vector<pair<int, Point2f>>` for feature tracking

### Comments

- Use `//` for single-line comments
- Document complex algorithms briefly
- Prefer self-documenting code with clear naming

## Common Operations

### Adding a New Class

1. Create header in `include/`, implementation in `src/`
2. Update `CMakeLists.txt` to include new .cpp file
3. Follow existing naming and structure conventions

### Adding Dependencies

Update `CMakeLists.txt`:
```cmake
find_package(NewLib REQUIRED)
target_link_libraries(vioodometry ${NewLib_LIBRARIES})
```

## Testing

No formal test suite currently exists. Test by:
1. Building successfully: `make`
2. Running with sample data from config/
3. Verifying no runtime crashes with address sanitizer if available

## Notes

- Project uses C++11 standard
- Heavy use of `using namespace` directives (std, cv, Eigen) - follow existing pattern
- Be careful with Eigen alignment on 32-bit systems (use EIGEN_MAKE_ALIGNED_OPERATOR_NEW if adding classes with fixed-size Eigen members)
- Thread safety: Not currently designed for multi-threading

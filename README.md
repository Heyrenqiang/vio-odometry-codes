# Visual-Inertial Odometry (VIO)

A Visual-Inertial Odometry system implementing visual SfM, IMU pre-integration, and sliding window optimization using non-linear optimization methods.

## Overview

This project implements a complete VIO pipeline that fuses visual and inertial measurements to estimate camera pose and 3D structure. The system includes:

- **Visual Front-end**: Feature tracking using Lucas-Kanade optical flow with Shi-Tomasi corner detection
- **Visual Initialization**: Structure from Motion (SfM) using 2D-2D matching and PnP
- **IMU Pre-integration**: Mid-point integration with covariance propagation
- **Bundle Adjustment**: Ceres-based non-linear optimization for pose and structure refinement
- **Visual-Inertial Alignment**: Gyroscope bias calibration, gravity estimation, and metric scale recovery

## System Architecture

```
Input: Image + IMU Data
  ↓
Feature Tracking (LK Optical Flow)
  ↓
IMU Pre-integration
  ↓
Initialization (SfM + PnP)
  ↓
Bundle Adjustment (Ceres Solver)
  ↓
Visual-IMU Alignment
  ↓
Output: Camera Pose + 3D Map Points
```

## Dependencies

- **OpenCV** (≥3.0) - Computer vision and image processing
- **Ceres Solver** - Non-linear optimization library
- **Sophus** - Lie groups and algebra library
- **Pangolin** - Visualization library
- **Eigen3** - Linear algebra library

## Build Instructions

```bash
# Create build directory
mkdir -p build && cd build

# Configure with CMake
cmake ..

# Build with parallel jobs
make -j$(nproc)
```

## Usage

1. **Prepare Data**: 
   - Place image sequence in `data/cam0_1/data/`
   - Place timestamps in `data/cam0_1/times.txt`
   - Place IMU data in `data/imu0_1/data.csv`

2. **Configure Parameters**:
   - Edit `config/config.yaml` for camera calibration and VIO parameters

3. **Run**:
   ```bash
   ./build/vioodometry
   ```

## Configuration

Key parameters in `config/config.yaml`:

### Camera Calibration
- `image_width`, `image_height`: Image resolution
- `projection_parameters`: Camera intrinsics (fx, fy, cx, cy)
- `distortion_parameters`: Radial and tangential distortion (k1, k2, p1, p2)

### Feature Tracking
- `max_cnt`: Maximum number of features (default: 150)
- `min_dist`: Minimum distance between features (default: 30)
- `F_threshold`: RANSAC threshold for fundamental matrix (default: 1.0)

### Initialization
- `numofframestoinit`: Number of frames for initialization (default: 12)
- `startframe`: Frame to start initialization (default: 300)

### IMU Parameters
- `acc_n`, `gyr_n`: Accelerometer and gyroscope noise
- `acc_w`, `gyr_w`: Random walk noise for biases

## Code Structure

```
├── include/
│   ├── vioodometry.h      # Main VIO system interface
│   ├── integration_base.h # IMU pre-integration
│   ├── feature_track.h    # Feature tracking
│   ├── frame.h            # Frame data structure
│   ├── featurepoints.h    # Feature point management
│   └── parameters.h       # Global parameters
├── src/
│   ├── vioodometry.cpp    # VIO implementation
│   ├── feature_track.cpp  # Optical flow tracking
│   ├── frame.cpp          # Frame operations
│   ├── featurepoints.cpp  # Feature management
│   └── parameters.cpp     # Parameter loading
├── config/
│   └── config.yaml        # Configuration file
└── CMakeLists.txt         # Build configuration
```

## Key Algorithms

### 1. Feature Tracking
- **Detector**: Shi-Tomasi corner detection (`goodFeaturesToTrack`)
- **Tracker**: Lucas-Kanade optical flow with pyramid
- **Outlier Rejection**: Fundamental matrix estimation with RANSAC

### 2. IMU Pre-integration
- **Integration Method**: Mid-point integration for better accuracy
- **State Variables**: Position, rotation (quaternion), velocity, biases
- **Covariance**: Propagated using linearized error-state dynamics

### 3. Visual Initialization
- **Relative Pose**: Essential matrix decomposition with 2D-2D matching
- **Triangulation**: Linear triangulation using DLT
- **PnP**: Perspective-n-Point for frame pose estimation

### 4. Bundle Adjustment
- **Optimization Library**: Ceres Solver with Levenberg-Marquardt
- **Cost Function**: Reprojection error in normalized coordinates
- **Parameterization**: Quaternion local parameterization for rotation

### 5. Visual-IMU Alignment
- **Gyroscope Bias**: Minimizing visual-IMU rotation discrepancy
- **Gravity Direction**: Initial alignment and refinement
- **Metric Scale**: Recovering real-world scale from IMU integration

## Output

The system generates:
- **Trajectory**: `trajectory.txt` and `trajectoryafterba.txt` containing frame poses (quaternion + translation)
- **Visualization**: Real-time display using Pangolin

Trajectory format:
```
frame_id  qw  qx  qy  qz  tx  ty  tz
```

## Notes

- The system uses C++11 standard
- Both Eigen and OpenCV representations are maintained for compatibility
- Heavy use of `using namespace` directives follows existing codebase style
- Thread safety: Not currently designed for multi-threading

## References

This implementation is inspired by:
- VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator
- OKVIS: Open Keyframe-based Visual-Inertial SLAM
- SVO: Fast Semi-Direct Monocular Visual Odometry

## License

This project is for educational and research purposes.

## Author

Developed as a VIO learning and implementation project.

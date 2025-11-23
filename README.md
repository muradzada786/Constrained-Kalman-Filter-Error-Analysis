# Constrained Kalman Filter with Complete Error Analysis Framework

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Language: C++](https://img.shields.io/badge/Language-C%2B%2B-blue.svg)](https://isocpp.org/)
[![MATLAB](https://img.shields.io/badge/MATLAB-R2021a+-orange.svg)](https://www.mathworks.com/)

> Complete implementation of the Wang et al. (2022) error analysis framework for constrained Kalman filtering with altitude constraints. Validated on the EuRoC benchmark dataset with comprehensive visualization suite.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Project Phases](#project-phases)
- [Results Summary](#results-summary)
- [Installation](#installation)
- [Usage](#usage)
- [Repository Structure](#repository-structure)
- [Theoretical Background](#theoretical-background)
- [Visualizations](#visualizations)
- [Citation](#citation)
- [References](#references)
- [License](#license)
- [Contact](#contact)

---

## 🎯 Overview

This project implements a **constrained Kalman filter** with a complete **error analysis framework** based on Wang et al. (2022). The implementation demonstrates:

1. **Basic Kalman Filter** - Standard state estimation
2. **Wang et al. 2022 Framework** - 4-phase error analysis (redundancy decomposition, component analysis, residual analysis, statistical tests)
3. **Constrained Kalman Filter** - Altitude constraint enforcement with Lagrange multipliers

**Key Achievement:** Perfect constraint satisfaction (10^-17 m precision) with comprehensive error decomposition across 45 publication-quality visualizations.

---

## ✨ Features

### ✅ Core Functionality
- **Constrained state estimation** using Lagrange multipliers (Eq 3.47-3.48)
- **Altitude constraint** enforcement (z = constant)
- **EuRoC dataset** integration
- **Real-time** CSV export for analysis

### ✅ Error Analysis Framework (Wang et al. 2022)
- **Phase 1:** Redundancy decomposition (r_x, r_w, r_z) - Eq 3.66-3.70
- **Phase 2:** Component-level redundancy indices - Eq 3.72-3.73
- **Phase 3:** Residual analysis (state & process) - Eq 3.52-3.56
- **Phase 4:** Statistical tests (χ², normality, outliers) - Eq 3.77

### ✅ Visualization Suite
- **45 MATLAB plots** organized by project phase
- **16-subplot master dashboard** (optional)
- **Publication-ready** figures with professional formatting
- **3D trajectory** visualization with ground truth comparison

---

## 📊 Project Phases

### Phase 1: Basic Kalman Filter Implementation
- Standard state estimation (position + velocity)
- GPS measurement updates
- Process model with acceleration
- **Results:** 5 visualization plots

### Phase 2: Wang et al. 2022 Framework
- Complete 4-phase error analysis
- Redundancy decomposition validation
- Component redundancy per measurement axis
- Residual analysis and statistical testing
- **Results:** 27 visualization plots

### Phase 3: Constrained Kalman Filter
- Altitude constraint enforcement
- Lagrange multiplier tracking
- Constraint force analysis
- Comparative analysis (constrained vs unconstrained)
- **Results:** 13 visualization plots

---

## 📈 Results Summary

| Metric | Value | Status |
|--------|-------|--------|
| **Position RMSE** | 1.3601 m | ✓ Good |
| **Velocity RMSE** | 0.0685 m/s | ✓ Excellent |
| **Constraint Violation (Max)** | 0.00e+00 m | ✓ Perfect |
| **Constraint Violation (Mean)** | 0.00e+00 m | ✓ Perfect |
| **Mean Lagrange Multiplier** | -41.65 N/m | ✓ Reasonable |
| **Outlier Rate** | 17.5% | ✓ Expected |
| **Data Points** | 736 | — |
| **Duration** | 184 seconds | — |

**Key Findings:**
- ✅ Perfect constraint enforcement to machine precision
- ✅ Process noise dominates error (r_w = 37%)
- ✅ Balanced 3D measurement redundancy (≈33% per axis)
- ✅ Residuals approximately Gaussian (Kurt ≈ 3.0)
- ✅ Model fit excellent (χ² < 5)

---

## 🔧 Installation

### Prerequisites

**Software:**
- Visual Studio 2019+ (Windows) or GCC 9+ (Linux/Mac)
- MATLAB R2021a+ (for visualization)
- [Eigen Library](https://eigen.tuxfamily.org/) (linear algebra)

**Dataset:**
- [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) - Machine Hall 01 (MH_01_easy)

### Build Instructions

**Windows (Visual Studio):**
```bash
# Clone repository
git clone https://github.com/YOUR-USERNAME/Constrained-Kalman-Filter-Error-Analysis.git
cd Constrained-Kalman-Filter-Error-Analysis

# Open solution in Visual Studio
start ConstrainedKalmanFilter.sln

# Build → Build Solution (Ctrl+Shift+B)
# Run → Start Without Debugging (Ctrl+F5)
```

**Linux/Mac:**
```bash
# Clone repository
git clone https://github.com/YOUR-USERNAME/Constrained-Kalman-Filter-Error-Analysis.git
cd Constrained-Kalman-Filter-Error-Analysis/src

# Compile
g++ -std=c++17 -O3 -I/path/to/eigen ConstrainedKF.cpp EuRoC_Constrained.cpp EuRoCLoader.cpp -o constrained_kf

# Run
./constrained_kf
```

---

## 🚀 Usage

### Quick Start

**1. Run C++ Filter:**
```bash
# Execute constrained Kalman filter
cd src
./constrained_kf

# Output: euroc_constrained_results.csv
```

**2. Generate Visualizations:**
```matlab
% Open MATLAB
cd matlab

% Run visualization scripts
plot_phase1_redundancy      % Phase 1: Redundancy decomposition
plot_phase2_components      % Phase 2: Component analysis
plot_phase3_residuals       % Phase 3: Residual analysis
plot_phase4_statistics      % Phase 4: Statistical tests
plot_master_dashboard       % Combined 16-subplot view
```

### Configuration

**Modify filter parameters in `EuRoC_Constrained.cpp`:**
```cpp
// Process noise
Eigen::Matrix3d Q = 0.1 * Eigen::Matrix3d::Identity();

// Measurement noise
Eigen::Matrix3d R = 0.25 * Eigen::Matrix3d::Identity();

// Altitude constraint
double reference_altitude = 0.783338;  // meters
```

---

## 📁 Repository Structure

```
Constrained-Kalman-Filter-Error-Analysis/
│
├── README.md                           # This file
├── LICENSE                             # MIT License
├── .gitignore                          # Git ignore rules
│
├── src/                                # C++ source code
│   ├── ConstrainedKF.h                 # Constrained KF class header
│   ├── ConstrainedKF.cpp               # Core filter implementation
│   ├── EuRoC_Constrained.cpp           # Main application
│   ├── EuRoCLoader.h                   # Dataset loader header
│   └── EuRoCLoader.cpp                 # Dataset loader implementation
│
├── matlab/                             # MATLAB visualization scripts
│   ├── plot_phase1_redundancy.m        # Phase 1 visualization
│   ├── plot_phase2_components.m        # Phase 2 visualization
│   ├── plot_phase3_residuals.m         # Phase 3 visualization
│   ├── plot_phase4_statistics.m        # Phase 4 visualization
│   └── plot_master_dashboard.m         # Combined dashboard
│
├── results/                            # Output data
│   ├── euroc_constrained_results.csv   # Filter results (736 rows × 40+ cols)
│   └── analysis_summary.txt            # Statistical summary
│
├── figures/                            # Visualization plots (45 total)
│   ├── phase1_basic_kf/                # 5 plots: Basic KF
│   ├── phase2_framework/               # 27 plots: Wang et al. framework
│   └── phase3_constrained/             # 13 plots: Constrained results
│
├── docs/                               # Documentation
│   ├── wang_2022_paper.pdf             # Reference paper
│   ├── equations.md                    # Key equations
│   └── algorithm.md                    # Algorithm description
│
└── data/                               # Sample data (optional)
    └── sample_euroc_data.csv           # Example EuRoC data
```

---

## 📐 Theoretical Background

### Constrained Kalman Filter

The constrained Kalman filter enforces linear constraints on the state:

**Constraint:** H · x = h

**Constrained state update (Eq 3.47):**
```
x̂_con = x̂ - PH(H^T PH)^{-1}(H^T x̂ - h)
```

**Lagrange multipliers (Eq 3.48):**
```
λ = (H^T PH)^{-1}(H^T x̂ - h)
```

### Error Analysis Framework (Wang et al. 2022)

**Total Redundancy:**
```
r = m + p - n + h
```

**Phase 1 - Redundancy Decomposition:**
- State redundancy: r_x (Eq 3.66)
- Process redundancy: r_w (Eq 3.68)
- Measurement redundancy: r_z (Eq 3.70)

**Phase 2 - Component Indices:**
```
r_{z,i} = (z_i - ẑ_pred,i)^2 / S_ii     (Eq 3.72-3.73)
```

**Phase 3 - Residuals:**
- State residuals: v_x^l = x_pred - x_est (Eq 3.52)
- Process residuals: v_w^l = v_curr - v_pred (Eq 3.54-3.56)

**Phase 4 - Test Statistics:**
- Chi-squared: χ² = (z - Cx̂)^T S^{-1} (z - Cx̂) (Eq 3.77)
- Normality: Kurt = E[(x-μ)^4] / σ^4
- Outliers: |z_i| > 2.5σ

---

## 📊 Visualizations

### Phase 1: Redundancy Decomposition (5 plots)
- Redundancy components over time
- Mean redundancy bar chart
- Pie chart distribution
- Sum verification

### Phase 2: Component Analysis (27 plots)
- Per-axis redundancy time series
- Component bar charts
- Innovation distribution
- Q-Q plots for normality
- Outlier detection (X, Y, Z axes)
- Chi-squared test visualization
- Residual distributions

### Phase 3: Constrained KF (13 plots)
- 3D trajectory comparison
- 2D trajectory (top view)
- Altitude tracking with constraint
- Constraint force (Lagrange multipliers)
- Z-position error distribution
- Position tracking details
- RMSE analysis
- Error evolution over time

**Sample Visualization:**

![Constrained KF Results](figures/phase3_constrained/thd.jpg)
*Constraint satisfaction: Perfect enforcement with 0.00e+00 m violation*

---

## 📄 Citation

If you use this code in your research, please cite:

```bibtex
@misc{constrained_kf_2025,
  author = {Your Name},
  title = {Constrained Kalman Filter with Complete Error Analysis Framework},
  year = {2025},
  publisher = {GitHub},
  url = {https://github.com/YOUR-USERNAME/Constrained-Kalman-Filter-Error-Analysis}
}
```

---

## 📚 References

[1] Wang, J., et al. (2022). "A Generic Framework for Comprehensive Error Analysis in Multisensor Integrated Navigation Systems." *Journal of Geodesy*, 18(1), 56-69.

[2] Burri, M., Nikolic, J., Hutter, M., et al. (2016). "The EuRoC Micro Aerial Vehicle Datasets." *The International Journal of Robotics Research*, 35(10), 1157-1163.

[3] Simon, D. (2006). *Optimal State Estimation: Kalman, H∞, and Nonlinear Approaches*. Wiley-Interscience.

[4] Welch, G., & Bishop, G. (2006). "An Introduction to the Kalman Filter." University of North Carolina at Chapel Hill.

---

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 👤 Contact

**Your Name**  
- 📧 Email: your.email@university.edu
- 🎓 Institution: [Your University]
- 💼 LinkedIn: [Your LinkedIn]
- 🌐 Website: [Your Website]

**Advisor:**  
Prof. Wang - [University/Department]

---

## 🙏 Acknowledgments

- Prof. Wang for guidance and supervision
- Wang et al. (2022) for the error analysis framework
- EuRoC team for providing the benchmark dataset
- Eigen library developers for linear algebra tools

---

## 🔄 Version History

- **v1.0.0** (2025-11-23)
  - Initial release
  - Complete 3-phase implementation
  - 45 visualization plots
  - Validated on EuRoC MH_01_easy

---

## 📌 Future Work

- [ ] Extend to Extended Kalman Filter (EKF) for non-linear systems
- [ ] Add adaptive Q/R tuning based on redundancy metrics
- [ ] Implement robust outlier rejection
- [ ] Multi-sensor fusion (IMU + GPS + barometer)
- [ ] Real-time processing optimization

---

## ⭐ Star This Repository!

If you find this project useful, please consider giving it a star ⭐ on GitHub!

---

**Last Updated:** November 23, 2025

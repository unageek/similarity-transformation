# similarity-transformation

Given two lists of points $`(𝐩_i)_{i=1}^N`$ and $`(𝐪_i)_{i=1}^N`$, `SimilarityTransformation::estimate` finds the common similarity transformation (rotation, translation, and scaling) that maps each $𝐩_i$ to $𝐪_i$.

## Usage

This library consists of a single header file [`similarity_transformation.h`](similarity_transformation.h).

The following dependencies are required:

- [Eigen](https://eigen.tuxfamily.org/)
- [artivis/manif](https://github.com/artivis/manif)
- [TartanLlama/optional](https://github.com/TartanLlama/optional/tree/master)

See [`tests/test.cc`](tests/test.cc) for usage.

## Implementation

The Gauss–Newton method is used to minimize the sum of squared errors $`∑_{i=1}^N ‖𝐪_i - f(𝐩_i)‖^2`$.

## References

1. C. Hertzberg, R. Wagner, U. Frese, and L. Schröder, Integrating Generic Sensor Fusion Algorithms with Sound State Representations through Encapsulation of Manifolds. https://doi.org/10.48550/arXiv.1107.1119

2. J. Solà, J. Deray, and D. Atchuthan, A micro Lie theory for state estimation in robotics. https://doi.org/10.48550/arXiv.1812.01537

#pragma once

namespace boids {

struct Config {
    float maxVelocity         = 2.0f;
    float neighbourhoodRadius = 100.0f;

    float alignmentScale = 0.1;
    float coheasionScale = 0.05;
    float repelScale     = 0.1;
};
} // namespace boids
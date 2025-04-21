#pragma once

namespace boids {

struct Config {
    float maxVelocity         = 2.0f;
    float neighbourhoodRadius = 100.0f;

    float alignmentScale     = 0.1f;
    float coheasionScale     = 0.05f;
    float repelScale         = 0.1f;
    float obstacleRepelScale = 2.0f;
    float predatorRepelScale = 5.0f;

    float repelMinDist = 50.0f;
};
} // namespace boids

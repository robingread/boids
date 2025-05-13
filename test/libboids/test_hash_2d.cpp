#include "hash_2d.h"
#include <catch2/catch.hpp>

namespace boids {

TEST_CASE("Test the Hash2D class") {
    GIVEN("A single boid") {
        const double          cell_size = 0.5;
        const QRectF          bounds(0.0, 0.0, 1.0, 1.0);
        Hash2D                hash(cell_size, bounds);
        const uint16_t        id = 0;
        const Eigen::Vector2d position(0.1, 0.1);

        WHEN("Querying and updating the bounds") {
            REQUIRE(hash.getBounds() == bounds);
            WHEN("Setting new bounds") {
                const QRectF new_bounds(0.0, 0.0, 2.0, 2.0);
                hash.setBounds(new_bounds);
                REQUIRE(hash.getBounds() == new_bounds);
            }
        }

        WHEN("The boid is added to the hash") {
            hash.update(id, position);

            THEN("The boid should be in the hash") {
                const auto result = hash.query(position, cell_size);
                REQUIRE(result.size() == 1);
                REQUIRE(result.contains(id));
            }

            WHEN("Querying the boid position") {
                const auto result = hash.queryPositionById(id);
                THEN("The position should be the same as the one added") {
                    REQUIRE(result.x() == Approx(position.x()));
                    REQUIRE(result.y() == Approx(position.y()));
                }
            }

            WHEN("Querying a position outside of the same cell") {
                const double          radius = 0.1;
                const Eigen::Vector2d query_position(9.0, 9.0);
                const auto            result = hash.query(query_position, radius);
                THEN("The result should be empty") { REQUIRE(result.size() == 0); }
            }

            WHEN("Querying a position within the same cell, but outside the radius") {
                const double          radius = 0.1;
                const Eigen::Vector2d query_position(0.25, 0.25);
                const auto            result = hash.query(query_position, radius);
                THEN("The boid should not be in the result") { REQUIRE(result.empty()); }
            }

            WHEN("Updating the boid position") {
                const Eigen::Vector2d new_position(0.9, 0.9);
                hash.update(id, new_position);

                WHEN("Querying the new position") {
                    const auto result = hash.query(new_position, 0.1);
                    THEN("The boid should be in the hash") {
                        REQUIRE(result.size() == 1);
                        REQUIRE(result.contains(id));
                    }
                }

                WHEN("Querying the old position") {
                    const auto result = hash.query(position, 0.1);
                    THEN("The boid should not be in the hash") { REQUIRE(result.size() == 0); }
                }

                WHEN("Getting the position by ID") {
                    const auto result = hash.queryPositionById(id);
                    THEN("The position should be the new position") {
                        REQUIRE(result.x() == Approx(new_position.x()));
                        REQUIRE(result.y() == Approx(new_position.y()));
                    }
                }
            }

            WHEN("Clearing the hash") {
                hash.clear();
                THEN("The boid should be removed from the hash") {
                    const auto result = hash.query(position, cell_size);
                    REQUIRE(result.size() == 0);
                }
                THEN("It should not be possible to query a position by ID") {
                    REQUIRE_THROWS(hash.queryPositionById(0));
                }
            }
        }
    }
}

SCENARIO("Three boids, two close and one far") {
    const uint16_t        id1 = 0;
    const Eigen::Vector2d position1(0.1, 0.1);

    const uint16_t        id2 = 1;
    const Eigen::Vector2d position2(0.2, 0.1);

    const uint16_t        id3 = 2;
    const Eigen::Vector2d position3(0.1, 0.95);

    GIVEN("A Hash with a cell size of 0.1") {

        const double cell_size = 0.1;
        const QRectF bounds(0.0, 0.0, 1.0, 1.0);

        // Add the boids to the hash
        Hash2D hash(cell_size, bounds);
        hash.update(id1, position1);
        hash.update(id2, position2);
        hash.update(id3, position3);

        WHEN("Searching around boid1 with a radius of 0.15") {
            const double radius = 0.15;
            const auto   result = hash.query(position1, radius);

            THEN("The boids should be in the hash") {
                REQUIRE(result.contains(id1));
                REQUIRE(result.contains(id2));
            }
        }

        WHEN("Searching around boid3 with a radius of 0.16") {
            const double radius = 0.16;
            const auto   result = hash.query(position3, radius);

            THEN("The boids should be in the hash") {
                REQUIRE(result.contains(id1));
                REQUIRE(result.contains(id3));
            }
        }

        WHEN("Searching around boid3 with a radius of 0.2") {
            const double radius = 0.2;
            const auto   result = hash.query(position3, radius);

            THEN("The boids should be in the hash") {
                REQUIRE(result.contains(id1));
                REQUIRE(result.contains(id2));
                REQUIRE(result.contains(id3));
            }
        }
    }
}

TEST_CASE("Test the Hash2D::getCellPosition() method") {
    GIVEN("A cell size of 1.0") {
        const double cell_size = 1.0;

        WHEN("The Position is (2.5, 3.5)") {
            const Eigen::Vector2d position(2.5, 3.5);
            const Eigen::Vector2d expected(2.5, 3.5);
            const auto            result = Hash2D::getCellPosition(position, cell_size);
            REQUIRE(result.isApprox(expected));
        }
        WHEN("The Position is (2.95, 3.95)") {
            const Eigen::Vector2d position(2.95, 3.95);
            const Eigen::Vector2d expected(2.5, 3.5);
            const auto            result = Hash2D::getCellPosition(position, cell_size);
            REQUIRE(result.isApprox(expected));
        }
        WHEN("The Position is (2.05, 3.05)") {
            const Eigen::Vector2d position(2.05, 3.95);
            const Eigen::Vector2d expected(2.5, 3.5);
            const auto            result = Hash2D::getCellPosition(position, cell_size);
            THEN("The result should be (2.5, 3.5)") {
                REQUIRE(result.x() == Approx(expected.x()));
                REQUIRE(result.y() == Approx(expected.y()));
            }
        }
        WHEN("The Position is (1.0), 1.0)") {
            const Eigen::Vector2d position(1.0, 1.0);
            const Eigen::Vector2d expected(1.5, 1.5);
            const auto            result = Hash2D::getCellPosition(position, cell_size);
            THEN("The result should be (1.5, 1.5)") {
                REQUIRE(result.x() == Approx(expected.x()));
                REQUIRE(result.y() == Approx(expected.y()));
            }
        }
        WHEN("The position is (-1.5, -2.5)") {
            const Eigen::Vector2d position(-1.5, -2.5);
            const Eigen::Vector2d expected(-1.5, -2.5);
            const auto            result = Hash2D::getCellPosition(position, cell_size);
            REQUIRE(result.isApprox(expected));
        }
    } // namespace boids
    GIVEN("A cell size of 0.5") {
        const double cell_size = 0.5;
        WHEN("The position is (2.05, 3.05)") {
            const Eigen::Vector2d position(2.05, 3.05);
            const Eigen::Vector2d expected(2.25, 3.25);
            const auto            result = Hash2D::getCellPosition(position, cell_size);
            THEN("The result should be (2.25, 3.25)") {
                REQUIRE(result.x() == Approx(expected.x()));
                REQUIRE(result.y() == Approx(expected.y()));
            }
        }
        WHEN("The position is (2.5, 3.5)") {
            const Eigen::Vector2d position(2.5, 3.5);
            const Eigen::Vector2d expected(2.75, 3.75);
            const auto            result = Hash2D::getCellPosition(position, cell_size);
            THEN("The result should be (2.75, 3.75)") {
                REQUIRE(result.x() == Approx(expected.x()));
                REQUIRE(result.y() == Approx(expected.y()));
            }
        }
        WHEN("The position is (2.95, 3.95)") {
            const Eigen::Vector2d position(2.95, 3.95);
            const Eigen::Vector2d expected(2.75, 3.75);
            const auto            result = Hash2D::getCellPosition(position, cell_size);
            THEN("The result should be (2.75, 3.75)") {
                REQUIRE(result.x() == Approx(expected.x()));
                REQUIRE(result.y() == Approx(expected.y()));
            }
        }
        WHEN("The position is (-1.5, -2.5)") {
            const Eigen::Vector2d position(-1.5, -2.5);
            const Eigen::Vector2d expected(-1.25, -2.25);
            const auto            result = Hash2D::getCellPosition(position, cell_size);
            THEN("The result should be (-1.25, -2.25)") {
                REQUIRE(result.x() == Approx(expected.x()));
                REQUIRE(result.y() == Approx(expected.y()));
            }
        }
    }
}

TEST_CASE("Test the Hash2D::getPositionHash() method") {
    GIVEN("A Hash2D with a cell size of 0.1") {
        const double cell_size = 0.1;
        const QRectF bounds(0.0, 0.0, 1.0, 1.0);
        Hash2D       hash(cell_size, bounds);

        WHEN("Getting the hash for two points that are very close") {
            const Eigen::Vector2d p1(0.04, 0.04);
            const Eigen::Vector2d p2(0.05, 0.05);
            const std::size_t     h1 = hash.getPositionHash(p1, cell_size);
            const std::size_t     h2 = hash.getPositionHash(p2, cell_size);

            THEN("The hash values should be the same") { REQUIRE(h1 == h2); }
        }

        WHEN("Getting the hash for two points that are far apart") {
            const Eigen::Vector2d p1(0.1, 0.1);
            const Eigen::Vector2d p2(0.9, 0.9);
            const std::size_t     h1 = hash.getPositionHash(p1, cell_size);
            const std::size_t     h2 = hash.getPositionHash(p2, cell_size);

            THEN("The hash values should NOT be the same") { REQUIRE(h1 != h2); }
        }
    }
}
} // namespace boids

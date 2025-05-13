#pragma once

#include <Eigen/Core>
#include <QRect>
#include <map>
#include <set>
#include <vector>

namespace boids {

// TODO: Add a docstring
class Hash2DBase {
  public:
    ~Hash2DBase() = default;

    // TODO: Add a docstring
    virtual void clear() = 0;

    // TODO: Add a docstring
    virtual void update(const uint16_t& id, const Eigen::Vector2d& position) = 0;

    // TODO: Add a docstring
    virtual std::set<uint16_t> query(const Eigen::Vector2d& position, const double radius) = 0;
};

// TODO: Add a docstring
// TODO: This class needs to be able to wrap around the space when selecting neighbouring cells
class Hash2D : public Hash2DBase {
  public:
    /**
     * @brief Structure used to provide a mechanism to hash Eigen::Vector2d objects.
     */
    struct Hash {
        std::size_t operator()(const Eigen::Vector2d& position) const {
            const auto h1 = std::hash<double>()(position.x());
            const auto h2 = std::hash<double>()(position.y());

            // Hash combine (based on Boost's implementation)
            std::size_t seed = h1;
            seed ^= h2 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };

    Hash2D(const double cell_size, const QRectF& bounds);

    /**
     * @brief Clear all the hash entries.
     */
    void clear() override;

    /**
     * @brief Get the space/scene bounds that are used to wrap the space.
     * @return Bounds of the space.
     */
    QRectF getBounds() const;

    /**
     * @brief Set/update the bounds object.
     * @param bounds Bounds value to set
     */
    void setBounds(const QRectF& bounds);

    void update(const uint16_t& id, const Eigen::Vector2d& position) override;

    std::set<uint16_t> query(const Eigen::Vector2d& position, const double radius) override;

    Eigen::Vector2d queryPositionById(const uint16_t id) const;

    static Eigen::Vector2d getCellPosition(const Eigen::Vector2d& position, const double cell_size);

    static std::size_t getPositionHash(const Eigen::Vector2d& position, const double cell_size);

  private:
    double cell_size_;
    QRectF bounds_;

    using HashTable = std::unordered_map<std::size_t, std::vector<uint16_t>>;
    HashTable                           hash_table_;
    std::map<uint16_t, Eigen::Vector2d> id_to_position_map_;
};

} // namespace boids

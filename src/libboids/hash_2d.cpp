#include "hash_2d.h"
#include "utils.h"

namespace boids {

Hash2D::Hash2D(double cell_size, const QRectF& bounds) : cell_size_(cell_size), bounds_(bounds) {}

void Hash2D::clear() {
    hash_table_.clear();
    id_to_position_map_.clear();
}

QRectF Hash2D::getBounds() const { return bounds_; }

void Hash2D::setBounds(const QRectF& bounds) { bounds_ = bounds; }

void Hash2D::update(const uint16_t& id, const Eigen::Vector2d& position) {
    // If this ID doesn't already exist, add it to the hash table
    if (id_to_position_map_.find(id) == id_to_position_map_.end()) {
        const auto cell_position = getCellPosition(position, cell_size_);
        const auto hash_key      = Hash()(cell_position);
        id_to_position_map_[id]  = position;
        hash_table_[hash_key].push_back(id);
        return;
    }

    // Calculate the old cell position
    const auto old_cell_position = getCellPosition(id_to_position_map_[id], cell_size_);
    const auto old_hash_key      = Hash()(old_cell_position);

    // Calculate the new cell position
    const auto new_cell_position = getCellPosition(position, cell_size_);
    const auto new_hash_key      = Hash()(new_cell_position);

    // If the positions are different, remove the ID from the old cell and add it to the new cell
    if (old_hash_key != new_hash_key) {
        auto it = hash_table_.find(old_hash_key);
        if (it != hash_table_.end()) {
            auto& ids = it->second;
            ids.erase(std::remove(ids.begin(), ids.end(), id), ids.end());
            if (ids.empty()) {
                hash_table_.erase(it);
            }
        }
    }

    id_to_position_map_[id] = position;
    hash_table_[new_hash_key].push_back(id);
};

std::set<uint16_t> Hash2D::query(const Eigen::Vector2d& position, const double radius) {

    std::set<uint16_t> result;

    const Eigen::Vector2d min_cell =
        getCellPosition(position - Eigen::Vector2d::Constant(radius), cell_size_);

    const Eigen::Vector2d max_cell =
        getCellPosition(position + Eigen::Vector2d::Constant(radius), cell_size_);

    for (double x = min_cell.x(); x <= max_cell.x(); x += cell_size_) {
        for (double y = min_cell.y(); y <= max_cell.y(); y += cell_size_) {

            // Get the wrapped position of the cell
            const auto cell_xy   = utils::wrapVector2d(Eigen::Vector2d(x, y), bounds_);
            const auto cell_hash = getPositionHash(cell_xy, cell_size_);

            // check that the cell position is in the hash
            if (hash_table_.find(cell_hash) == hash_table_.end()) {
                continue;
            }

            // for each boid in the cell, check that it is within the radius.
            const auto& ids = hash_table_[cell_hash];

            // Check that the IDs in a cell are within the radius
            for (const auto& id : ids) {
                const auto& pos = id_to_position_map_[id];

                const auto distance = utils::vectorBetweenPoints(position, pos, bounds_).norm();

                if (distance <= radius) {
                    result.insert(id);
                }
            }
        }
    }

    return result;
}

Eigen::Vector2d Hash2D::queryPositionById(const uint16_t id) const {
    const auto it = id_to_position_map_.find(id);
    if (it != id_to_position_map_.end()) {
        return it->second;
    }
    throw std::runtime_error("ID not found the hash table");
}

Eigen::Vector2d Hash2D::getCellPosition(const Eigen::Vector2d& position, const double cell_size) {
    const auto cell_x = (std::floor(position.x() / cell_size) * cell_size) + (0.5 * cell_size);
    const auto cell_y = (std::floor(position.y() / cell_size) * cell_size) + (0.5 * cell_size);
    const auto cell_position = Eigen::Vector2d(cell_x, cell_y);
    return cell_position;
}

std::size_t Hash2D::getPositionHash(const Eigen::Vector2d& position, const double cell_size) {
    const auto cell_xy = getCellPosition(position, cell_size);
    const auto hash    = Hash()(cell_xy);
    return hash;
}

} // namespace boids

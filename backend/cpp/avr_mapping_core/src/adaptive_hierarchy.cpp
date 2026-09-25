#include "avr_mapping/adaptive_hierarchy.hpp"

#include <algorithm>
#include <cassert>

namespace avr::mapping {

namespace {

constexpr size_t safeMul(size_t a, size_t b) noexcept {
  if (a == 0 || b == 0) {
    return 0;
  }
  if (a > std::numeric_limits<size_t>::max() / b) {
    return std::numeric_limits<size_t>::max();
  }
  return a * b;
}

}  // namespace

AdaptiveHierarchy::AdaptiveHierarchy(size_t pool_block_capacity)
    : tile_pool_(pool_block_capacity) {}

AdaptiveHierarchy::~AdaptiveHierarchy() {
  reset();
}

std::optional<avr::spatial::CellKey> AdaptiveHierarchy::ensureRootForBase(
    const avr::spatial::GridCoord2D& base) {
  const auto root_key_opt = rootKeyForBase(base);
  if (!root_key_opt.has_value()) {
    return std::nullopt;
  }
  const auto root_key = *root_key_opt;
  const auto state = nodeState(root_key);
  if (state != NodeState::ABSENT) {
    return root_key;
  }

  const auto tile_key_opt = avr::spatial::TileKey::fromCellKey(root_key);
  if (!tile_key_opt.has_value()) {
    return std::nullopt;
  }
  const auto tile_key = *tile_key_opt;

  auto it = tiles_.find(tile_key);
  if (it == tiles_.end()) {
    AdaptiveTile* tile = tile_pool_.acquireTile();
    it = tiles_.emplace(tile_key, tile).first;
  }

  const auto coord = root_key.decode();
  const uint16_t local_idx = avr::spatial::localCellIndex(coord.x, coord.y);
  it->second->setLeaf(local_idx);

  ++node_count_;
  ++leaf_count_;
  ++leaf_count_by_level_[avr::spatial::L3];

  return root_key;
}

bool AdaptiveHierarchy::hasRootForBase(const avr::spatial::GridCoord2D& base) const noexcept {
  const auto root_key = rootKeyForBase(base);
  if (!root_key.has_value()) {
    return false;
  }
  return nodeState(*root_key) != NodeState::ABSENT;
}

std::optional<avr::spatial::CellKey> AdaptiveHierarchy::rootKeyForBase(
    const avr::spatial::GridCoord2D& base) noexcept {
  return avr::spatial::CellKey::fromGridCoord(base, avr::spatial::L3);
}

std::optional<avr::spatial::CellKey> AdaptiveHierarchy::findLeafContainingBase(
    const avr::spatial::GridCoord2D& base) const noexcept {
  const auto root_key_opt = rootKeyForBase(base);
  if (!root_key_opt.has_value()) {
    return std::nullopt;
  }
  const auto root_key = *root_key_opt;
  auto state = nodeState(root_key);
  if (state == NodeState::ABSENT) {
    return std::nullopt;
  }
  if (state == NodeState::LEAF) {
    return root_key;
  }

  // L3 is INTERNAL: descend level-by-level
  for (int lvl = static_cast<int>(avr::spatial::L2); lvl >= static_cast<int>(avr::spatial::L0); --lvl) {
    const auto key_opt = avr::spatial::CellKey::fromGridCoord(base, static_cast<uint8_t>(lvl));
    if (!key_opt.has_value()) {
      return std::nullopt;
    }
    const auto key = *key_opt;
    state = nodeState(key);
    if (state == NodeState::LEAF) {
      return key;
    }
    if (state == NodeState::ABSENT) {
      return std::nullopt;
    }
  }

  return std::nullopt;
}

NodeState AdaptiveHierarchy::nodeState(const avr::spatial::CellKey& key) const noexcept {
  if (!avr::spatial::isValidLevel(key.level)) {
    return NodeState::ABSENT;
  }
  const auto tile_key_opt = avr::spatial::TileKey::fromCellKey(key);
  if (!tile_key_opt.has_value()) {
    return NodeState::ABSENT;
  }
  const auto* tile = findTile(*tile_key_opt);
  if (tile == nullptr) {
    return NodeState::ABSENT;
  }
  const auto coord = key.decode();
  const uint16_t local_idx = avr::spatial::localCellIndex(coord.x, coord.y);
  return tile->nodeState(local_idx);
}

bool AdaptiveHierarchy::splitLeaf(const avr::spatial::CellKey& parent_key) {
  if (!avr::spatial::isValidLevel(parent_key.level) || parent_key.level <= avr::spatial::L0) {
    return false;
  }
  const auto parent_tile_key_opt = avr::spatial::TileKey::fromCellKey(parent_key);
  if (!parent_tile_key_opt.has_value()) {
    return false;
  }
  auto* parent_tile = findTile(*parent_tile_key_opt);
  if (parent_tile == nullptr) {
    return false;
  }
  const auto pcoord = parent_key.decode();
  const uint16_t plocal = avr::spatial::localCellIndex(pcoord.x, pcoord.y);
  if (parent_tile->nodeState(plocal) != NodeState::LEAF) {
    return false;
  }

  const auto children_opt = avr::spatial::childrenOf(parent_key);
  if (!children_opt.has_value()) {
    return false;
  }
  const auto& children = *children_opt;

  // Transaction check: reject if any child node already exists
  for (const auto& child : children) {
    if (nodeState(child) != NodeState::ABSENT) {
      return false;
    }
  }

  // Mutate parent: LEAF -> INTERNAL
  parent_tile->setInternal(plocal);
  --leaf_count_;
  ++internal_count_;
  --leaf_count_by_level_[parent_key.level];
  ++internal_count_by_level_[parent_key.level];

  // Insert four children as LEAF
  const uint8_t child_level = static_cast<uint8_t>(parent_key.level - 1);
  for (const auto& child : children) {
    const auto ctile_key = *avr::spatial::TileKey::fromCellKey(child);
    auto it = tiles_.find(ctile_key);
    if (it == tiles_.end()) {
      AdaptiveTile* ctile = tile_pool_.acquireTile();
      it = tiles_.emplace(ctile_key, ctile).first;
    }
    const auto ccoord = child.decode();
    const uint16_t clocal = avr::spatial::localCellIndex(ccoord.x, ccoord.y);
    it->second->setLeaf(clocal);
  }

  node_count_ += 4;
  leaf_count_ += 4;
  leaf_count_by_level_[child_level] += 4;

  return true;
}

bool AdaptiveHierarchy::mergeChildren(const avr::spatial::CellKey& parent_key) {
  if (!avr::spatial::isValidLevel(parent_key.level) || parent_key.level <= avr::spatial::L0) {
    return false;
  }
  const auto parent_tile_key_opt = avr::spatial::TileKey::fromCellKey(parent_key);
  if (!parent_tile_key_opt.has_value()) {
    return false;
  }
  auto* parent_tile = findTile(*parent_tile_key_opt);
  if (parent_tile == nullptr) {
    return false;
  }
  const auto pcoord = parent_key.decode();
  const uint16_t plocal = avr::spatial::localCellIndex(pcoord.x, pcoord.y);
  if (parent_tile->nodeState(plocal) != NodeState::INTERNAL) {
    return false;
  }

  const auto children_opt = avr::spatial::childrenOf(parent_key);
  if (!children_opt.has_value()) {
    return false;
  }
  const auto& children = *children_opt;

  // Non-recursive merge requirement: all four direct children must exist and be LEAF
  for (const auto& child : children) {
    if (nodeState(child) != NodeState::LEAF) {
      return false;
    }
  }

  // Remove four children
  const uint8_t child_level = static_cast<uint8_t>(parent_key.level - 1);
  for (const auto& child : children) {
    const auto ctile_key = *avr::spatial::TileKey::fromCellKey(child);
    auto it = tiles_.find(ctile_key);
    assert(it != tiles_.end());
    const auto ccoord = child.decode();
    const uint16_t clocal = avr::spatial::localCellIndex(ccoord.x, ccoord.y);
    it->second->eraseNode(clocal);
    if (it->second->empty()) {
      AdaptiveTile* freed = it->second;
      tiles_.erase(it);
      tile_pool_.releaseTile(freed);
    }
  }

  // Mutate parent: INTERNAL -> LEAF
  parent_tile->setLeaf(plocal);
  ++leaf_count_;
  --internal_count_;
  ++leaf_count_by_level_[parent_key.level];
  --internal_count_by_level_[parent_key.level];

  node_count_ -= 4;
  leaf_count_ -= 4;
  leaf_count_by_level_[child_level] -= 4;

  return true;
}

std::optional<avr::spatial::CellKey> AdaptiveHierarchy::refineToLevel(
    const avr::spatial::GridCoord2D& base,
    uint8_t requested_level) {
  if (!avr::spatial::isValidLevel(requested_level)) {
    return std::nullopt;
  }
  const auto root_opt = ensureRootForBase(base);
  if (!root_opt.has_value()) {
    return std::nullopt;
  }

  auto leaf_opt = findLeafContainingBase(base);
  if (!leaf_opt.has_value()) {
    return std::nullopt;
  }

  if (leaf_opt->level <= requested_level) {
    return leaf_opt;
  }

  while (leaf_opt->level > requested_level) {
    if (!splitLeaf(*leaf_opt)) {
      return std::nullopt;
    }
    leaf_opt = findLeafContainingBase(base);
    if (!leaf_opt.has_value()) {
      return std::nullopt;
    }
  }

  return leaf_opt;
}

TopologyValidationResult AdaptiveHierarchy::validateInvariants() const {
  // Check 1: Invariants H, G, F on all tiles
  for (const auto& [tile_key, tile_ptr] : tiles_) {
    if (tile_ptr == nullptr || !tile_pool_.owns(tile_ptr) || !tile_pool_.isAcquired(tile_ptr)) {
      return TopologyValidationResult{
          .valid = false,
          .category = TopologyValidationCategory::TILE_STORE_POINTER_INVALID,
          .error_message = "TileStore contains null, unowned, or unacquired TilePool pointer"};
    }
    if (tile_ptr->empty()) {
      return TopologyValidationResult{
          .valid = false,
          .category = TopologyValidationCategory::TILE_STORE_POINTER_INVALID,
          .error_message = "TileStore retains an empty tile that should have been released"};
    }
    if (!tile_ptr->validateInvariants()) {
      return TopologyValidationResult{
          .valid = false,
          .category = TopologyValidationCategory::MASK_COUNT_MISMATCH,
          .error_message = "AdaptiveTile bitset mask counts do not match counters"};
    }
    if (!avr::spatial::isValidLevel(tile_key.level)) {
      return TopologyValidationResult{
          .valid = false,
          .category = TopologyValidationCategory::INVALID_LEVEL,
          .error_message = "TileKey has invalid hierarchy level"};
    }
  }

  // Check 2: Structural tree invariants A, B, C, D, E across all allocated nodes
  size_t counted_nodes = 0;
  size_t counted_leaves = 0;
  size_t counted_internals = 0;
  std::array<size_t, 4> counted_leaves_by_lvl{0, 0, 0, 0};
  std::array<size_t, 4> counted_internals_by_lvl{0, 0, 0, 0};

  for (const auto& [tile_key, tile_ptr] : tiles_) {
    for (uint16_t idx = 0; idx < AdaptiveTile::CELL_COUNT; ++idx) {
      if (!tile_ptr->isPresent(idx)) {
        continue;
      }
      const auto cell_key_opt = avr::spatial::cellKeyFromTileAndLocal(tile_key, idx);
      if (!cell_key_opt.has_value()) {
        return TopologyValidationResult{
            .valid = false,
            .category = TopologyValidationCategory::OTHER,
            .error_message = "Failed to reconstruct CellKey from tile slot"};
      }
      const auto cell_key = *cell_key_opt;
      const bool is_leaf_node = tile_ptr->isLeaf(idx);

      ++counted_nodes;
      if (is_leaf_node) {
        ++counted_leaves;
        ++counted_leaves_by_lvl[cell_key.level];

        // Invariant A: Every LEAF has no child nodes
        if (cell_key.level > avr::spatial::L0) {
          const auto children_opt = avr::spatial::childrenOf(cell_key);
          if (children_opt.has_value()) {
            for (const auto& child : *children_opt) {
              if (nodeState(child) != NodeState::ABSENT) {
                return TopologyValidationResult{
                    .valid = false,
                    .category = TopologyValidationCategory::LEAF_HAS_CHILDREN,
                    .error_message = "LEAF node has allocated children in hierarchy"};
              }
            }
          }
        }
      } else {
        ++counted_internals;
        ++counted_internals_by_lvl[cell_key.level];

        // Invariant F: L0 nodes cannot be INTERNAL
        if (cell_key.level == avr::spatial::L0) {
          return TopologyValidationResult{
              .valid = false,
              .category = TopologyValidationCategory::INVALID_LEVEL,
              .error_message = "L0 cell cannot be an INTERNAL node"};
        }

        // Invariant B: Every INTERNAL node has exactly four direct child nodes
        const auto children_opt = avr::spatial::childrenOf(cell_key);
        if (!children_opt.has_value()) {
          return TopologyValidationResult{
              .valid = false,
              .category = TopologyValidationCategory::INTERNAL_MISSING_CHILDREN,
              .error_message = "INTERNAL node children coordinate overflowed"};
        }
        for (const auto& child : *children_opt) {
          if (nodeState(child) == NodeState::ABSENT) {
            return TopologyValidationResult{
                .valid = false,
                .category = TopologyValidationCategory::INTERNAL_CHILD_NOT_PRESENT,
                .error_message = "INTERNAL node is missing one or more direct children"};
          }
        }
      }

      // Invariant C & D: Parent validation
      if (cell_key.level == avr::spatial::L3) {
        // Invariant D: L3 roots are the only nodes allowed without a parent
      } else {
        // Invariant C: Every non-L3 allocated node has an allocated INTERNAL parent
        const auto parent_opt = avr::spatial::parentOf(cell_key);
        if (!parent_opt.has_value()) {
          return TopologyValidationResult{
              .valid = false,
              .category = TopologyValidationCategory::ORPHAN_NON_ROOT,
              .error_message = "Non-L3 node failed parent coordinate calculation"};
        }
        if (nodeState(*parent_opt) != NodeState::INTERNAL) {
          return TopologyValidationResult{
              .valid = false,
              .category = TopologyValidationCategory::ORPHAN_NON_ROOT,
              .error_message = "Non-L3 node lacks an allocated INTERNAL parent"};
        }
      }
    }
  }

  // Check aggregate and per-level counters
  if (counted_nodes != node_count_ || counted_leaves != leaf_count_ ||
      counted_internals != internal_count_ ||
      counted_leaves_by_lvl != leaf_count_by_level_ ||
      counted_internals_by_lvl != internal_count_by_level_) {
    return TopologyValidationResult{
        .valid = false,
        .category = TopologyValidationCategory::MASK_COUNT_MISMATCH,
        .error_message = "Hierarchy node counters do not match counted nodes across tiles"};
  }

  return TopologyValidationResult{.valid = true, .category = TopologyValidationCategory::VALID, .error_message = ""};
}

HierarchyStats AdaptiveHierarchy::statistics() const noexcept {
  HierarchyStats s;
  s.tile_count = tiles_.size();
  s.node_count = node_count_;
  s.leaf_count = leaf_count_;
  s.internal_count = internal_count_;
  s.leaf_count_by_level = leaf_count_by_level_;
  s.internal_count_by_level = internal_count_by_level_;

  s.pool_capacity = tile_pool_.capacity();
  s.pool_in_use = tile_pool_.inUse();
  s.pool_free = tile_pool_.freeCount();
  s.pool_block_count = tile_pool_.blockCount();

  s.sizeof_adaptive_tile = sizeof(AdaptiveTile);
  s.tile_payload_bytes = safeMul(s.tile_count, sizeof(AdaptiveTile));
  s.pool_reserved_payload_bytes = safeMul(s.pool_capacity, sizeof(AdaptiveTile));

  return s;
}

std::vector<avr::spatial::CellKey> AdaptiveHierarchy::leafSnapshot() const {
  std::vector<avr::spatial::CellKey> result;
  result.reserve(leaf_count_);

  for (const auto& [tile_key, tile_ptr] : tiles_) {
    for (uint16_t idx = 0; idx < AdaptiveTile::CELL_COUNT; ++idx) {
      if (tile_ptr->isLeaf(idx)) {
        const auto cell_key = avr::spatial::cellKeyFromTileAndLocal(tile_key, idx);
        if (cell_key.has_value()) {
          result.push_back(*cell_key);
        }
      }
    }
  }

  std::sort(result.begin(), result.end());
  return result;
}

std::vector<TopologyRecord> AdaptiveHierarchy::topologySnapshot() const {
  std::vector<TopologyRecord> result;
  result.reserve(node_count_);

  for (const auto& [tile_key, tile_ptr] : tiles_) {
    for (uint16_t idx = 0; idx < AdaptiveTile::CELL_COUNT; ++idx) {
      if (tile_ptr->isPresent(idx)) {
        const auto cell_key = avr::spatial::cellKeyFromTileAndLocal(tile_key, idx);
        if (cell_key.has_value()) {
          result.push_back(TopologyRecord{
              .key = *cell_key,
              .state = tile_ptr->nodeState(idx),
          });
        }
      }
    }
  }

  std::sort(result.begin(), result.end());
  return result;
}

void AdaptiveHierarchy::reset() noexcept {
  tiles_.clear();
  tile_pool_.reset();

  node_count_ = 0;
  leaf_count_ = 0;
  internal_count_ = 0;
  leaf_count_by_level_.fill(0);
  internal_count_by_level_.fill(0);
}

const AdaptiveTile* AdaptiveHierarchy::findTile(const avr::spatial::TileKey& tile_key) const noexcept {
  const auto it = tiles_.find(tile_key);
  return it != tiles_.end() ? it->second : nullptr;
}

AdaptiveTile* AdaptiveHierarchy::findTile(const avr::spatial::TileKey& tile_key) noexcept {
  const auto it = tiles_.find(tile_key);
  return it != tiles_.end() ? it->second : nullptr;
}

}  // namespace avr::mapping

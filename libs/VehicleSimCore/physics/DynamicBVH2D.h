#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "BroadphasePair2D.h"
#include "libs/VehicleSimCore/math/AABB2D.h"
#include "libs/VehicleSimCore/math/Ray2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {

/// @brief Constant denoting an invalid or null node index in the BVH tree.
inline constexpr std::int32_t kNullBVHNode { -1 };

/// @struct BVHNode2D
/// @brief Internal node in the dynamic bounding volume hierarchy tree.
/// @details Stored in a contiguous array with integer indices for memory safety.
struct BVHNode2D {
    /// @brief Fat bounding box enclosing this node and its children.
    math::AABB2D aabb {};

    /// @brief Pointer to user data attached to this proxy (e.g., Collider2D*).
    void* userData { nullptr };

    /// @brief Index of parent node (or kNullBVHNode if root or free).
    std::int32_t parentIndex { kNullBVHNode };

    /// @brief Index of left child node (or next free node in free-list).
    std::int32_t child1 { kNullBVHNode };

    /// @brief Index of right child node (or kNullBVHNode if leaf).
    std::int32_t child2 { kNullBVHNode };

    /// @brief Height of this subtree (0 for leaf, -1 for free node).
    std::int32_t height { 0 };

    /// @brief Tests whether this node is an active leaf.
    /// @return True if node is a leaf (child2 is kNullBVHNode).
    [[nodiscard]] constexpr bool isLeaf() const noexcept
    {
        return child2 == kNullBVHNode;
    }
};

/// @class DynamicBVH2D
/// @brief Self-balancing dynamic Axis-Aligned Bounding Box (AABB) tree broadphase.
/// @details Accelerates spatial collision queries, raycasting, and broadphase pair generation.
/// Implements Surface Area Heuristic (SAH) perimeter insertion and AVL tree rotations.
class DynamicBVH2D {
public:
    /// @brief Callback function type for AABB overlap queries.
    /// @details Return true to continue traversal, or false to abort query early.
    using QueryCallback = std::function<bool(std::int32_t proxyId)>;

    /// @brief Callback function type for raycast queries.
    /// @details Returns new maximum ray fraction in [0.0, 1.0], or 0.0 to terminate immediately.
    using RayCallback = std::function<double(const math::Ray2D& ray, std::int32_t proxyId)>;

    /// @brief Default constructor creating an empty tree with default capacity.
    DynamicBVH2D();

    /// @brief Parameterized constructor reserving initial node capacity.
    /// @param[in] initialCapacity Number of nodes to pre-allocate.
    explicit DynamicBVH2D(std::size_t initialCapacity);

    ~DynamicBVH2D() noexcept = default;

    DynamicBVH2D(const DynamicBVH2D&) = default;
    DynamicBVH2D& operator=(const DynamicBVH2D&) = default;
    DynamicBVH2D(DynamicBVH2D&&) noexcept = default;
    DynamicBVH2D& operator=(DynamicBVH2D&&) noexcept = default;

    /// @brief Creates a new proxy leaf node in the tree.
    /// @param[in] aabb Tight world-space bounding box.
    /// @param[in] userData User pointer attached to this proxy (e.g., Collider2D*).
    /// @return Unique proxy integer identifier (node index).
    [[nodiscard]] std::int32_t createProxy(const math::AABB2D& aabb, void* userData);

    /// @brief Destroys an existing proxy and removes it from the tree.
    /// @param[in] proxyId Unique proxy identifier to remove.
    void destroyProxy(std::int32_t proxyId);

    /// @brief Updates a proxy's bounding box with motion hysteresis.
    /// @details If tight AABB remains inside current fat AABB, skips re-insertion.
    /// @param[in] proxyId Proxy identifier to update.
    /// @param[in] aabb New tight world-space bounding box.
    /// @param[in] displacement Predicted linear motion vector (velocity * dt).
    /// @return True if tree topology was modified (re-inserted).
    bool moveProxy(std::int32_t proxyId, const math::AABB2D& aabb, const math::Vec2& displacement);

    /// @brief Gets user data associated with a proxy.
    /// @param[in] proxyId Proxy identifier.
    /// @return Attached user data pointer.
    [[nodiscard]] void* getUserData(std::int32_t proxyId) const;

    /// @brief Sets user data associated with a proxy.
    /// @param[in] proxyId Proxy identifier.
    /// @param[in] userData New user data pointer.
    void setUserData(std::int32_t proxyId, void* userData);

    /// @brief Gets the current fat AABB of a proxy.
    /// @param[in] proxyId Proxy identifier.
    /// @return Fat bounding box in world space.
    [[nodiscard]] const math::AABB2D& getFatAABB(std::int32_t proxyId) const;

    /// @brief Queries all proxy leaves whose fat AABB overlaps a query box.
    /// @param[in] aabb World-space query bounding box.
    /// @param[in] callback Invoked for each overlapping proxy leaf.
    void query(const math::AABB2D& aabb, const QueryCallback& callback) const;

    /// @brief Raycasts against all proxies in the tree with branch pruning.
    /// @param[in] ray 2D ray query.
    /// @param[in] callback Invoked for each intersected proxy leaf.
    void raycast(const math::Ray2D& ray, const RayCallback& callback) const;

    /// @brief Generates all overlapping proxy pairs.
    /// @param[out] outPairs Vector receiving unique overlapping pairs (proxyA < proxyB).
    void queryPairs(std::vector<BroadphasePair2D>& outPairs) const;

    /// @brief Computes total height of the tree.
    /// @return Height of root node (0 if empty).
    [[nodiscard]] std::int32_t getTreeHeight() const noexcept;

    /// @brief Gets total count of active proxy leaves.
    /// @return Active proxy count.
    [[nodiscard]] std::size_t getProxyCount() const noexcept;

    /// @brief Gets total allocated node capacity.
    /// @return Allocated node count.
    [[nodiscard]] std::size_t getNodeCount() const noexcept;

    /// @brief Validates tree invariants (height, balance, and bounding hierarchies).
    /// @return True if all tree invariants hold.
    [[nodiscard]] bool validate() const;

    /// @brief Rebuilds the tree bottom-up to optimize spatial layout.
    void rebuildBottomUp();

    /// @brief Clears all proxies and resets the tree.
    void clear() noexcept;

    /// @brief Gets the uniform margin expansion factor.
    /// @return Margin expansion in meters.
    [[nodiscard]] double getExpansionMargin() const noexcept;

    /// @brief Sets the uniform margin expansion factor.
    /// @param[in] margin Margin expansion in meters.
    void setExpansionMargin(double margin) noexcept;

    /// @brief Gets the predictive velocity extrusion multiplier.
    /// @return Velocity multiplier.
    [[nodiscard]] double getVelocityMultiplier() const noexcept;

    /// @brief Sets the predictive velocity extrusion multiplier.
    /// @param[in] multiplier Multiplier >= 1.0.
    void setVelocityMultiplier(double multiplier) noexcept;

private:
    [[nodiscard]] std::int32_t allocateNode();
    void freeNode(std::int32_t nodeIndex);
    void insertLeaf(std::int32_t leafIndex);
    void removeLeaf(std::int32_t leafIndex);
    [[nodiscard]] std::int32_t balance(std::int32_t nodeIndex);

    std::vector<BVHNode2D> mNodes {};
    std::int32_t mRoot { kNullBVHNode };
    std::int32_t mFreeListHead { 0 };
    std::size_t mProxyCount { 0 };
    double mExpansionMargin { 0.1 };
    double mVelocityMultiplier { 2.0 };
};

} // namespace vehicle_sim::physics

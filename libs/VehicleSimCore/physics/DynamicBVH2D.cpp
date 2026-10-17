#include "DynamicBVH2D.h"

#include <algorithm>
#include <cmath>
#include <stack>

#include "Collider2D.h"

namespace vehicle_sim::physics {

namespace {

[[nodiscard]] double halfPerimeter(const math::AABB2D& box) noexcept
{
    return (box.max.x - box.min.x) + (box.max.y - box.min.y);
}

[[nodiscard]] math::AABB2D combineAABB(const math::AABB2D& a, const math::AABB2D& b) noexcept
{
    return math::AABB2D {
        math::Vec2 { std::min(a.min.x, b.min.x), std::min(a.min.y, b.min.y) },
        math::Vec2 { std::max(a.max.x, b.max.x), std::max(a.max.y, b.max.y) }
    };
}

} // namespace

DynamicBVH2D::DynamicBVH2D()
    : DynamicBVH2D(32)
{
}

DynamicBVH2D::DynamicBVH2D(std::size_t initialCapacity)
{
    const std::size_t capacity { std::max<std::size_t>(initialCapacity, 16) };
    mNodes.resize(capacity);
    for (std::size_t i { 0 }; i < capacity - 1; ++i) {
        mNodes[i].child1 = static_cast<std::int32_t>(i + 1);
        mNodes[i].height = -1;
    }
    mNodes[capacity - 1].child1 = kNullBVHNode;
    mNodes[capacity - 1].height = -1;
    mFreeListHead = 0;
}

std::int32_t DynamicBVH2D::allocateNode()
{
    if (mFreeListHead == kNullBVHNode) {
        const std::size_t oldCapacity { mNodes.size() };
        const std::size_t newCapacity { oldCapacity * 2 };
        mNodes.resize(newCapacity);
        for (std::size_t i { oldCapacity }; i < newCapacity - 1; ++i) {
            mNodes[i].child1 = static_cast<std::int32_t>(i + 1);
            mNodes[i].height = -1;
        }
        mNodes[newCapacity - 1].child1 = kNullBVHNode;
        mNodes[newCapacity - 1].height = -1;
        mFreeListHead = static_cast<std::int32_t>(oldCapacity);
    }

    const std::int32_t nodeIndex { mFreeListHead };
    mFreeListHead = mNodes[static_cast<std::size_t>(nodeIndex)].child1;

    BVHNode2D& node { mNodes[static_cast<std::size_t>(nodeIndex)] };
    node.parentIndex = kNullBVHNode;
    node.child1 = kNullBVHNode;
    node.child2 = kNullBVHNode;
    node.height = 0;
    node.userData = nullptr;
    node.aabb = math::AABB2D {};

    return nodeIndex;
}

void DynamicBVH2D::freeNode(std::int32_t nodeIndex)
{
    if (nodeIndex < 0 || static_cast<std::size_t>(nodeIndex) >= mNodes.size()) {
        return;
    }

    BVHNode2D& node { mNodes[static_cast<std::size_t>(nodeIndex)] };
    node.child1 = mFreeListHead;
    node.child2 = kNullBVHNode;
    node.parentIndex = kNullBVHNode;
    node.height = -1;
    node.userData = nullptr;
    mFreeListHead = nodeIndex;
}

std::int32_t DynamicBVH2D::createProxy(const math::AABB2D& aabb, void* userData)
{
    const std::int32_t proxyId { allocateNode() };

    math::AABB2D fatBox { aabb.expanded(mExpansionMargin) };
    mNodes[static_cast<std::size_t>(proxyId)].aabb = fatBox;
    mNodes[static_cast<std::size_t>(proxyId)].userData = userData;
    mNodes[static_cast<std::size_t>(proxyId)].height = 0;

    insertLeaf(proxyId);
    ++mProxyCount;

    return proxyId;
}

void DynamicBVH2D::destroyProxy(std::int32_t proxyId)
{
    if (proxyId < 0 || static_cast<std::size_t>(proxyId) >= mNodes.size()) {
        return;
    }

    if (!mNodes[static_cast<std::size_t>(proxyId)].isLeaf()) {
        return;
    }

    removeLeaf(proxyId);
    freeNode(proxyId);
    --mProxyCount;
}

bool DynamicBVH2D::moveProxy(std::int32_t proxyId, const math::AABB2D& aabb, const math::Vec2& displacement)
{
    if (proxyId < 0 || static_cast<std::size_t>(proxyId) >= mNodes.size()) {
        return false;
    }

    BVHNode2D& leafNode { mNodes[static_cast<std::size_t>(proxyId)] };
    if (leafNode.aabb.contains(aabb)) {
        return false;
    }

    math::AABB2D fatBox { aabb.expanded(mExpansionMargin) };
    const math::Vec2 motion { displacement * mVelocityMultiplier };

    if (motion.x < 0.0) {
        fatBox.min.x += motion.x;
    } else {
        fatBox.max.x += motion.x;
    }

    if (motion.y < 0.0) {
        fatBox.min.y += motion.y;
    } else {
        fatBox.max.y += motion.y;
    }

    removeLeaf(proxyId);
    leafNode.aabb = fatBox;
    insertLeaf(proxyId);

    return true;
}

void* DynamicBVH2D::getUserData(std::int32_t proxyId) const
{
    if (proxyId < 0 || static_cast<std::size_t>(proxyId) >= mNodes.size()) {
        return nullptr;
    }
    return mNodes[static_cast<std::size_t>(proxyId)].userData;
}

void DynamicBVH2D::setUserData(std::int32_t proxyId, void* userData)
{
    if (proxyId >= 0 && static_cast<std::size_t>(proxyId) < mNodes.size()) {
        mNodes[static_cast<std::size_t>(proxyId)].userData = userData;
    }
}

const math::AABB2D& DynamicBVH2D::getFatAABB(std::int32_t proxyId) const
{
    static const math::AABB2D kDefaultAABB {};
    if (proxyId < 0 || static_cast<std::size_t>(proxyId) >= mNodes.size()) {
        return kDefaultAABB;
    }
    return mNodes[static_cast<std::size_t>(proxyId)].aabb;
}

void DynamicBVH2D::insertLeaf(std::int32_t leafIndex)
{
    if (mRoot == kNullBVHNode) {
        mRoot = leafIndex;
        mNodes[static_cast<std::size_t>(mRoot)].parentIndex = kNullBVHNode;
        return;
    }

    // 1. Find best sibling via Surface Area Heuristic (SAH minimal perimeter)
    const math::AABB2D leafAABB { mNodes[static_cast<std::size_t>(leafIndex)].aabb };
    std::int32_t index { mRoot };

    while (!mNodes[static_cast<std::size_t>(index)].isLeaf()) {
        const std::int32_t child1 { mNodes[static_cast<std::size_t>(index)].child1 };
        const std::int32_t child2 { mNodes[static_cast<std::size_t>(index)].child2 };

        const double area { halfPerimeter(mNodes[static_cast<std::size_t>(index)].aabb) };
        const math::AABB2D combinedAABB { combineAABB(mNodes[static_cast<std::size_t>(index)].aabb, leafAABB) };
        const double combinedArea { halfPerimeter(combinedAABB) };

        const double cost { 2.0 * combinedArea };
        const double inheritanceCost { 2.0 * (combinedArea - area) };

        double cost1 { 0.0 };
        const math::AABB2D aabb1 { combineAABB(mNodes[static_cast<std::size_t>(child1)].aabb, leafAABB) };
        if (mNodes[static_cast<std::size_t>(child1)].isLeaf()) {
            cost1 = halfPerimeter(aabb1) + inheritanceCost;
        } else {
            const double oldArea { halfPerimeter(mNodes[static_cast<std::size_t>(child1)].aabb) };
            const double newArea { halfPerimeter(aabb1) };
            cost1 = (newArea - oldArea) + inheritanceCost;
        }

        double cost2 { 0.0 };
        const math::AABB2D aabb2 { combineAABB(mNodes[static_cast<std::size_t>(child2)].aabb, leafAABB) };
        if (mNodes[static_cast<std::size_t>(child2)].isLeaf()) {
            cost2 = halfPerimeter(aabb2) + inheritanceCost;
        } else {
            const double oldArea { halfPerimeter(mNodes[static_cast<std::size_t>(child2)].aabb) };
            const double newArea { halfPerimeter(aabb2) };
            cost2 = (newArea - oldArea) + inheritanceCost;
        }

        if (cost < cost1 && cost < cost2) {
            break;
        }

        index = (cost1 < cost2) ? child1 : child2;
    }

    const std::int32_t sibling { index };

    // 2. Create new internal parent node
    const std::int32_t oldParent { mNodes[static_cast<std::size_t>(sibling)].parentIndex };
    const std::int32_t newParent { allocateNode() };

    BVHNode2D& parentNode { mNodes[static_cast<std::size_t>(newParent)] };
    parentNode.parentIndex = oldParent;
    parentNode.userData = nullptr;
    parentNode.aabb = combineAABB(leafAABB, mNodes[static_cast<std::size_t>(sibling)].aabb);
    parentNode.height = mNodes[static_cast<std::size_t>(sibling)].height + 1;

    if (oldParent != kNullBVHNode) {
        if (mNodes[static_cast<std::size_t>(oldParent)].child1 == sibling) {
            mNodes[static_cast<std::size_t>(oldParent)].child1 = newParent;
        } else {
            mNodes[static_cast<std::size_t>(oldParent)].child2 = newParent;
        }
    } else {
        mRoot = newParent;
    }

    parentNode.child1 = sibling;
    parentNode.child2 = leafIndex;
    mNodes[static_cast<std::size_t>(sibling)].parentIndex = newParent;
    mNodes[static_cast<std::size_t>(leafIndex)].parentIndex = newParent;

    // 3. Walk back up the tree refitting AABBs and balancing
    index = mNodes[static_cast<std::size_t>(leafIndex)].parentIndex;
    while (index != kNullBVHNode) {
        index = balance(index);

        const std::int32_t c1 { mNodes[static_cast<std::size_t>(index)].child1 };
        const std::int32_t c2 { mNodes[static_cast<std::size_t>(index)].child2 };

        mNodes[static_cast<std::size_t>(index)].height =
            1 + std::max(mNodes[static_cast<std::size_t>(c1)].height, mNodes[static_cast<std::size_t>(c2)].height);
        mNodes[static_cast<std::size_t>(index)].aabb =
            combineAABB(mNodes[static_cast<std::size_t>(c1)].aabb, mNodes[static_cast<std::size_t>(c2)].aabb);

        index = mNodes[static_cast<std::size_t>(index)].parentIndex;
    }
}

void DynamicBVH2D::removeLeaf(std::int32_t leafIndex)
{
    if (leafIndex == mRoot) {
        mRoot = kNullBVHNode;
        return;
    }

    const std::int32_t parent { mNodes[static_cast<std::size_t>(leafIndex)].parentIndex };
    const std::int32_t grandParent { mNodes[static_cast<std::size_t>(parent)].parentIndex };
    const std::int32_t sibling { (mNodes[static_cast<std::size_t>(parent)].child1 == leafIndex)
        ? mNodes[static_cast<std::size_t>(parent)].child2
        : mNodes[static_cast<std::size_t>(parent)].child1 };

    if (grandParent != kNullBVHNode) {
        if (mNodes[static_cast<std::size_t>(grandParent)].child1 == parent) {
            mNodes[static_cast<std::size_t>(grandParent)].child1 = sibling;
        } else {
            mNodes[static_cast<std::size_t>(grandParent)].child2 = sibling;
        }
        mNodes[static_cast<std::size_t>(sibling)].parentIndex = grandParent;
        freeNode(parent);

        std::int32_t index { grandParent };
        while (index != kNullBVHNode) {
            index = balance(index);

            const std::int32_t c1 { mNodes[static_cast<std::size_t>(index)].child1 };
            const std::int32_t c2 { mNodes[static_cast<std::size_t>(index)].child2 };

            mNodes[static_cast<std::size_t>(index)].aabb =
                combineAABB(mNodes[static_cast<std::size_t>(c1)].aabb, mNodes[static_cast<std::size_t>(c2)].aabb);
            mNodes[static_cast<std::size_t>(index)].height =
                1 + std::max(mNodes[static_cast<std::size_t>(c1)].height, mNodes[static_cast<std::size_t>(c2)].height);

            index = mNodes[static_cast<std::size_t>(index)].parentIndex;
        }
    } else {
        mRoot = sibling;
        mNodes[static_cast<std::size_t>(sibling)].parentIndex = kNullBVHNode;
        freeNode(parent);
    }
}

std::int32_t DynamicBVH2D::balance(std::int32_t iA)
{
    BVHNode2D& nodeA { mNodes[static_cast<std::size_t>(iA)] };
    if (nodeA.isLeaf() || nodeA.height < 2) {
        return iA;
    }

    const std::int32_t iB { nodeA.child1 };
    const std::int32_t iC { nodeA.child2 };

    const BVHNode2D& nodeB { mNodes[static_cast<std::size_t>(iB)] };
    const BVHNode2D& nodeC { mNodes[static_cast<std::size_t>(iC)] };

    const std::int32_t balanceFactor { nodeC.height - nodeB.height };

    // Rotate C up
    if (balanceFactor > 1) {
        const std::int32_t iF { nodeC.child1 };
        const std::int32_t iG { nodeC.child2 };

        BVHNode2D& modC { mNodes[static_cast<std::size_t>(iC)] };
        modC.child1 = iA;
        modC.parentIndex = nodeA.parentIndex;
        nodeA.parentIndex = iC;

        if (modC.parentIndex != kNullBVHNode) {
            if (mNodes[static_cast<std::size_t>(modC.parentIndex)].child1 == iA) {
                mNodes[static_cast<std::size_t>(modC.parentIndex)].child1 = iC;
            } else {
                mNodes[static_cast<std::size_t>(modC.parentIndex)].child2 = iC;
            }
        } else {
            mRoot = iC;
        }

        if (mNodes[static_cast<std::size_t>(iF)].height > mNodes[static_cast<std::size_t>(iG)].height) {
            modC.child2 = iF;
            nodeA.child2 = iG;
            mNodes[static_cast<std::size_t>(iG)].parentIndex = iA;

            nodeA.aabb = combineAABB(nodeB.aabb, mNodes[static_cast<std::size_t>(iG)].aabb);
            modC.aabb = combineAABB(nodeA.aabb, mNodes[static_cast<std::size_t>(iF)].aabb);

            nodeA.height = 1 + std::max(nodeB.height, mNodes[static_cast<std::size_t>(iG)].height);
            modC.height = 1 + std::max(nodeA.height, mNodes[static_cast<std::size_t>(iF)].height);
        } else {
            modC.child2 = iG;
            nodeA.child2 = iF;
            mNodes[static_cast<std::size_t>(iF)].parentIndex = iA;

            nodeA.aabb = combineAABB(nodeB.aabb, mNodes[static_cast<std::size_t>(iF)].aabb);
            modC.aabb = combineAABB(nodeA.aabb, mNodes[static_cast<std::size_t>(iG)].aabb);

            nodeA.height = 1 + std::max(nodeB.height, mNodes[static_cast<std::size_t>(iF)].height);
            modC.height = 1 + std::max(nodeA.height, mNodes[static_cast<std::size_t>(iG)].height);
        }

        return iC;
    }

    // Rotate B up
    if (balanceFactor < -1) {
        const std::int32_t iD { nodeB.child1 };
        const std::int32_t iE { nodeB.child2 };

        BVHNode2D& modB { mNodes[static_cast<std::size_t>(iB)] };
        modB.child1 = iA;
        modB.parentIndex = nodeA.parentIndex;
        nodeA.parentIndex = iB;

        if (modB.parentIndex != kNullBVHNode) {
            if (mNodes[static_cast<std::size_t>(modB.parentIndex)].child1 == iA) {
                mNodes[static_cast<std::size_t>(modB.parentIndex)].child1 = iB;
            } else {
                mNodes[static_cast<std::size_t>(modB.parentIndex)].child2 = iB;
            }
        } else {
            mRoot = iB;
        }

        if (mNodes[static_cast<std::size_t>(iD)].height > mNodes[static_cast<std::size_t>(iE)].height) {
            modB.child2 = iD;
            nodeA.child1 = iE;
            mNodes[static_cast<std::size_t>(iE)].parentIndex = iA;

            nodeA.aabb = combineAABB(nodeC.aabb, mNodes[static_cast<std::size_t>(iE)].aabb);
            modB.aabb = combineAABB(nodeA.aabb, mNodes[static_cast<std::size_t>(iD)].aabb);

            nodeA.height = 1 + std::max(nodeC.height, mNodes[static_cast<std::size_t>(iE)].height);
            modB.height = 1 + std::max(nodeA.height, mNodes[static_cast<std::size_t>(iD)].height);
        } else {
            modB.child2 = iE;
            nodeA.child1 = iD;
            mNodes[static_cast<std::size_t>(iD)].parentIndex = iA;

            nodeA.aabb = combineAABB(nodeC.aabb, mNodes[static_cast<std::size_t>(iD)].aabb);
            modB.aabb = combineAABB(nodeA.aabb, mNodes[static_cast<std::size_t>(iE)].aabb);

            nodeA.height = 1 + std::max(nodeC.height, mNodes[static_cast<std::size_t>(iD)].height);
            modB.height = 1 + std::max(nodeA.height, mNodes[static_cast<std::size_t>(iE)].height);
        }

        return iB;
    }

    return iA;
}

void DynamicBVH2D::query(const math::AABB2D& aabb, const QueryCallback& callback) const
{
    if (mRoot == kNullBVHNode) {
        return;
    }

    std::vector<std::int32_t> stack {};
    stack.reserve(64);
    stack.push_back(mRoot);

    while (!stack.empty()) {
        const std::int32_t nodeIndex { stack.back() };
        stack.pop_back();

        const BVHNode2D& node { mNodes[static_cast<std::size_t>(nodeIndex)] };
        if (node.aabb.intersects(aabb)) {
            if (node.isLeaf()) {
                const bool proceed { callback(nodeIndex) };
                if (!proceed) {
                    return;
                }
            } else {
                stack.push_back(node.child1);
                stack.push_back(node.child2);
            }
        }
    }
}

void DynamicBVH2D::raycast(const math::Ray2D& ray, const RayCallback& callback) const
{
    if (mRoot == kNullBVHNode) {
        return;
    }

    double maxFraction { 1.0 };
    std::vector<std::int32_t> stack {};
    stack.reserve(64);
    stack.push_back(mRoot);

    while (!stack.empty()) {
        const std::int32_t nodeIndex { stack.back() };
        stack.pop_back();

        const BVHNode2D& node { mNodes[static_cast<std::size_t>(nodeIndex)] };

        // Test ray against fat AABB
        const auto hit { node.aabb.raycast(ray) };
        if (!hit.has_value()) {
            continue;
        }

        if (hit->distance > maxFraction * ray.maxDistance) {
            continue;
        }

        if (node.isLeaf()) {
            const double newFraction { callback(ray, nodeIndex) };
            if (newFraction < maxFraction) {
                maxFraction = newFraction;
            }
            if (maxFraction <= 0.0) {
                return;
            }
        } else {
            const auto hit1 { mNodes[static_cast<std::size_t>(node.child1)].aabb.raycast(ray) };
            const auto hit2 { mNodes[static_cast<std::size_t>(node.child2)].aabb.raycast(ray) };

            const double dist1 { hit1.has_value() ? hit1->distance : 1e30 };
            const double dist2 { hit2.has_value() ? hit2->distance : 1e30 };

            if (dist1 <= dist2) {
                if (hit2.has_value() && dist2 <= maxFraction * ray.maxDistance) {
                    stack.push_back(node.child2);
                }
                if (hit1.has_value() && dist1 <= maxFraction * ray.maxDistance) {
                    stack.push_back(node.child1);
                }
            } else {
                if (hit1.has_value() && dist1 <= maxFraction * ray.maxDistance) {
                    stack.push_back(node.child1);
                }
                if (hit2.has_value() && dist2 <= maxFraction * ray.maxDistance) {
                    stack.push_back(node.child2);
                }
            }
        }
    }
}

void DynamicBVH2D::queryPairs(std::vector<BroadphasePair2D>& outPairs) const
{
    if (mProxyCount < 2 || mRoot == kNullBVHNode) {
        return;
    }

    // Query each active leaf against the tree
    for (std::size_t i { 0 }; i < mNodes.size(); ++i) {
        const BVHNode2D& leafNode { mNodes[i] };
        if (leafNode.height != 0 || !leafNode.isLeaf()) {
            continue;
        }

        auto* colA { static_cast<Collider2D*>(leafNode.userData) };
        const std::int32_t proxyA { static_cast<std::int32_t>(i) };

        query(leafNode.aabb, [&](std::int32_t proxyB) -> bool {
            if (proxyA >= proxyB) {
                return true;
            }

            auto* colB { static_cast<Collider2D*>(mNodes[static_cast<std::size_t>(proxyB)].userData) };
            if (colA != nullptr && colB != nullptr) {
                if (!colA->canCollideWith(*colB)) {
                    return true;
                }
                if (colA->getBody() != nullptr && colB->getBody() != nullptr && colA->getBody() == colB->getBody()) {
                    return true;
                }
            }

            outPairs.push_back(BroadphasePair2D { colA, colB, proxyA, proxyB });
            return true;
        });
    }
}

std::int32_t DynamicBVH2D::getTreeHeight() const noexcept
{
    if (mRoot == kNullBVHNode) {
        return 0;
    }
    return mNodes[static_cast<std::size_t>(mRoot)].height;
}

std::size_t DynamicBVH2D::getProxyCount() const noexcept
{
    return mProxyCount;
}

std::size_t DynamicBVH2D::getNodeCount() const noexcept
{
    return mNodes.size();
}

bool DynamicBVH2D::validate() const
{
    if (mRoot == kNullBVHNode) {
        return mProxyCount == 0;
    }

    std::vector<std::int32_t> stack {};
    stack.push_back(mRoot);
    std::size_t leafCount { 0 };

    while (!stack.empty()) {
        const std::int32_t index { stack.back() };
        stack.pop_back();

        const BVHNode2D& node { mNodes[static_cast<std::size_t>(index)] };
        if (node.isLeaf()) {
            if (node.height != 0) {
                return false;
            }
            ++leafCount;
        } else {
            const std::int32_t c1 { node.child1 };
            const std::int32_t c2 { node.child2 };

            if (c1 < 0 || c2 < 0) {
                return false;
            }
            if (mNodes[static_cast<std::size_t>(c1)].parentIndex != index) {
                return false;
            }
            if (mNodes[static_cast<std::size_t>(c2)].parentIndex != index) {
                return false;
            }

            const std::int32_t expectedHeight {
                1 + std::max(mNodes[static_cast<std::size_t>(c1)].height, mNodes[static_cast<std::size_t>(c2)].height)
            };
            if (node.height != expectedHeight) {
                return false;
            }

            const std::int32_t balanceFactor {
                std::abs(mNodes[static_cast<std::size_t>(c2)].height - mNodes[static_cast<std::size_t>(c1)].height)
            };
            if (balanceFactor > 2) {
                return false;
            }

            if (!node.aabb.contains(mNodes[static_cast<std::size_t>(c1)].aabb)) {
                return false;
            }
            if (!node.aabb.contains(mNodes[static_cast<std::size_t>(c2)].aabb)) {
                return false;
            }

            stack.push_back(c1);
            stack.push_back(c2);
        }
    }

    return leafCount == mProxyCount;
}

void DynamicBVH2D::rebuildBottomUp()
{
    if (mProxyCount < 2 || mRoot == kNullBVHNode) {
        return;
    }

    std::vector<std::int32_t> leaves {};
    leaves.reserve(mProxyCount);

    for (std::size_t i { 0 }; i < mNodes.size(); ++i) {
        if (mNodes[i].isLeaf() && mNodes[i].height == 0) {
            leaves.push_back(static_cast<std::int32_t>(i));
        }
    }

    mRoot = kNullBVHNode;
    for (const auto leaf : leaves) {
        mNodes[static_cast<std::size_t>(leaf)].parentIndex = kNullBVHNode;
        mNodes[static_cast<std::size_t>(leaf)].child1 = kNullBVHNode;
        mNodes[static_cast<std::size_t>(leaf)].child2 = kNullBVHNode;
        mNodes[static_cast<std::size_t>(leaf)].height = 0;
        insertLeaf(leaf);
    }
}

void DynamicBVH2D::clear() noexcept
{
    mRoot = kNullBVHNode;
    mProxyCount = 0;
    const std::size_t capacity { mNodes.size() };
    for (std::size_t i { 0 }; i < capacity - 1; ++i) {
        mNodes[i].child1 = static_cast<std::int32_t>(i + 1);
        mNodes[i].child2 = kNullBVHNode;
        mNodes[i].parentIndex = kNullBVHNode;
        mNodes[i].height = -1;
        mNodes[i].userData = nullptr;
    }
    mNodes[capacity - 1].child1 = kNullBVHNode;
    mNodes[capacity - 1].child2 = kNullBVHNode;
    mNodes[capacity - 1].parentIndex = kNullBVHNode;
    mNodes[capacity - 1].height = -1;
    mNodes[capacity - 1].userData = nullptr;
    mFreeListHead = 0;
}

double DynamicBVH2D::getExpansionMargin() const noexcept
{
    return mExpansionMargin;
}

void DynamicBVH2D::setExpansionMargin(double margin) noexcept
{
    mExpansionMargin = (margin > 0.0) ? margin : 0.0;
}

double DynamicBVH2D::getVelocityMultiplier() const noexcept
{
    return mVelocityMultiplier;
}

void DynamicBVH2D::setVelocityMultiplier(double multiplier) noexcept
{
    mVelocityMultiplier = (multiplier >= 1.0) ? multiplier : 1.0;
}

} // namespace vehicle_sim::physics

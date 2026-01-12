#include "kdtree.h"
#include <algorithm> // std::nth_element
#include <limits>
#include <cmath>

KDTree::KDTree(const std::vector<Point3f>& points)
{
    m_points = points;               // 원본 좌표 복사
    int N = (int)points.size();
    m_indices.resize(N);
    for(int i=0; i<N; ++i){
        m_indices[i] = i;           // [0..N-1]
    }
    if(N > 0) {
        root_ = buildTree(0, N, 0);
    }
}

KDTree::~KDTree()
{
    deleteNode(root_);
    root_ = nullptr;
}

void KDTree::deleteNode(Node* node)
{
    if(!node) return;
    deleteNode(node->left);
    deleteNode(node->right);
    delete node;
}

// 빌드
KDTree::Node* KDTree::buildTree(int start, int end, int depth)
{
    if(start >= end) return nullptr;

    int axis = depth % 3;
    int mid  = (start + end)/2;

    // compare using m_points[m_indices[i]][axis]
    auto compareAxis = [&](int idxA, int idxB){
        return m_points[idxA][axis] < m_points[idxB][axis];
    };

    // ↓↓↓ 4개의 인자를 전달해야 합니다. ↓↓↓
    std::nth_element(
        m_indices.begin() + start,
        m_indices.begin() + mid,
        m_indices.begin() + end,
        compareAxis
    );

    int pivotIdx = m_indices[mid];
    Node* node = new Node();
    node->index = pivotIdx;

    node->left  = buildTree(start,   mid, depth+1);
    node->right = buildTree(mid + 1, end, depth+1);
    return node;
}

std::pair<float, KDTree::Point3f>
KDTree::nearestNeighbor(const Point3f& query) const
{
    float bestDist = std::numeric_limits<float>::max();
    int bestIdx = -1;
    searchNN(root_, query, 0, bestDist, bestIdx);

    if(bestIdx < 0) {
        return { -1.f, {0.f,0.f,0.f} };
    } else {
        float dist = std::sqrt(bestDist);
        return { dist, m_points[bestIdx] };
    }
}

void KDTree::searchNN(Node* node,
                      const Point3f& query,
                      int depth,
                      float& bestDist,
                      int& bestIdx) const
{
    if(!node) return;
    int idx = node->index;

    float dx = m_points[idx][0] - query[0];
    float dy = m_points[idx][1] - query[1];
    float dz = m_points[idx][2] - query[2];
    float distSq = dx*dx + dy*dy + dz*dz;
    if(distSq < bestDist){
        bestDist = distSq;
        bestIdx  = idx;
    }

    int axis = depth % 3;
    float diff = query[axis] - m_points[idx][axis];

    Node* first  = (diff < 0.f) ? node->left : node->right;
    Node* second = (diff < 0.f) ? node->right : node->left;

    searchNN(first, query, depth+1, bestDist, bestIdx);

    if(diff*diff < bestDist){
        searchNN(second, query, depth+1, bestDist, bestIdx);
    }
}

std::pair<std::vector<float>, std::vector<int>>
KDTree::query(const std::vector<Point3f>& queries) const
{
    int M = (int)queries.size();
    std::vector<float> dists(M);
    std::vector<int>   idxs(M);

    for(int i=0; i<M; ++i){
        float bestDist = std::numeric_limits<float>::max();
        int bestIdx = -1;
        searchNN(root_, queries[i], 0, bestDist, bestIdx);
        if(bestIdx < 0){
            dists[i] = -1.f;
            idxs[i]  = -1;
        } else {
            dists[i] = std::sqrt(bestDist);
            idxs[i]  = bestIdx;
        }
    }
    return { dists, idxs };
}

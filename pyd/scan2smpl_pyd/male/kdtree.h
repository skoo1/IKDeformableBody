#ifndef KDTREE_H
#define KDTREE_H

#include <vector>
#include <array>

class KDTree
{
public:
    using Point3f = std::array<float, 3>;

    KDTree(const std::vector<Point3f>& points);
    ~KDTree();

    // (distance, point)
    std::pair<float, Point3f> nearestNeighbor(const Point3f& query) const;

    // (distances, indices)
    std::pair<std::vector<float>, std::vector<int>>
    query(const std::vector<Point3f>& queries) const;

private:
    struct Node {
        int index;  // ← 원본 인덱스
        Node* left;
        Node* right;
    };

    std::vector<Point3f> m_points;   // 원본 좌표 (절대 재배열 안 함)
    std::vector<int>     m_indices;  // 인덱스 배열만 in-place 재배열

    Node* root_ = nullptr;

    Node* buildTree(int start, int end, int depth);
    void deleteNode(Node* node);

    void searchNN(Node* node,
                  const Point3f& query,
                  int depth,
                  float& bestDist,
                  int& bestIdx) const;
};

#endif

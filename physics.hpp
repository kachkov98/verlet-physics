#ifndef PHYSICS_HPP
#define PHYSICS_HPP

#include <glm/gtc/constants.hpp>
#include <glm/geometric.hpp>
#include <glm/vec2.hpp>
#include <algorithm>
#include <optional>
#include <vector>

constexpr float pos_inf =  std::numeric_limits<float>::infinity();
constexpr float neg_inf = -std::numeric_limits<float>::infinity();

class ImVec2;
class World;

class StaticPoint {
public:
  StaticPoint(glm::vec2 pos):
    pos_(pos)
  {}
  glm::vec2 get() const {
    return pos_;
  }
  virtual ~StaticPoint()
  {}
protected:
  glm::vec2 pos_;
};

class DynamicPoint final : public StaticPoint {
public:
  DynamicPoint(glm::vec2 pos):
    StaticPoint(pos), prev_pos_(pos)
  {}
  void update(glm::vec2 offset) {
    glm::vec2 delta = pos_ - prev_pos_;
    prev_pos_ = pos_;
    pos_ += delta + offset;
  }
  void set(glm::vec2 pos) {
    pos_ = pos;
  }
protected:
  glm::vec2 prev_pos_;
};

class PointIterator {
public:
  enum Type {
    STATIC,
    DYNAMIC
  };
  PointIterator() = default;
  PointIterator(uint32_t index, Type type):
    iterator_(index << 1 | type)
  {};
  uint32_t getIndex() const {
    return iterator_ >> 1;
  }
  Type getType() const {
    return static_cast<Type>(iterator_ & 1);
  }
private:
  uint32_t iterator_;
};

using Edge = std::pair<PointIterator, PointIterator>;

class Link {
public:
  Link(Edge edge, float length):
    edge_(edge),
    length_(length)
  {}
  void resolve(World& world, float stiffness) const;

  Edge get() const {
    return edge_;
  }
private:
  Edge edge_;
  float length_;
};

class BoundingBox {
public:
  BoundingBox():
    min_(pos_inf, pos_inf),
    max_(neg_inf, neg_inf)
  {}
  BoundingBox(glm::vec2 min, glm::vec2 max):
    min_(min),
    max_(max) {
    assert(min_.x < max_.x && min_.y < max_.y);
  }
  bool overlaps(const BoundingBox &other) const {
    return !(other.min_.x > max_.x ||
             other.max_.x < min_.x ||
             other.min_.y > max_.y ||
             other.max_.y < min_.y);
  };
private:
  glm::vec2 min_, max_;
};

struct CollisionInfo {
  Edge edge;
  PointIterator point;
  glm::vec2 normal;
  float depth;
};

class Collider {
public:
  Collider(const std::vector<PointIterator>& shape):
    shape_(shape) {
    auto isDynamic = [](const PointIterator& p) {
      return p.getType() == PointIterator::DYNAMIC;
    };
    is_dynamic_ = std::any_of(shape_.begin(), shape_.end(), isDynamic);
  }

  bool isDynamic() const {
    return is_dynamic_;
  }

  void calculateBoundingBox(World &world);

  PointIterator findNearestPoint(World &world, Edge edge) const;

  std::optional<CollisionInfo>
  collide(World& world, const Collider &other) const {
    if (!bbox_.overlaps(other.bbox_))
      return std::nullopt;
    return detectCollision(world, other);
  }

  const std::vector<PointIterator> &get() const {
    return shape_;
  }
private:
  std::vector<PointIterator> shape_;
  BoundingBox bbox_;
  bool is_dynamic_;

  std::pair<float, float> project(World& world, glm::vec2 axis) const;

  std::optional<CollisionInfo>
  detectCollision(World &world, const Collider &other) const;
};

class Viewport {
public:
  Viewport(ImVec2 extent, glm::vec2 center, float scale):
    extent_(extent),
    center_(center),
    scale_(scale)
  {}
  void setExtent(ImVec2 extent) {
    extent_ = extent;
  }
  void moveCenter(ImVec2 delta) {
    center_ += glm::vec2(-delta.x / scale_, delta.y /scale_);
  }
  void changeScale(float multiplier) {
    scale_ *= multiplier;
  }
  glm::vec2 screenToWorld(ImVec2 screen) const;
  ImVec2 worldToScreen(glm::vec2 world) const;
private:
  ImVec2 extent_;
  glm::vec2 center_;
  float scale_;
};

class World {
public:
  static constexpr glm::vec2 acceleration = glm::vec2(0.0f, -9.8f);
  static constexpr float dt = 0.016f;
  static constexpr float stiffness = 0.99f;

  World() = default;
  World(const World&) = delete;
  World operator=(const World&) = delete;

  void update(unsigned num_iterations);
  void render(const Viewport &viewport) const;

  PointIterator addPoint(glm::vec2 pos, PointIterator::Type type) {
    uint32_t index;
    if (type == PointIterator::STATIC) {
      index = static_points_.size();
      static_points_.emplace_back(pos);
    }
    else {
      index = dynamic_points_.size();
      dynamic_points_.emplace_back(pos);
    }
    return PointIterator(index, type);
  }

  StaticPoint getPoint(PointIterator iterator) const {
    auto index = iterator.getIndex();
    if (iterator.getType() == PointIterator::STATIC)
      return static_points_[index];
    else
      return dynamic_points_[index];
  }

  void setPoint(PointIterator iterator, glm::vec2 pos) {
    assert(iterator.getType() == PointIterator::DYNAMIC);
    dynamic_points_[iterator.getIndex()].set(pos);
  }

  std::optional<PointIterator>
  getNearestDynamicPoint(glm::vec2 pos, float cutoff) const;

  void addLink(Edge edge) {
    float length = glm::distance(getPoint(edge.first).get(),
                                 getPoint(edge.second).get());
    links_.emplace_back(edge, length);
  }

  void addCollider(const std::vector<PointIterator> &shape) {
    Collider collider(shape);
    if (!collider.isDynamic())
      collider.calculateBoundingBox(*this);
    colliders_.push_back(collider);
  }

  void reset();
private:
  std::vector<StaticPoint> static_points_;
  std::vector<DynamicPoint> dynamic_points_;
  std::vector<Link> links_;
  std::vector<Collider> colliders_;

  void integrate(glm::vec2 acceleration, float dt);
  void resolveLinks(float stiffness);
  std::vector<CollisionInfo> collisionDetection();
  void collisionResponce(const std::vector<CollisionInfo>& collisionInfo);
};

#endif

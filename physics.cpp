#include <glm/gtx/norm.hpp>
#include <imgui.h>
#include "physics.hpp"

void Link::resolve(World& world, float stiffness) const {
  if (edge_.first.getType() == PointIterator::STATIC &&
      edge_.second.getType() == PointIterator::STATIC)
    return;

  glm::vec2 p1 = world.getPoint(edge_.first).get(),
            p2 = world.getPoint(edge_.second).get(),
            diff = (p2 - p1) * (length_ / glm::distance(p1, p2) - 1.0f) * stiffness;

  if (edge_.first.getType() == PointIterator::STATIC) {
    world.setPoint(edge_.second, p2 + diff);
    return;
  }
  if (edge_.second.getType() == PointIterator::STATIC) {
    world.setPoint(edge_.first, p1 - diff);
    return;
  }
  world.setPoint(edge_.first,  p1 - 0.5f * diff);
  world.setPoint(edge_.second, p2 + 0.5f * diff);
}

void Collider::calculateBoundingBox(World &world) {
  glm::vec2 min(pos_inf, pos_inf),
            max(neg_inf, neg_inf);
  for (const auto &pointIter: shape_) {
    glm::vec2 pos = world.getPoint(pointIter).get();
    min.x = std::min(min.x, pos.x);
    min.y = std::min(min.y, pos.y);
    max.x = std::max(max.x, pos.x);
    max.y = std::max(max.y, pos.y);
  }
  bbox_ = BoundingBox(min, max);
}
std::pair<float, float>
Collider::project(World &world, glm::vec2 axis) const {
  float min = pos_inf, max = neg_inf;
  for (auto pointIter: shape_) {
    glm::vec2 pos = world.getPoint(pointIter).get();
    min = std::min(min, glm::dot(axis, pos));
    max = std::max(max, glm::dot(axis, pos));
  }
  return {min, max};
}

PointIterator Collider::findNearestPoint(World &world, Edge edge) const {
  glm::vec2 p1 = world.getPoint(edge.first).get(),
            p2 = world.getPoint(edge.second).get();
  glm::vec2 normal = glm::normalize(glm::vec2(p1.y - p2.y, p2.x - p1.x));
  float minDist = pos_inf;
  PointIterator res;
  for (auto point: shape_) {
    float dist = glm::dot(normal, world.getPoint(point).get() - p1);
    if (dist < minDist) {
      minDist = dist;
      res = point;
    }
  }
  return res;
}

std::optional<CollisionInfo>
Collider::detectCollision(World &world, const Collider &other) const {
  CollisionInfo info;
  info.depth = pos_inf;
  bool isFst;
  for (unsigned i = 0; i < shape_.size() + other.shape_.size(); ++i) {
    Edge edge;
    if (i < shape_.size())
      edge = Edge(shape_[i], shape_[(i + 1) % shape_.size()]);
    else
      edge = Edge(other.shape_[i - shape_.size()],
                  other.shape_[(i - shape_.size() + 1) % other.shape_.size()]);
    glm::vec2 p1 = world.getPoint(edge.first).get(),
              p2 = world.getPoint(edge.second).get();
    glm::vec2 normal = glm::normalize(glm::vec2(p1.y - p2.y, p2.x - p1.x));
    auto [minFst, maxFst] = project(world, normal);
    auto [minSnd, maxSnd] = other.project(world, normal);
    float distance = minFst < minSnd ? minSnd - maxFst : minFst - maxSnd;
    if (distance > 0.0f)
      return std::nullopt;
    if (i < shape_.size()) {
      if (minSnd < minFst)
        continue;
    }
    else {
      if (minFst < minSnd)
        continue;
    }
    distance *= -1.0f;
    if (distance < info.depth) {
      info.depth = distance;
      info.edge = edge;
      info.normal = normal;
      isFst = i < shape_.size();
    }
  }
  info.point = isFst ? other.findNearestPoint(world, info.edge) :
                             findNearestPoint(world, info.edge);
  return info;
}

glm::vec2 Viewport::screenToWorld(ImVec2 screen) const {
  return center_ + glm::vec2((screen.x - extent_.x / 2) / scale_,
                             (extent_.y / 2 - screen.y) / scale_);
}

ImVec2 Viewport::worldToScreen(glm::vec2 world) const {
  return ImVec2(extent_.x / 2 + (world.x - center_.x) * scale_,
                extent_.y / 2 - (world.y - center_.y) * scale_);
}

void World::update(unsigned num_iterations) {
  integrate(acceleration, dt);
  for (unsigned i = 0; i < num_iterations; ++i) {
    resolveLinks(stiffness);
    collisionResponce(collisionDetection());
  }
}

void World::render(const Viewport &viewport) const {
  ImDrawList *draw_list = ImGui::GetBackgroundDrawList();
  // render links
  for (const auto &link: links_) {
    glm::vec2 p1 = getPoint(link.get().first).get(),
              p2 = getPoint(link.get().second).get();
    draw_list->AddLine(viewport.worldToScreen(p1),
                       viewport.worldToScreen(p2),
                       IM_COL32(191, 191, 191, 255));
  }
  // render colliders
  for (const auto &collider: colliders_) {
    std::vector<ImVec2> points;
    points.reserve(collider.get().size());
    for (auto pointIter: collider.get())
      points.push_back(viewport.worldToScreen(getPoint(pointIter).get()));
    draw_list->AddPolyline(points.data(), points.size(),
                           IM_COL32(255, 0, 0, 255),
                           ImDrawFlags_Closed, 2.0f);
  }
  // render static points
  for (const auto &point: static_points_)
    draw_list->AddCircleFilled(viewport.worldToScreen(point.get()),
                               3.0f, IM_COL32(127, 127, 127, 255));
  // render dynamic points
  for (const auto &point: dynamic_points_)
    draw_list->AddCircleFilled(viewport.worldToScreen(point.get()),
                               3.0f, IM_COL32(255, 255, 255, 255));
}
  
std::optional<PointIterator>
World::getNearestDynamicPoint(glm::vec2 pos, float cutoff) const {
  std::optional<PointIterator> result;
  float min_distance = pos_inf;
  for (unsigned i = 0; i < dynamic_points_.size(); ++i) {
    auto distance = glm::distance2(pos, dynamic_points_[i].get());
    if (cutoff * cutoff < distance)
      continue;
    if (distance < min_distance) {
      min_distance = distance;
      result = PointIterator(i, PointIterator::DYNAMIC);
    }
  }
  return result;
}

void World::reset() {
  static_points_.clear();
  dynamic_points_.clear();
  links_.clear();
  colliders_.clear();
}

void World::integrate(glm::vec2 acceleration, float dt) {
  glm::vec2 offset = acceleration * dt * dt;
  for (auto& point: dynamic_points_)
    point.update(offset);
}

void World::resolveLinks(float stiffness) {
  for (const auto& link : links_)
    link.resolve(*this, stiffness);
}

std::vector<CollisionInfo> World::collisionDetection() {
  // update bounding boxes
  for (auto &collider: colliders_)
    if (collider.isDynamic())
      collider.calculateBoundingBox(*this);
  // calculate collision info between collider pairs
  std::vector<CollisionInfo> results;
  for (auto fstIt = colliders_.begin(); fstIt != colliders_.end(); ++fstIt)
    for (auto sndIt = std::next(fstIt); sndIt != colliders_.end(); ++sndIt)
      if (fstIt->isDynamic() || sndIt->isDynamic())
        if (auto res = fstIt->collide(*this, *sndIt))
          results.push_back(*res);
  return results;
}

void World::collisionResponce(const std::vector<CollisionInfo>& collisionInfo) {
  for (const auto &collision: collisionInfo) {
    glm::vec2 p = getPoint(collision.point).get(),
              p1 = getPoint(collision.edge.first).get(),
              p2 = getPoint(collision.edge.second).get(),
              n = collision.normal * collision.depth;
    float t = glm::dot(glm::normalize(p2 - p1), p - p1),
          lambda = 1.0f / (t * t + (1.0f - t) * (1.0f - t));
    if (collision.point.getType() == PointIterator::STATIC) {
      if (collision.edge.first.getType()  == PointIterator::STATIC &&
          collision.edge.second.getType() == PointIterator::STATIC)
        continue;
      if (collision.edge.first.getType() == PointIterator::STATIC)
        setPoint(collision.edge.second, p2 - t * lambda * n);
      else if (collision.edge.second.getType() == PointIterator::STATIC)
        setPoint(collision.edge.first, p1 - (1 - t) * lambda * n);
      else {
        setPoint(collision.edge.first, p1 - (1.0f - t) * lambda * n);
        setPoint(collision.edge.second, p2 - t * lambda * n);
      }
    }
    else {
      if (collision.edge.first.getType()  == PointIterator::STATIC &&
          collision.edge.second.getType() == PointIterator::STATIC)
        setPoint(collision.point, p + n);
      else {
        setPoint(collision.point, p + 0.5f * collision.normal * collision.depth);
        if (collision.edge.first.getType() == PointIterator::STATIC)
          setPoint(collision.edge.second, p2 - t * lambda * n);
        else if (collision.edge.second.getType() == PointIterator::STATIC)
          setPoint(collision.edge.first, p1 - (1 - t) * lambda * n);
        else {
          setPoint(collision.edge.first, p1 - 0.5f * (1.0f - t) * lambda * n);
          setPoint(collision.edge.second, p2 - 0.5f * t * lambda * n);
        }
      }
    }
  }
}

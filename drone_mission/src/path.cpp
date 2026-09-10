#include "drone_mission/path.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace drone_mission
{

namespace
{
// Returns unit vector in the direction of the yaw angle (heading of the drone)
Vec2 forward(double heading) { return {std::cos(heading), std::sin(heading)}; }

// Returns unit vector 90deg right of yaw angle (heading of the drone )
Vec2 right(double heading) { return {-std::sin(heading), std::cos(heading)}; }
}  // namespace


// ── PolylinePath ───────────────────────────────────────────────────────────────────────

// Essentially builds a distance-so-far lookup vector for square/triangle
// E.x. [0, 4, 8, 12, 16] where each value represents distance from start
PolylinePath::PolylinePath(std::vector<Vec2> points)
: route_(std::move(points))
{
  if (route_.size() < 2) {
    throw std::invalid_argument("PolylinePath needs at least two points");
  }
  
  cumul_dist_.reserve(route_.size());
  cumul_dist_.push_back(0.0);
  
  for (std::size_t i = 1; i < route_.size(); ++i) {
    const double seg = (route_[i] - route_[i - 1]).norm();
    if (seg <= 0.0) {
      throw std::invalid_argument("PolylinePath has a zero-length segment");
    }
    cumul_dist_.push_back(cumul_dist_.back() + seg);
  }
}

// Figures out which straight line segment of shape we are currently on
std::size_t PolylinePath::segmentIndex(double dist) const
{
  // Find the vertex whose distance from start is strictly bigger than distance covered
  auto next_vertex = std::upper_bound(cumul_dist_.begin(), cumul_dist_.end(), dist); // If dist was 5, this would be 8       
  std::size_t next_vertex_idx = next_vertex - cumul_dist_.begin();                   // If next_vertex was 8, this would be 2

  // The segment is the one that starts at the previous vertex (0-indexed), clamped so dist == perimeter() stays on the last segment.
  return std::min(next_vertex_idx - 1, route_.size() - 2);
}

// Given a distance traveled, returns the actual (x,y) point at that distance
Vec2 PolylinePath::pointAt(double dist) const
{
  dist = std::clamp(dist, 0.0, perimeter());
  const std::size_t seg_idx = segmentIndex(dist);
  
  // Linear interpolation between the segment's two vertices; 
  const double seg_len = cumul_dist_[seg_idx + 1] - cumul_dist_[seg_idx];
  const double seg_frac = (dist - cumul_dist_[seg_idx]) / seg_len;                // seg_frac is the fraction of the segment we have travelled
  return route_[seg_idx] + (route_[seg_idx + 1] - route_[seg_idx]) * seg_frac;    // Find (x,y) of actual position
}

// Given distance along the path, returns direction of travel at that point
// Later multiplied by speed to get velocity vector
Vec2 PolylinePath::tangentAt(double dist) const
{
  dist = std::clamp(dist, 0.0, perimeter());
  const std::size_t seg_idx = segmentIndex(dist);
  const Vec2 dir = route_[seg_idx + 1] - route_[seg_idx]; // Gets raw displacement vector
  return dir * (1.0 / dir.norm());  // Transform into unit vector
}


// ── CirclePath ─────────────────────────────────────────────────────────────────────────

// Set the circle center point and drone's polar angle
CirclePath::CirclePath(Vec2 start, double radius, double heading)
: radius_(radius)
{
  if (radius <= 0.0) {
    throw std::invalid_argument("CirclePath radius must be positive");
  }

  center_ = start + right(heading) * radius;
  // Drone's polar angle is just 90deg left of where we placed the center. 
  theta0_ = heading - M_PI / 2.0;
}

// Given a distance traveled, returns the actual (x,y) point at that distance
Vec2 CirclePath::pointAt(double dist) const
{
  dist = std::clamp(dist, 0.0, perimeter());
  // Travelling dist along the circle sweeps an angle dist / r. The angle grows clockwise
  const double theta = theta0_ + dist / radius_;
  return center_ + Vec2{std::cos(theta), std::sin(theta)} * radius_;
}

// Given distance along the path, returns direction of travel at that point
// Later multiplied by speed to get velocity vector
Vec2 CirclePath::tangentAt(double dist) const
{
  dist = std::clamp(dist, 0.0, perimeter());
  const double theta = theta0_ + dist / radius_;
  // Rotate theta (position direction from center) by 90deg clockwise to get sideways direction
  // because direction of travel (tangent) always perpendicular to line from center to you
  return {-std::sin(theta), std::cos(theta)}; 
}


// ── Path Builders ───────────────────────────────────────────────────────────────────────────

// Build the square flight pattern
std::unique_ptr<Path> makeSquare(Vec2 start, double side, double heading)
{
  if (side <= 0.0) {
    throw std::invalid_argument("square side must be positive");
  }

  // The vertical and horizontal vectors of the square 
  const Vec2 ahead = forward(heading) * side;
  const Vec2 sideways = right(heading) * side;

  // Build vector of points to follow
  return std::make_unique<PolylinePath>(std::vector<Vec2>{start, start + ahead, start + ahead + sideways, start + sideways, start});
}

// Build the triangle flight pattern
std::unique_ptr<Path> makeTriangle(Vec2 start, double side, double heading)
{
  if (side <= 0.0) {
    throw std::invalid_argument("triangle side must be positive");
  }
 
  // Determine apex and second point
  const double height = std::sqrt(side * side - (side/2.0) * (side/2.0));
  const Vec2 apex = start + forward(heading) * (side/2.0) + right(heading) * height;
  const Vec2 second = start + forward(heading) * side;

  // Build vector of points to follow
  return std::make_unique<PolylinePath>(std::vector<Vec2>{start, apex, second, start});
}

// Build the circle flight pattern
std::unique_ptr<Path> makeCircle(Vec2 start, double radius, double heading)
{
  return std::make_unique<CirclePath>(start, radius, heading);
}


// ── PathSampler ────────────────────────────────────────────────────────────────────────

PathSampler::PathSampler(std::shared_ptr<const Path> path, double v_max, double a_max)
: path_(std::move(path)), v_max_(v_max), a_max_(a_max)
{
  if (!path_ || v_max_ <= 0.0 || a_max_ <= 0.0) {
    throw std::invalid_argument("PathSampler needs a path and positive v_max / a_max");
  }
  stops_ = path_->stopDistances();
}

// Given a distance along the path, how fast should the drone be moving
double PathSampler::speedAt(double dist) const
{
  double v = v_max_;
  // Must respect every stop's speed, so find the min speed across all stops
  for (const double stop : stops_) {
    const double breaking_curve = std::sqrt(2.0 * a_max_ * std::fabs(dist - stop));   // Trapezoidal speed: speed at distance from pt cannot exceed sqrt(2 * a_max * d)
    const double stop_speed_limit = breaking_curve + kMinSpeed;                       // The stop's minimum speed
    v = std::min(v, stop_speed_limit);
  }
  return v;
}

// Where the carrot is right now without moving it
PathSample PathSampler::current() const
{
  PathSample sample;
  const double path_perimeter = path_->perimeter();
  
  sample.done = dist_ >= path_perimeter;
  sample.position = path_->pointAt(dist_);
  sample.speed = sample.done ? 0.0 : speedAt(dist_);
  sample.velocity = sample.done ? Vec2{} : path_->tangentAt(dist_) * sample.speed;
  sample.progress = std::clamp(dist_ / path_perimeter, 0.0, 1.0);
  return sample;
}

// Moves the carrot the drone follows
PathSample PathSampler::advance(double dt)
{
  const double path_perimeter = path_->perimeter();
  // Move the carrot along the path at the speed allowed where it currently is
  if (dist_ < path_perimeter && dt > 0.0) {
    dist_ = std::min(path_perimeter, dist_ + speedAt(dist_) * dt);
  }
  return current();
}

}  // namespace drone_mission

#pragma once

#include <cmath>
#include <memory>
#include <vector>

namespace drone_mission
{

// Provides vector math to work with points and directions on the map (x is north and y is east)
struct Vec2
{
  double x{0.0};  
  double y{0.0};

  Vec2 operator+(const Vec2 & o) const { return {x + o.x, y + o.y}; }
  Vec2 operator-(const Vec2 & o) const { return {x - o.x, y - o.y}; }
  Vec2 operator*(double k) const { return {x * k, y * k}; }
  double norm() const { return std::hypot(x, y); }  // Returns the length of the vector
};



// ── Shape Data ───────────────────────────────────────────────────────────────────────

// The shape of a pattern described as a closed line of a given total perimeter()
// Given a distance, class returns a point and direction. 
class Path
{
public:
  virtual ~Path() = default;
  virtual double perimeter() const = 0;                   // Total perimeter of the shape in metres
  virtual Vec2 pointAt(double dist) const = 0;            // Position after travelling dist from the start (clamped to [0, perimeter()])
  virtual Vec2 tangentAt(double dist) const = 0;          // Direction of travel at dist, as a unit vector
  virtual std::vector<double> stopDistances() const = 0;  // Distances where the vehicle should almost be stoped: start, end and sharp corners.

};

// The shape data for square and triangle. Basically a path made of straight segments between consecutive vertices
class PolylinePath : public Path
{
public:
  explicit PolylinePath(std::vector<Vec2> points);
  double perimeter() const override { return cumul_dist_.back(); }
  Vec2 pointAt(double dist) const override;
  Vec2 tangentAt(double dist) const override;
  std::vector<double> stopDistances() const override { return cumul_dist_; }

private:
  std::size_t segmentIndex(double dist) const;  // Given a distance, what segment (edge of the shape) am I on
  std::vector<Vec2> route_;                     // Points to fly to, in travel order (first == last for a closed shape)
  std::vector<double> cumul_dist_;              // Distance along the path from the start to each vertex: [0] == 0, back() == perimeter()
};

// The shape data for circle. Full circle of a given radius; the vehicle starts on the circle heading forward and turns right.
class CirclePath : public Path
{
public:
  CirclePath(Vec2 start, double radius, double heading);
  double perimeter() const override { return 2.0 * M_PI * radius_; }
  Vec2 pointAt(double dist) const override;
  Vec2 tangentAt(double dist) const override;
  std::vector<double> stopDistances() const override { return {0.0, perimeter()}; }

private:
  Vec2 center_;    // One radius to the right of the start point
  double radius_;
  double theta0_;  // Polar angle of the start point around the centre
};



// ── Shape Builders ───────────────────────────────────────────────────────────────────────

// Build a flight pattern anchored at 'start' and oriented by 'heading'. The square and the circle
// set off along the heading and turn right; the triangle sets off on the diagonal to its apex.
std::unique_ptr<Path> makeSquare(Vec2 start, double side, double heading);
std::unique_ptr<Path> makeTriangle(Vec2 start, double side, double heading);
std::unique_ptr<Path> makeCircle(Vec2 start, double radius, double heading);



// ── Path Following ───────────────────────────────────────────────────────────────────────

// Snapshot of where drone should be right now, return by PathSampler each time it advances
struct PathSample
{
  Vec2 position;          // The point on the path to f ly to (fed to PX4 as position setpoint)
  Vec2 velocity;          // Direction * speed (fed to PX4 as velocity)
  double speed{0.0};      // How fast to move at this point
  double progress{0.0};   // Progress in [0, 1]
  bool done{false};       // Reached end of the path?
};

// Turns static Path into a moving carrot over time Speed is limited to sqrt(2 * a_max * distance) around every stop point
class PathSampler
{
public:
  PathSampler(std::shared_ptr<const Path> path, double v_max, double a_max);
  PathSample advance(double dt);          // Advance by dt seconds and return the new sample.
  PathSample current() const;             // Sample at the current distance without advancing.

private:
  double speedAt(double dist) const;
  static constexpr double kMinSpeed = 0.2;  // m/s floor so the profile never stalls
  std::shared_ptr<const Path> path_;
  double v_max_;                            // Cruise speed
  double a_max_;                            // Acceleration used for the ramps into / out of stop points
  std::vector<double> stops_;               // Distances along the path where the speed must dip (start, corners, end)
  double dist_{0.0};                        // Distance the carrot has travelled along the path
};

}  // namespace drone_mission

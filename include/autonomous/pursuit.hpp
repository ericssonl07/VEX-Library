#ifndef VEXLIBRARY_PURE_PURSUIT_HPP
#define VEXLIBRARY_PURE_PURSUIT_HPP

#include <pathing/path.hpp>
#include <vector>
#include <utility>

/**
 * @brief A class that implements the pure pursuit algorithm.
 * 
 * The pure pursuit algorithm is a path tracking algorithm that calculates the steering angle of a robot to follow a path.
 * It is based on the concept of a lookahead point, which is a point on the path that the robot is trying to reach, ahead
 * of the robot's current position by a certain distance.
 * 
 * The algorithm calculates the steering angle based on the distance between the robot and the lookahead point, and the
 * curvature of the path to reach the lookahead point.
 * 
 * By constantly "pursuing" the lookahead point, which stays on the path a fixed distance ahead of the robot, the robot can
 * follow the path accurately and smoothly as it constantly adjusts its steering angle to chase the moving target.
 */
class Pursuit {

    /**
     * @brief The index of the last found point on the path.
     * 
     * This variable allows the implementation of forward-searching the path without repetition and back-tracing. The advantage
     * of forward-searching is that it allows the program to trace a closed path, whereas back-tracing would not (it would find
     * the end point and return that).
     */
    int last_found_idx;

    /**
     * @brief The lookahead distance.
     * 
     * This is the distance between the robot and the lookahead point.
     */
    double lookahead;

    /**
     * @brief The `Path` to follow.
     */
    Path path;

public:

    /**
     * @brief Construct a new `Pursuit` object.
     * 
     * @param path The `Path` object to follow. It is assumed that the `Path` object is in a valid state- this will be true
     * if the `Pursuit` object is constructed by the `Chassis` object. If the Path object is not in a valid state, the behavior
     * of the `Pursuit` object is undefined.
     * @param lookahead_distance The lookahead distance, in units consistent with the rest of the program.
     * This is the distance between the robot and the lookahead point. A higher lookahead distance will result in smoother tracing,
     * but the robot may cut corners and cause unexpected behavior; a lower lookahead distance will result in tighter tracing,
     * but the robot may oscillate violently, causing unstable behavior.
     */
    Pursuit(Path path, double lookahead_distance);

    /**
     * @brief Construct a new `Pursuit` object.
     * 
     * @param x The x-coordinates of the path, in units consistent with the rest of the program.
     * @param y The y-coordinates of the path, in units consistent with the rest of the program.
     * @param num_points The number of points in the path which will be interpolated. This is used to calculate the path.
     * There is a tradeoff: a lower number of points is faster to compute, but may result in a less smooth path;
     * a higher number of points will result in a smoother path, but will require more computation. This overhead
     * can be side-stepped by serializing the path using the `Path::serialize` function with a `.path` file extension.
     * @param lookahead_distance The lookahead distance. This is the distance between the robot and the lookahead point.
     * A higher lookahead distance will result in smoother tracing, but the robot may cut corners and cause unexpected behavior;
     * a lower lookahead distance will result in tighter tracing, but the robot may oscillate violently, causing unstable behavior.
     * The lookahead distance should be in units consistent with the rest of the program.
     */
    Pursuit(std::vector<double> x, std::vector<double> y, int num_points, double lookahead_distance);

    /**
     * @brief Reference to the lookahead distance.
     * 
     * @return The lookahead distance (modifiable reference) for get and set.
     */
    double & lookahead_distance();

    /**
     * @brief Get the target point.
     * 
     * @param x_bot The x-coordinate of the robot, in units consistent with the rest of the program.
     * @param y_bot The y-coordinate of the robot, in units consistent with the rest of the program.
     * 
     * @return The coordinate `(x, y)` with the property that `(x, y)` lies on the path and `sqrt((x - x_bot) ^ 2 + (y - y_bot) ^ 2) = lookahead_distance`.
     */
    Coordinate2D get_target(double x_bot, double y_bot);

    /**
     * @brief Get the relative steering angle.
     * 
     * This function calculates the relative steering angle based on the robot's width, current position, and orientation.
     * 
     * @param x_bot The x-coordinate of the robot, in units consistent with the rest of the program.
     * @param y_bot The y-coordinate of the robot, in units consistent with the rest of the program.
     * @param theta_bot The orientation of the robot, in radians. This value is unbounded (not capped to [0, 2π]), and may be negative.
     * @param width_bot The width of the robot, in units consistent with the rest of the program. This is the distance between the left and right wheels.
     * @return A pair of doubles- (L, R)- representing the left and right steering values. Spinning the motor groups at these proportions will result
     * in the robot turning towards the target point.
     */
    std::pair<double, double> get_relative_steering(double x_bot, double y_bot, double theta_bot, double width_bot);
    
};

#endif // #ifndef VEXLIBRARY_PURE_PURSUIT_HPP
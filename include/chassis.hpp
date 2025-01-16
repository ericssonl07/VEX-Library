#ifndef VEXLIBRARY_CHASSIS_HPP
#define VEXLIBRARY_CHASSIS_HPP

#include <autonomous/motor.hpp>
#include <autonomous/pursuit.hpp>
#include <autonomous/odometry.hpp>
#include <autonomous/pid.hpp>
#include <pathing/path.hpp>
#include <vector>

class Chassis {
public:
    /**
     * @brief The left motor group
    */
    MotorGroup * left;

    /**
     * @brief The right motor group
    */
    MotorGroup * right;

    /**
     * @brief The front-back encoder for odometry
     */
    vex::rotation * front_back_encoder;

    /**
     * @brief The left-right encoder for odometry
     */
    vex::rotation * left_right_encoder;

    /**
     * @brief The inertial sensor for odometry
     */
    vex::inertial * inertial_sensor;

    /**
     * @brief The odometry device to track position
    */
    Odometry odom;

    /**
     * @brief The width of the robot
    */
    double base_width;

    /**
     * @brief The radius of the wheel
    */
    double wheel_radius;

    /**
     * @brief The look-ahead distance
    */
    double pursuit_distance;

    /**
     * @brief The gear multiplier
    */
    double gear_multiplier;
    
    /**
     * @brief The sleep time for the tracking thread, in microseconds
    */
    int thread_sleep;

    /**
     * @brief Construct a new Chassis object
     * @param left The left motor group
     * @param right The right motor group
     * @param front_back_encoder The front-back encoder for odometry
     * @param left_right_encoder The left-right encoder for odometry
     * @param inertial_sensor The inertial sensor for odometry
     * @param base_width The width of the robot
     * @param wheel_radius The radius of the wheel
     * @param pursuit_distance The look-ahead distance
     * @param gear_multiplier The gear multiplier
     * @param kp The proportional gain constant
     * @param ki The integral gain constant
     * @param kd The derivative gain constant
     * @param minimum_output The absolute value of the minimum output of the PID controller
     * @param maximum_output The absolute value of the maximum output of the PID controller
     * @param gamma The integral discount value of the PID controller (earlier values get "forgotten")
     * @param initial_x The initial x-coordinate of the robot
     * @param initial_y The initial y-coordinate of the robot
     * @param initial_rotation The initial rotation of the robot
     * @param thread_sleep The sleep time for the tracking thread, in microseconds
     * @attention Ensure that the distance measurements are in consistent units as odometry.
    */
    Chassis(MotorGroup* left, MotorGroup* right, vex::rotation * front_back_encoder, vex::rotation * left_right_encoder, vex::inertial * inertial_sensor, double base_width, double wheel_radius, double pursuit_distance, double gear_multiplier, double initial_x = 0.0, double initial_y = 0.0, double initial_rotation = 0.0, int thread_sleep = 10);

    /**
     * @brief Follow the path interpolated by the given coordinates
     * @param x The x-coordinates
     * @param y The y-coordinates
     * @param tolerance The distance tolerance for the path (ending condition)
     * @param speed_factor The speed factor for the movement
     * @param num_points The number of points to interpolate
    */
    void follow_path(std::vector<double> x, std::vector<double> y, double tolerance, double speed_factor = 1.0, int num_points = 50);

    /**
     * @brief Follow the given path
     * @param points The points to follow
     * @param tolerance The distance tolerance for the path (ending condition)
     * @param speed_factor The speed factor for the movement
     * @param point_count The number of points to interpolate
    */
    void follow_path(std::vector<std::pair<double, double>> points, double tolerance, double speed_factor = 1.0, int point_count = 50);

    /**
     * @brief Follow the given path
     * @param path The path object to follow
     * @param tolerance The distance tolerance for the path (ending condition)
     * @param speed_factor The speed factor for the movement
    */
    void follow_path(Path path, double tolerance = 5.0, double speed_factor = 1.0);

    /**
     * @brief Turn to the given angle
     * @param angle The angle to turn to, in radians
     * @param tolerance The angle tolerance for the path, in radians (ending condition)
     * @param speed_factor The speed factor for the movement
    */
    void turn_to(double angle, double tolerance = 0.01, double speed_factor = 1.0);

    /**
     * @brief Turn by the given angle
     * @param angle The angle to turn by, in radians
     * @param tolerance The angle tolerance for the path, in radians (ending condition)
     * @param speed_factor The speed factor for the movement
    */
    void turn(double angle, double tolerance = 0.01, double speed_factor = 1.0);

    /**
     * @brief Move the robot by the given distance
     * @param distance The distance to move by
     * @param tolerance The distance tolerance for the path (ending condition)
     * @param speed_factor The speed factor for the movement
     */
    void forward(double distance, double tolerance = 5.0, double speed_factor = 1.0);

    /**
     * @brief Get the x-coordinate of the robot
     * @return The x-coordinate of the robot
     */
    double x();

    /**
     * @brief Get the y-coordinate of the robot
     * @return The y-coordinate of the robot
     */
    double y();

    /**
     * @brief Get the rotation of the robot
     * @return The rotation of the robot
     */
    double rotation();

    /**
     * @brief Basic controller movement- should be threaded
     */
    static void basic_control(void * instance);

};

#endif // #ifndef VEXLIBRARY_CHASSIS_HPP
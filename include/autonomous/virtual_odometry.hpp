#ifndef VEXLIBRARY_VIRTUAL_ODOMETRY_HPP
#define VEXLIBRARY_VIRTUAL_ODOMETRY_HPP

#include <vex.h>
#include <autonomous/motor.hpp>
#include <vector>
#include <cmath>

/**
 * @brief The `VirtualOdometry` class tracks the position and rotation of the robot, in a virtual environment. 
 * A "virtual" environment is a minimal VEX development setup with no physical robot, and only two motors to simulate
 * basic movement.
 * @attention The `VirtualOdometry` class will only function within scope, so make sure it is managed by another object.
 * By default, `VirtualChassis` instances (and all instances of derived classes) manages the `VirtualOdometry` object.
*/
class VirtualOdometry {
    /**
     * @brief The tracking function to update the position and rotation of the robot.
     * @details This function is used by the tracking thread, which is initialized by the odometry object constructor.
     * @attention This function is NOT meant to be called directly.
     * @param o The odometry object to track as a void pointer.
     */
    friend int virtual_track(void * o);

    /**
     * @brief Display function for odometry. Displays the position and rotation of the robot.
     * @details This function is used by the display thread, which is initialized by the odometry object constructor.
     * @attention This function is NOT meant to be called directly.
     * @param o The odometry object as a void*. Immediately cast back to an VirtualOdometry*.
     */
    friend int virtual_display(void * o);

    /**
     * @brief Thread to update the odometry object's position and rotation.
     * @attention This thread is managed by the Odometry object. It does NOT detach; therefore, the Odometry
       object must remain in scope throughout the desired period of coordinate tracking, either through
       management by another object or through the use of a blocking function. By default, the Odometry
       object is managed by the Chassis object (and all derived classes).
    */
    vex::thread tracking;

    /**
     * @brief Thread to display the odometry object's position and rotation.
     * @attention This thread is managed by the Odometry object. It does NOT detach; therefore, the Odometry
       object must remain in scope throughout the desired period of coordinate tracking, either through
       management by another object or through the use of a blocking function. By default, the Odometry
       object is managed by the Chassis object (and all derived classes).
     */
    vex::thread display;

    /**
     * @brief The left motor group.
    */
    MotorGroup * left;

    /**
     * @brief The right motor group.
    */
    MotorGroup * right;

    /**
     * @brief The width of the base.
     * @attention Units must be consistent with wheel_radius.
    */
    double base_width;

    /**
     * @brief The radius of the wheel.
     * @attention Units must be consistent with base_width.
    */
    double wheel_radius;

    /**
     * @brief The gear ratio between the motor and the wheel.
    */
    double gear_multiplier;

    /**
     * @brief The x position of the robot.
    */
    double x_position;

    /**
     * @brief The y position of the robot.
    */
    double y_position;

    /**
     * @brief The rotation of the robot.
     * @attention Units are in radians.
    */
    double rotation_value;

    /**
     * @brief The measure of the left encoders.
    */
    double left_position;

    /**
     * @brief The measure of the right encoders.
    */
    double right_position;

    /**
     * @brief The last measure of the left encoders.
    */
    double last_left_position;

    /**
     * @brief The last measure of the right encoders.
    */
    double last_right_position;

    /**
     * @brief The change in measure of the left encoders.
    */
    double delta_left;

    /**
     * @brief The change in measure of the right encoders.
    */
    double delta_right;

    /**
     * @brief The change in rotation.
     * @attention Units are in radians.
    */
    double delta_rotation;

    /**
     * @brief The radius of the rotation.
    */
    double center_radius;

    /**
     * @brief The local translation in the transformed basis.
    */
    double local_x_translation;

    /**
     * @brief The local rotation in the transformed basis.
     */
    double local_offset;

    /**
     * @brief Get the average value of the left encoders.
     * @return The average value as a double.
    */
    double get_left();

    /**
     * @brief Get the average value of the right encoders.
     * @return The average value as a double.
    */
    double get_right();

    /**
     * @brief The sleep time for the tracking thread, in microseconds.
    */
    int thread_sleep;
public:
    /**
     * @brief The constructor for the VirtualOdometry class. Automatically construct tracking thread.
     * @param left The left motor group.
     * @param right The right motor group.
     * @param base_width The width of the base.
     * @param wheel_radius The radius of the wheel.
     * @param gear_multiplier The multiplier associated with the gear ratio and motor cartridges.
     * @param thread_sleep The sleep time for the tracking thread, in microseconds.
     * @attention All units must be consistent with odometry.
    */
    VirtualOdometry(MotorGroup* left, MotorGroup* right, double base_width, double wheel_radius, double gear_multiplier, int thread_sleep = 10);

    /**
     * @brief Retrieve the x coordinate.
     * @return The x coordinate as a double.
    */
    double x();

    /**
     * @brief Retrieve the y coordinate.
     * @return The y coordinate as a double.
    */
    double y();

    /**
     * @brief Retrieve the rotation.
     * @return The rotation as a double, in radians.
    */
    double rotation();

    /**
     * @brief Set the position and rotation of the robot.
     * @param x The x coordinate.
     * @param y The y coordinate.
     * @param rotation The rotation, in radians.
     */
    void set_pose(double x, double y, double rotation);
};

#endif // #ifndef VEXLIBRARY_VIRTUAL_ODOMETRY_HPP
#ifndef VEXLIBRARY_ODOMETRY_HPP
#define VEXLIBRARY_ODOMETRY_HPP

#include <vex.h>
#include <autonomous/motor.hpp>
#include <vector>
#include <cmath>

/**
 * @brief The `Odometry` class tracks the position and rotation of the robot.
 * @attention The `Odometry` class will only function within scope, so make sure it is managed by another object.
 * By default, `Chassis` instances (and all instances of derived classes) manages the `Odometry` object.
*/
class Odometry {
    /**
     * @brief The tracking function to update the position and rotation of the robot.
     * @details This function is used by the tracking thread, which is initialized by the odometry object constructor.
     * @attention This function is NOT meant to be called directly.
     * @param o The odometry object to track as a void pointer.
     */
    friend int track(void* o);

    /**
     * @brief Display function for odometry. Displays the position and rotation of the robot.
     * @details This function is used by the display thread, which is initialized by the odometry object constructor.
     * @attention This function is NOT meant to be called directly.
     * @param o The odometry object as a void*. Immediately cast back to an VirtualOdometry*.
     */
    friend int display(void* o);

    /**
     * @brief Thread to update the odometry object's position and rotation.
     * @attention This thread is managed by the Odometry object. It does NOT detach; therefore, the Odometry
       object must remain in scope throughout the desired period of coordinate tracking, either through
       management by another object or through the use of a blocking function. By default, the Odometry
       object is managed by the Chassis object (and all derived classes).
    */
    vex::thread tracking;

    /**
     * @brief Display thread for the odometry object- initialized automatically.
     * @attention This thread is managed by the Odometry object. It does NOT detach; therefore, the Odometry
       object must remain in scope throughout the desired period of coordinate tracking, either through
       management by another object or through the use of a blocking function. By default, the Odometry
       object is managed by the Chassis object (and all derived classes).
     */
    vex::thread displaying;

    /**
     * @brief The front-back encoder, which measures motion in the direction parallel to the robot's velocity.
     * Ensure that this encoder is tracking the LEFT side of the robot, otherwise the Odometry logic will fail.
     */
    vex::rotation * front_back_encoder;

    /**
     * @brief The left-right encoder, which measures motion in the direction perpendicular to the robot's velocity.
     * This encoder can track a wheel that is attached anywhere in the two degrees of freedom.
     */
    vex::rotation * left_right_encoder;

    /**
     * @brief The inertial sensor. Ensure that the sensor is mounted in the correct orientation.
     */
    vex::inertial * inertial_sensor;

    /**
     * @brief The base width of the robot. This is the distance between the left and right group.
     */
    double base_width;

    /**
     * @brief The radius of the wheel. This is the distance from the center of the wheel to the edge.
     */
    double wheel_radius;

    /**
     * @brief The x position of the robot. This is the horizontal position of the robot.
     */
    double x_position;

    /**
     * @brief The y position of the robot. This is the vertical position of the robot.
     */
    double y_position;

    /**
     * @brief The rotation of the robot, in radians. This is the orientation of the robot, and is unbounded.
     */
    double rotation_value;

    /**
     * @brief The time in milliseconds to sleep between iterations. This is used to prevent resource exhaustion.
     * @details If the thread sleep is too low, the CPU will be overworked with all resources dedicated to the tracking function,
     * which will cause the program to freeze; other threads will not be able to run. If the thread sleep value is too high,
     * the tracking function will experience a high degree of granularity, which may cause tracking inaccuracies. By default,
     * it is set to 10 milliseconds, which is a good balance between high granularity and resource efficiency.
     */
    int thread_sleep;

public:

    /**
     * @brief Constructor for the odometry class.
     * 
     * @param front_back_encoder The front-back encoder, which measures motion in the direction parallel to the robot's velocity.
     * Ensure that this encoder is tracking the LEFT side of the robot, otherwise the Odometry logic will fail.
     * @param left_right_encoder The left-right encoder, which measures motion in the direction perpendicular to the robot's velocity.
     * This encoder can track a wheel that is attached anywhere in the two degrees of freedom.
     * @param inertial_sensor The inertial sensor. Ensure that the sensor is mounted in the correct orientation.
     * @param base_width The width of the base. This is the distance between the left and right group.
     * @param wheel_radius The radius of the wheel. This is the distance from the center of the wheel to the edge.
     * @param thread_sleep The time in milliseconds to sleep between iterations. This is used to prevent resource exhaustion.
     * @attention Ensure that the measurements are in consistent units across the entire program, or the robot may not behave as expected.
     */
    Odometry(vex::rotation * front_back_encoder, vex::rotation * left_right_encoder, vex::inertial * inertial_sensor, double base_width, double wheel_radius, int thread_sleep);
    
    /**
     * @brief Retrieve the x coordinate, in units consistent with the rest of the program.
     */
    double x();

    /**
     * @brief Retrieve the y coordinate, in units consistent with the rest of the program.
     */
    double y();

    /**
     * @brief Retrieve the rotation, in radians. This value is unbounded (not capped to [0, 2π]), and may be negative.
     */
    double rotation();

    /**
     * @brief Set the pose of the robot.
     * 
     * @param x The x position, in units consistent with the rest of the program.
     * @param y The y position, in units consistent with the rest of the program.
     * @param rotation The rotation, in radians. This value may be any real number, not necessarily in the range [0, 2π)-
     * though it is recommended to keep the value within this range for simplicity and practicality.
     * @attention It is not recommended to call this function during autonomous operation unless the programmer
     * is certain of the robot's position and that the robot will not move during the call. Failure to do so
     * may result in unexpected behavior or tracking that is not consistent with the robot's actual position.
     */
    void set_pose(double x, double y, double rotation);

};

#endif // #ifndef VEXLIBRARY_ODOMETRY_HPP
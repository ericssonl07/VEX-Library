#include <autonomous/virtual_odometry.hpp>

int virtual_track(void* o) {
    // Cast the void pointer to an VirtualOdometry pointer
    VirtualOdometry* odometry = (VirtualOdometry*) o;

    // Reset all positions and variables to zero
    odometry -> left -> resetPosition();
    odometry -> right -> resetPosition();
    odometry -> last_left_position = 0.0;
    odometry -> last_right_position = 0.0;
    odometry -> x_position = 0.0;
    odometry -> y_position = 0.0;
    odometry -> rotation_value = 0.0;

    // Infinite tracking loop
    while (true) {
        // Retrieve the motor values
        odometry -> left_position = odometry ->  get_left();
        odometry -> right_position = odometry -> get_right();

        // Calculate the change in positions
        odometry -> delta_left = odometry -> left_position - odometry -> last_left_position;
        odometry -> delta_right = odometry -> right_position - odometry -> last_right_position;

        // Update the last positions
        odometry -> last_left_position = odometry -> left_position;
        odometry -> last_right_position = odometry -> right_position;

        // Calculate the change in rotation
        odometry -> delta_rotation = (odometry -> delta_right - odometry -> delta_left) / odometry -> base_width;
        // Calculate the local rotation offset
        odometry -> local_offset = odometry -> rotation_value + odometry -> delta_rotation * 0.5;

        // Casework on the change in rotation
        if (odometry -> delta_rotation != 0.0) {
            // Nonzero angle: calculate the center of rotation and the local translation
            odometry -> center_radius = odometry -> delta_left / odometry -> delta_rotation + odometry -> base_width * 0.5;
            odometry -> local_x_translation = sin(odometry -> delta_rotation * 0.5) * odometry -> center_radius * 2.0;
        } else {
            // Zero angle: translation equals the left/right change in positions
            odometry -> local_x_translation = odometry -> delta_left;
        }

        // Update positions based on the changes in positions
        odometry -> x_position += odometry -> local_x_translation * cos(odometry -> local_offset);
        odometry -> y_position += odometry -> local_x_translation * sin(odometry -> local_offset);
        odometry -> rotation_value += odometry -> delta_rotation;

        // Sleep the thread to prevent resource exhaustion
        vex::this_thread::sleep_for(std::chrono::milliseconds(odometry -> thread_sleep));
    }
}

int virtual_display(void* o) {
    VirtualOdometry* odometry = (VirtualOdometry*) o;
    while (true) {
        // printf("(%.5f, %.5f)\n", odometry -> x_position, odometry -> y_position);
        vex::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

double VirtualOdometry::get_left() {
    std::vector<double> pos = left -> position(vex::rotationUnits::deg);
    double sum = 0.0;
    for (double p : pos) {
        sum += p;
    }
    return sum / (double) pos.size() * gear_multiplier;
}

double VirtualOdometry::get_right() {
    std::vector<double> pos = right -> position(vex::rotationUnits::deg);
    double sum = 0.0;
    for (double p : pos) {
        sum += p;
    }
    return sum / (double) pos.size() * gear_multiplier;
}

VirtualOdometry::VirtualOdometry(MotorGroup* left, MotorGroup* right, double base_width, double wheel_radius, double gear_multiplier, int thread_sleep) {
    VirtualOdometry::left = left;
    VirtualOdometry::right = right;
    VirtualOdometry::base_width = base_width;
    VirtualOdometry::wheel_radius = wheel_radius;
    VirtualOdometry::gear_multiplier = gear_multiplier;
    VirtualOdometry::thread_sleep = thread_sleep;
    VirtualOdometry::tracking = vex::thread(virtual_track, (void*) this);
    VirtualOdometry::display = vex::thread(virtual_display, (void*) this);
}

double VirtualOdometry::x() {
    return VirtualOdometry::x_position;
}

double VirtualOdometry::y() {
    return VirtualOdometry::y_position;
}

double VirtualOdometry::rotation() {
    return VirtualOdometry::rotation_value;
}

void VirtualOdometry::set_pose(double x, double y, double rotation) {
    VirtualOdometry::x_position = x;
    VirtualOdometry::y_position = y;
    VirtualOdometry::rotation_value = rotation;
}
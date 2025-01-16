#include <virtual_chassis.hpp>

VirtualChassis::VirtualChassis(MotorGroup* left, MotorGroup* right, 
                 double base_width, double wheel_radius, double pursuit_distance, double gear_multiplier, 
                 double initial_x, double initial_y, double initial_rotation, int thread_sleep): 
                    left(left), right(right),
                    odom(left, right, base_width, wheel_radius, gear_multiplier, thread_sleep),
                    base_width(base_width), wheel_radius(wheel_radius), pursuit_distance(pursuit_distance), gear_multiplier(gear_multiplier), 
                    thread_sleep(thread_sleep) {
    odom.set_pose(initial_x, initial_y, initial_rotation);
    left -> resetPosition();
    right -> resetPosition();
}

void VirtualChassis::follow_path(std::vector<double> x, std::vector<double> y, double tolerance, double speed_factor, int point_count) {
    Path path(x, y, point_count);
    follow_path(path, tolerance, speed_factor);
}

void VirtualChassis::follow_path(std::vector<std::pair<double, double>> points, double tolerance, double speed_factor, int point_count) {
    std::vector<double> x_values;
    std::vector<double> y_values;
    for (auto [x, y] : points) {
        x_values.push_back(x);
        y_values.push_back(y);
    }
    follow_path(x_values, y_values, tolerance, speed_factor, point_count);
}

void VirtualChassis::follow_path(Path path, double tolerance, double speed_factor) {
    const double kp = 0.01;
    const double ki = 0.0;
    const double kd = 0.0;
    const double minimum_output = 0.5;
    const double maximum_output = 100.0;
    const double gamma = 0.999;
    PID linear_pid_controller(kp, ki, kd, minimum_output, maximum_output, gamma);
    Pursuit pursuit_controller(path, pursuit_distance);
    linear_pid_controller.set_target(0);
    double current_distance = path.distance_to_end(odom.x(), odom.y());
    vex::brain brain;
    while (current_distance > tolerance) {
        auto [steering_left, steering_right] = pursuit_controller.get_relative_steering(odom.x(), odom.y(), odom.rotation(), base_width);
        double pid_factor = linear_pid_controller.calculate(-current_distance);
        double raw_left_power = steering_left * pid_factor;
        double raw_right_power = steering_right * pid_factor;
        double normalization_factor = 100.0 / (fabs(raw_left_power) + fabs(raw_right_power));
        double normalized_left_power = raw_left_power * normalization_factor;
        double normalized_right_power = raw_right_power * normalization_factor;
        left -> spin(vex::fwd, normalized_left_power * speed_factor, vex::pct);
        right -> spin(vex::fwd, normalized_right_power * speed_factor, vex::pct);
        current_distance = path.distance_to_end(odom.x(), odom.y());
        vex::this_thread::sleep_for(thread_sleep);
    }
    left -> stop();
    right -> stop();
}

void VirtualChassis::turn_to(double angle, double tolerance, double speed_factor) {
    static auto nearest_coterminal = [] (double rotation, double target) -> double {
        static double pi2 = 3.14159265358979323846 * 2, divpi2 = 1 / pi2;
        return floor((rotation - target + 3.14159265358979323846) * divpi2) * pi2 + target;
    };
    double target_rotation = nearest_coterminal(odom.rotation(), angle);
    double change = target_rotation - odom.rotation();
    turn(change, tolerance, speed_factor);
}

void VirtualChassis::turn(double angle, double tolerance, double speed_factor) {
    double target = odom.rotation() + angle;
    PID pid_controller(45.0, 0.0025, 10.0, 2, 100, 0.9999); // kp, ki, kd, min, max, gamma
    pid_controller.set_target(target);
    while (fabs(odom.rotation() - target) > tolerance) {
        double output = pid_controller.calculate(odom.rotation()) * speed_factor;
        left -> spin(vex::fwd, -output, vex::pct);
        right -> spin(vex::fwd, output, vex::pct);
        vex::this_thread::sleep_for(thread_sleep);
    }
    left -> stop();
    right -> stop();
}

void VirtualChassis::forward(double distance, double tolerance, double speed_factor) {
    double x1 = odom.x();
    double y1 = odom.y();
    double rotation = odom.rotation();
    double x2 = x1 + distance * cos(rotation);
    double y2 = y1 + distance * sin(rotation);
    follow_path({x1, x2}, {y1, y2}, tolerance, speed_factor, 4);
}

double VirtualChassis::x() {
    return odom.x();
}

double VirtualChassis::y() {
    return odom.y();
}

double VirtualChassis::rotation() {
    return odom.rotation();
}

void VirtualChassis::basic_control(void * instance) {
    vex::controller controller;
    VirtualChassis * base = static_cast<VirtualChassis*>(instance);
    while (true) {
        double left_power = controller.Axis3.position() + controller.Axis1.position();
        double right_power = controller.Axis3.position() - controller.Axis1.position();
        base -> left -> spin(vex::fwd, left_power, vex::pct);
        base -> right -> spin(vex::fwd, right_power, vex::pct);
        vex::this_thread::sleep_for(base -> thread_sleep);
    }
}
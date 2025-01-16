#ifndef VEXLIBRARY_VIRTUAL_HIGHSTAKES_CHASSIS_HPP
#define VEXLIBRARY_VIRTUAL_HIGHSTAKES_CHASSIS_HPP

#include <virtual_chassis.hpp>
#include <chrono>

class VirtualHighStakesChassis : public VirtualChassis {
    vex::motor * intake_motor;
    vex::optical * color_sensor;
    vex::color * autonomous_side;
    int intake_for_msec_impl;
    double intake_power_impl;
    static void intake_impl(void * obj) {
        VirtualHighStakesChassis* instance = (VirtualHighStakesChassis*) obj;
        auto function_start = std::chrono::high_resolution_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - function_start).count();
        while (elapsed_ms < instance -> intake_for_msec_impl) {
            printf("Elapsed: %d\n", elapsed_ms);
            instance -> intake_motor -> spin(vex::fwd, instance -> intake_power_impl, vex::pct);
            if (instance -> color_sensor -> isNearObject()) {
                if (instance -> color_sensor -> color() == *(instance -> autonomous_side)) {
                    printf("Braking\n");
                    instance -> intake_motor -> stop(vex::brakeType::brake);
                    vex::this_thread::sleep_for(100);
                }
            }
            elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - function_start).count();
            vex::this_thread::sleep_for(1);
        }
        instance -> intake_motor -> stop(vex::brakeType::brake);
    }
public:
    VirtualHighStakesChassis(MotorGroup * left, MotorGroup * right, 
                 vex::motor * intake, vex::optical * color_sensor, vex::color * team_color,
                 double base_width, double wheel_radius, double pursuit_distance, double gear_multiplier, 
                 double initial_x, double initial_y, double initial_rotation, int thread_sleep): 
                    VirtualChassis(left, right, base_width, wheel_radius, pursuit_distance, gear_multiplier, initial_x, initial_y, initial_rotation, thread_sleep),
                    intake_motor(intake), color_sensor(color_sensor), autonomous_side(team_color) {

    }
    void intake(int for_msec, double power) {
        intake_for_msec_impl = for_msec;
        intake_power_impl = power;
        vex::thread intake_thread(intake_impl, this);
    }
};

#endif // #ifndef VEXLIBRARY_VIRTUAL_HIGHSTAKES_CHASSIS_HPP
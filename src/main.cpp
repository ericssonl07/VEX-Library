/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       ericssonlin                                               */
/*    Created:      12/26/2024, 11:35:15 PM                                   */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include <vex.h>
#include <cmath>
#include <autonomous/motor.hpp>
#include <virtual_chassis.hpp>
#include <highstakes/virtual_highstakes_chassis.hpp>
#include <chassis.hpp>

vex::motor l1(vex::PORT10, true), l2(vex::PORT6, true), l3(vex::PORT2, false);
vex::motor r1(vex::PORT16, false), r2(vex::PORT17, false), r3(vex::PORT19, true);
MotorGroup left_group(&l1, &l2, &l3);
MotorGroup right_group(&r1, &r2, &r3);
const double base_width = 34.606;
const double wheel_radius = 4.1275;
const double gear_multiplier = 0.14654501379148;
const double pursuit_distance = 12.5;
const double initial_x = 0.0;
const double initial_y = 0.0;
const double initial_rotation = 0.0;
const int thread_sleep = 10;
VirtualChassis base(&left_group, &right_group,
                    base_width, wheel_radius, pursuit_distance, gear_multiplier,
                    initial_x, initial_y, initial_rotation,
                    thread_sleep);

#define TILE * 60.96

void motor_testfire();

int main() {
    vexDelay(1);
    // base.turn(3.14159265358979323846 * 10, 0.005, 0.15);
    // base.follow_path({{0 TILE, 0 TILE}, {1 TILE, 0 TILE}, {1 TILE, 1 TILE}, {0 TILE, 1 TILE}, {0 TILE, 0 TILE}}, 5.0, 0.15, 50);
    // base.forward(1 TILE, 0.5, 0.15);
    // VirtualChassis::basic_control(&base);
    // motor_testfire();
    // base.turn(3.14159265358979323846, 0.005, 0.15);
    while (true) { vex::this_thread::sleep_for(1000); }
}


void motor_testfire() {
    // test each motor for 500 ms
    l1.spin(vex::fwd, 100, vex::pct);
    vex::this_thread::sleep_for(500);
    l1.stop();
    l2.spin(vex::fwd, 100, vex::pct);
    vex::this_thread::sleep_for(500);
    l2.stop();
    l3.spin(vex::fwd, 100, vex::pct);
    vex::this_thread::sleep_for(500);
    l3.stop();
    r1.spin(vex::fwd, 100, vex::pct);
    vex::this_thread::sleep_for(500);
    r1.stop();
    r2.spin(vex::fwd, 100, vex::pct);
    vex::this_thread::sleep_for(500);
    r2.stop();
    r3.spin(vex::fwd, 100, vex::pct);
    vex::this_thread::sleep_for(500);
    r3.stop();
}
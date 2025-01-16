#ifndef VEXLIBRARY_PID_HPP
#define VEXLIBRARY_PID_HPP

/**
 * @brief PID controller class
 * 
 * This class implements a general PID controller with output clamping to a given range, and integral discounting
 * to address integral windup. The controller is designed to be used in any feedback loop to control a system.
 */
class PID {

    /**
     * @brief PID controller parameters
     * 
     * These constants of proportionality, integration, and differentiation are used to calculate the output of the PID controller.
     * They should be fine-tuned as hyperparameters of the program.
     * 
     * kp controls the sensitivity of the controller to the error.
     * 
     * ki controls the sensitivity of the controller to an accumulation of error over time.
     * 
     * kd controls the sensitivity of the controller to the rate of change of the error, damping output to prevent overshooting.
     */
    double kp, ki, kd;

    /**
     * @brief The target value for the PID controller to reach, in whatever units the controller is operating in
     */
    double target;

    /**
     * @brief The integral of the error over time
     */
    double integral;

    /**
     * @brief The last error value
     */
    double last_error;

    /**
     * @brief The minimum and maximum output values
     * 
     * The output of the PID controller is clamped to these values so that `|min_output| <= |output| <= |max_output|`.
     */
    double minimum_output, maximum_output;

    /**
     * @brief The gamma value for the integral term
     * 
     * This value acts as a "discount" factor for the integral term,
     * which prevents the integral term from accumulating too much error
     * and has the effect of making the integral term "forget" older errors.
     * It should be a value between 0 and 1 (this is not enforced by the class).
     */
    double gamma;

public:

    /**
     * @brief Construct a new PID controller object
     * 
     * @param kp The proportionality constant- this controls the sensitivity of the controller to the error
     * @param ki The integration constant- this controls the sensitivity of the controller to an accumulation of error over time
     * @param kd The differentiation constant- this controls the sensitivity of the controller to the rate of change of the error, damping output to prevent overshooting
     * @param minimum_output The minimum output value- the output of the PID controller will be clamped to this value so that `|output| >= |min_output|`
     * @param maximum_output The maximum output value- the output of the PID controller will be clamped to this value so that `|output| <= |max_output|`
     * @param gamma The gamma value for the integral term- this value acts as a "discount" factor for the integral term, preventing the integral term
     * from accumulating too much error and making the integral term "forget" older errors- a value between 0 and 1 is recommended. Values close to 0
     * will make the integral term forget older errors faster, while values close to 1 will make the integral term remember older errors for longer.
     * By default, this value is set to 0.9999.
     * @attention The follow conditions are enforced, and throw if not met: `kp, ki, kd >= 0`, `|min_output| <= |max_output|`, `0 <= gamma <= 1`
     */
    PID(double kp, double ki, double kd, double minimum_output, double maximum_output, double gamma = 0.9999);

    /**
     * @brief Set the target value for the PID controller
     * 
     * @param target The target value, in whatever units the controller is operating
     */
    void set_target(double target);

    /**
     * @brief Calculate the output of the PID controller
     * 
     * @param value The current value, in whatever units the controller is operating
     * @return The output of the PID controller function
     */
    double calculate(double value);

    /**
     * @brief Reset the PID controller
     * 
     * This function resets the integral term and the last error value.
     */
    void reset();
    
};

#endif // #ifndef VEXLIBRARY_PID_HPP
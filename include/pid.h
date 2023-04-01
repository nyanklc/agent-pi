#ifndef __PID_H_
#define __PID_H_

class PIDController {
   public:
    PIDController(double p, double i, double d, double goal_value, double last_error = 0, double last_measurement = 0, double lim_min = -100, double lim_max = 100);

    double update(double measurement, double dt);

    void setGoal(double new_goal);

    double getGoal();

   private:
    double kp_;
    double ki_;
    double kd_;

    double lim_min_;
    double lim_max_;

    double goal_;

    double last_error_;
    double last_measurement_;

    double integrator_;
    double differentiator_;

    double out_;

    double tau_;
};

#endif
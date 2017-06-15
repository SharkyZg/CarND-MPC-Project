
# Model description

## 1. Model introduction

In this project I have used the non linear optimizer Ipopt (Interior Point OPTimizer, pronounced eye-pea-Opt) to optimize car steering in the Udacity’s self driving car simulator. I have optimized the car path by imputing preprocessed state parameters from the simulator into Ipopt non linear optimizer.

I have achieved that by collecting the following state parameters from the simulator: desired path, car coordinates, direction and speed. 

## 2. MPC Preprocessing

Before the parameters could be used as non linear optimizer input I needed to preprocess the parameters first to make the optimization problem more straightforward. I have done that by subtracting planned path coordinates with actual car coordinates from the simulator and then converting resulting „shift“ coordinates to car coordinate system (based on the car direction variable „psi“ received form the simulator).


for (int i = 0; i < ptsx.size(); i++) 
{
	double shift_x = ptsx[i] - px;
            double shift_y = ptsy[i] - py;
            
            ptsx[i] = (shift_x*cos(0-psi)-shift_y*sin(0-psi));
            ptsy[i] = (shift_x*sin(0-psi)+shift_y*cos(0-psi));
            
}
        
## 3. Polynomial Fitting

The goal of the optimization problem is to get the resulting ptsx and ptsy coordinates near zero in any point of time, which would result in no discrepancies between a desired path and a path that the car will take in the simulator based on calculated actuators (steering angle and velocity). This is why a cross track error (cte) is calculated based on polynomial evaluation between zero and a polynomial fit that is calculated based on ptsx and ptsy parameters (coeffs):

double cte = polyeval(coeffs, 0);

Same polynomial fit has been used to calculate the steering angle error as well:

double epsi = -atan(coeffs[1]);

## 4. Non linear optimization

Now that we have velocity, error values and polynomial coefficients we can pass them to non linear optimizer to calculate optimal steering angle and optimal acceleration:

auto vars = mpc.Solve(state, coeffs);

### 4.1. Timestep Length and Elapsed Duration (N & dt) parameter tuning

I have calculated the timestep length and elapsed duration based on experimentation, the final result was:

size_t N = 10;
double dt = .1;

I have first picked larger time step (dt=0.5) but the system was not as responsive enough. Smaller time steps required more computing power and they haven't resulted in any significant improvement of vehicle performance in the simulator.

### 4.2. Setting up objectives and constraints with ADvector

With help of ADvector I have calculated the input values that will be used for non linear optimisation. ADVector is used to calculate gradients automatically so that it is not necessary to calculate derivations manually. 

The first depended variable (f[0]) needs to be optimized to zero and it contains cross track error, steering error as well as velocity error (the desired velocity is set to 100). I have given much higher importance to cross track error and steering error to keep the car on the road by multiplying them with 2000. The first parameter f[0] is increased for changes in velocity and steering angle as well, to provide smoother driving experience. 

After the initial depended variable f[0] has been calculated, all the other variables have been added to the ADvector as well. Those are x and y coordinates, steering angle, velocity, cte and steering error for current state and 10 time steps projected in future (based on the N parameter that has been set to 10).

ADvector has been initialized in the optimizer as FG_eval. 

After I have set the FG_eval the next step is to set other inputs for the non linear optimizer.

### 4.3. Setting up initial values for the non linear optimizer and dealing with latency

While importing initial state parameters to non linear optimizer from the simulator I have recalculated the state 100 milliseconds in the future to handle 100 millisecond latency from the simulator:

double x = state[0];
double y = state[1];
double psi = state[2];
double v = state[3];
double cte = state[4];
double epsi = state[5];
  
double f = coeffs[0] + coeffs[1] * x + coeffs[2] * x * x + coeffs[3] * x * x * x ;
double psides = CppAD::atan(3*coeffs[3]*x*x + 2*coeffs[2]*x+coeffs[1]);
  
x = x + v * CppAD::cos(psi)*.1;
y = y + v * CppAD::sin(psi)*.1;

psi = psi - delta_last*Lf*.1;
  
v = v + a_last * .1;

cte = (f-y) + (v*CppAD::sin(epsi)*.1);
epsi = psi - psides;

After the initial state recalculation I have set inner and upper bounds of the independent variables that will be tuned by the optimizer. Variables of a steering angle and an acceleration range will be tuned by the optimizer while initial state variables will stay unchanged (their inner and upper bounds are identical).

### 4.4. Setting up non linear optimizer and returning solution

To start the non linear optimizer it is necessary to set up the options variable as well. The important parameter is max_cpu_time that I have set to 0.5 to avoid too long optimizations that could delay necessary steering response in the simulator.

After all the described parameters have been prepared optimizer can be started to calculate new steering angle and velocity. After a new steering angle and velocity have been calculated result is returned to the simulator and new input from the simulator can be processed to repeat the optimization loop. It is important to note that steering angle returned from solver needs to be converted for the simulator:

msgJson["steering_angle"] = vars[0]/(deg2rad(25)*Lf);

Please find the project video under the following link:

https://youtu.be/70pMUHsAsw8
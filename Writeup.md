[image1]: ./equations.PNG "Equations"

# Model Predictive Control
### Student describes their model in detail. This includes the state, actuators and update equations.

I used the kinematic model as it performs well without being too computationally expensive.

The current state is defined by the vector Fg which contains

* x coordinate
* y coordinate
* Orientation
* Velocity
* Cross Track Error
* Orientation Error

I used the following equations for updating the state

![alt text][image1]

There are two actuators that are used to control the vehicle 

* steering angle
* throttle

### Student discusses the reasoning behind the chosen N (time step length) and dt (elapsed duration between time steps) values.

The time step length and duration between time steps together decide how far into the future does the model predicts and tries to minimize the cost.
I have used a time step length (N) of 10 and the duration(dt) is set to 100 milliseconds which means my model "looks" 1 sec into the future.

### If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.
The waypoints are in global coordinate system. These need to be transformed to vehicle coordinate system before a polynomial is fitted to waypoints.
Here is how the transformation is done

          for (unsigned int i = 0; i < ptsx.size(); i++ ) 
          {
            double shift_x = ptsx[i] - px;
            double shift_y = ptsy[i] - py;

            ptsx[i] = shift_x * cos(0-psi) - shift_y * sin(0-psi);
            ptsy[i] = shift_x * sin(0-psi) + shift_y * cos(0-psi);
          }
          
### Student provides details on how they deal with latency.
A latency of 100ms is introduced to mimic real driving conditions where the car does not actuate the commands instantly.
To deal with latency I use the update equations to calculate the state vector 100 ms into the future and then use that state to calculate the steering angle and throttle.

          // Predicted state of the vehicle after delay.
          double xD = x0 + ( v * cos(psi0) * (Delay/1000.0) );
          double yD = y0 + ( v * sin(psi0) * (Delay/1000.0) );
          double psiD = psi0 - ( v * s_angle * (Delay/1000.0) / 2.67);
          double vD= v + throttle * (Delay/1000.0);
          double cteD = cte0 + ( v * sin(epsi0) * (Delay/1000.0) );
          double epsiD = epsi0 - ( v * atan(coeffs[1]) * (Delay/1000.0) / 2.67 );


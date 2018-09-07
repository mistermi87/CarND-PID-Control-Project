## [Rubric](https://review.udacity.com/#!/rubrics/824/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
## Writeup / README

In this Project the objectiv was to apply a PID-Controller to a car, trying to follow a optimal line on a race course. Furthermore the hyperparameters (P, I, D coefficients) for the controller needed to be found.

### The Hyperparameters:

| Hyper-parameter | Use in calculating the output   | Practical impact on driving car                                                                                                                                                                                                                                                                                                    |
|-----------|--------------------|-----------------------------------|
| P         | factors the direct influence of the error                                    | The most important factor. It is the main reason, the car steers towards the center line again. However, used without the other Hyperparamters it never reaches a stable solution. The car is always overshooting the center line, unless it would reach the center line with a Zero-Error and Zero-Angle. (Constant oscillation)   |
| I         | factors the sum of all past errors (integral)                                | This factor basically puts a penalty on leaving the optimal line for to long. During the rather straight driving part its influence is minuscule but without this factor (or a very low parameter value) the car mostly stays on the outside part of a curve leading to crashes especially at higher speeds                         |
| D         | factors the difference between the last and the current error (differential) | This is the second most important factor. It has a dampening effect on the car so that it is able to reach the optimal line. Additionally it foresees impulses (here the beginning of curves) as a sudden grow in error. However this also means that with growing influence (higher parameter-value) it delays reaching the optimal line.      |

### Finding the right Hyperparameters:

### Throttle calculation
How the car is behaving on the track is also greatly influenced by how it brakes and accelerates.  I decided against using the direct (absolute) error from the optimal line in a PID-controller for this parameter. Instead I only made the current throttle depended on the steering angle (as a [sane] human driver would do). Through this I indirectly also took the error into account, as the steering angle is greater at a greater error.

I added these lines to the main code (`gas_break_pedal` is the input to the throttle):

```cpp
gas_break_pedal=2*(0.5-fabs(steer_value));		// steer_value is output of PID, Range:[-1,1]
if(speed<=1) gas_break_pedal=0.1; 				// no going backwards
if(gas_break_pedal>0.7) gas_break_pedal=0.7; 	// no going full speed
```

### Twiddle

I used the "Twiddle"-Approach for tuning the parameters. For this I introduced a new function for the `PID`-class and several new class parameters:

#### The new parameters:
```cpp
int twiddle_index=0; 				//the index of the current paramter "twiddled" --> p[]=kp,ki,kd
double twiddle_best_err=0;			//the best error found "twiddleing"
double p [3] = {0.165, 0.001, 2.6}; //the Hyperparamters as a list so that the algorithm can iterate through it
double dp [3] ={0.00127093, 0, 0.0141215}; //the current stepsize for testing different parameter values
bool worse=0;						// is activated when the first condition of the Twiddle Algortithm is not met.
int twiddle_interval=1350; 			// how many steps one paramter combination is tested before evaluation an manipulation. 1350 equals aprox. driving one lap.
```
#### The new functions:
##### Total Error calculation:
Since I am trying to implement a "learning while driving" approach, the total error is just a moving average of the last lap's absolute distances to the optimum.
```cpp
void PID::TotalError(double cte) {
    total_error=(1-1.0/(twiddle_interval))*total_error+(1.0/(twiddle_interval))*fabs(cte);
}
```

##### The Twiddle algorithm:

> The parameter `run` is a counter of the steps. By using `run%twiddle_interval==0` the algorithm is applied every 1350 steps.

```cpp
void PID::twiddle(int run){

    if(run%twiddle_interval==0){
        if(worse==0){
            if(twiddle_index==3) twiddle_index=0;
            if(twiddle_index==1) twiddle_index=2; //this line lets the algortihm skip the second parameter Ki, more to that later

            if(total_error < twiddle_best_err){
                twiddle_best_err = total_error;
                dp[twiddle_index] *= 1.1;
                p[twiddle_index] += dp[twiddle_index];
                Kp=p[0];
                Ki=p[1];
                Kd=p[2];
                worse=0;
                twiddle_index+=1;
            }else{
                p[twiddle_index] -= 2 * dp[twiddle_index];
                Kp=p[0];
                Ki=p[1];
                Kd=p[2];
                worse=1;
            }

        }else{ //-->if worse==1
            if(total_error < twiddle_best_err){
                twiddle_best_err = total_error;
                dp[twiddle_index] *= 1.1;
                p[twiddle_index] += dp[twiddle_index];
                Kp=p[0];
                Ki=p[1];
                Kd=p[2];
                worse=0;
                twiddle_index+=1;
            }else{
                p[twiddle_index] += dp[twiddle_index];
                dp[twiddle_index] *= 0.9;
                Kp=p[0];
                Ki=p[1];
                Kd=p[2];
                worse=0;
                twiddle_index+=1;
                }
    }
    cout<< "---Iteration: " << run << " TI: " << twiddle_index <<" P:" << p[0] << " "<< p[1] << " "<< p[2] << " dp: "<< dp[0] << " "<< dp[1] << " "<< dp[2]<<endl;
    }
}
```

#### Starting values
In order to implement this approach the car has to make it around the track at least once. I tried around but never reached something that was robust enough that it could be near an absolute minimum total-error. I suspect for the solution to have many local minima, which the Twiddle algorithm does NOT ignore. On *Slack* I saw some students use Sebastian Thruns solution to the previous problem as a starting point. It appeared reasonably robust so I started from there.

The values are: 
```
Kp = 0.2
Ki = 0.004
Kd = 3.0
```

The performance of the hyperparamters is highly depended on the calculation of the speed (high speed --> overshooting), which means that my final result will be different from the initial values and there is not one single best solution when comparing my results to results of fellow students.

#### Issues:  The Ki-coefficient

The Twiddle algorithm always aimed towards a Ki-Hyperparameter of 0.  The nature of the algorithm even suggests negative values, leading to crashes mostly in the only right curve of the track. Because of that, I manually chose a low, non-zero value of 0.002 for the Hyperparamter and applied further twiddling only on the Kp- and the Kd-Hyperparameters.

### Final result:

After around  60 laps the final values are: 
```
Kp = 0.16
Ki = 0.002
Kd = 2.52
```
The car drives over the red and white side markers in the last curve but finishes at least 12 consecutive laps without crash (I stopped after that). This swerving can be diminished by reducing the maximum throttle, thus the overall speed. Currently the average speed is around 50mph with a top speed of ~65mph.
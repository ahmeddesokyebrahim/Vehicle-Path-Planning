# **Vehicle Path Planning**
---

**In this project, I have worked a path planning algorithm that safely navigates the car through a highway in udacity simulator.**

## Project Introduction
---
The goal of this project is to navigate a car around a simulated highway scenario, including traffic and given waypoint, telemetry, and sensor fusion data. The car must not violate a set of motion constraints, namely maximum velocity, maximum acceleration, and maximum jerk, while also avoiding collisions with other vehicles, keeping to within a highway lane (aside from short periods of time while changing lanes), and changing lanes when doing so is necessary to maintain a speed near the posted speed limit.

## Path Planning Algorithm Discussion
---
In the following, I will be discussing the algorithm details used through the project

### What is Path Planning problem ? 
---
Path Planning (or robot motion planning problem) can be defined as the final sequence of feasible movements in a configuration space that is moved by the robot from a start configuration to an end configuration without hitting any obstacles.

### Path Planning problem consists of main three parts:
---
* **Prediction**
The is the module that takes as input the map of the world and data from sensor fusion, and generate as output some predicitions of the future state of all the vehicles and other moving objects in the vicinity of our vehicle
* **Behavior Planning**
The behavior planning is responsible for providing guidance to the trajectory planner about what sorts of maneuvers they should plan trajectories for. Behavior module takes input from sensor fusion module (as well as the localization module).
* **Trajectory Gereration**
A trajectory is not just a curve that the car can follow but also it is a time sequence in which we say how fast the car should go.
Trajectory generation module takes input from both prediction and localization modules, and provides trajectories directly to the motion controller.

>In the following, I will be reflecting these three parts with the algorithms, with some code snippts to show how it works :)

#### Prediction
As mentioned above, we are using here the sensor fusion data to build the awarness of the surronding objects. The simulator is reporting this kind of list of all the other cars on the road . All other cars have some `s` , `d` , `vx`, `vy` values ..
We really want to use that to figure out where cars are, how fast they are going and how should we avoid collision to them ...
So, if we want to avoid hitting the car in front of us, we need to go through the sensor fusion list, to check if the car is in our lane or not and then to check how close this car to us and if it is so close ..

That is exactly shown in the next code snippet. Here we are using flags to check, it there is car on left, right, ahead, and ahead and too close.
What helps here is using the `d` value of the objects from the `sensor_fusion` vector. This `d` value as a lateral distance indicates where the vehicles are.
Knowing that we have 3 lanes, each lane is 4 meters, the most left is lane # 0, the center # 1 , and the most right is # 2, we can build this code easily

```cplusplus
for(int i = 0 ; i < sensor_fusion.size() ; i++)
{
	/*Car is in my lane*/
	float d = sensor_fusion[i][6];

	char car_lane = -1;

	if(d > 0 && d < 4)
	{
		car_lane = 0;
	}
	else if(d > 4 && d < 8)
	{
		car_lane = 1;
	}
	else if(d > 8 && d < 12)
	{
		car_lane = 2;
	}
	if(car_lane < 0)
  	{
  		continue;
  	}
```

Now, as the `sensor_fusion` vector also includes other data about these vehicle, we can use the vehicles `vx`, `vy`, to try to predict where these vehicle to be in the future.

*Remember: we are still in the prediciton / sensor fusion loop that began the previous code snippet*

```cplusplus
double vx = sensor_fusion[i][3];
double vy = sensor_fusion[i][4];
/*Calculate the velocity magnitude .. */
double check_speed = sqrt(vx*vx + vy*vy);
double check_car_s = sensor_fusion[i][5];
/*Try to predict where the car will be in the future*/
check_car_s += ((double) prev_size*0.02*check_speed);
```

So, here it comes the most interesting part in prediction. Do you remember those flags we talked about above ? Yeah we did not use them till now. Here, it comes the moment to check if we have a vehicle on the right, on the left, ahead of us, too close. After that in the behavior part, based on each flag, there will be a specific ation to take and a definite logic to follow .. 
It is worth to mention that here we are checking a gap of 30 meters. In case there is a car ahead of us 20 meter or less, it is too close then.

Here is the code snippet
*Remember: we are still in the prediciton / sensor fusion loop that began the previous code snippet*
```cplusplus
if(car_lane == lane)
{
	/*Check if s values are greater than mine and s gap*/
	if( (check_car_s > car_s) && ((check_car_s-car_s) < 30) )
	{
		car_ahead = true;
		if( (check_car_s > car_s) && ((check_car_s-car_s) < 20) )
		{
			car_ahead_and_too_close = true;
		}
	}
}
else if (car_lane < lane)
{
	if( (car_s + 30 > check_car_s) && (car_s - 30 < check_car_s) )
	{
		car_left = true;
	}
}
else if(car_lane > lane)
{
	if( (car_s + 30 > check_car_s) && (car_s - 30 < check_car_s))
	{
		car_right = true;
	}
}
```

#### Behavior Planning
Here, in the behavior planning part, we depend on the information we get from the prediction part. Here we can say it the heart of our decision making algrithm. Here, we can say you find our finite state machine, where the vehicle decides either to be in the same lane and activates follow mode, or change lanes, .. so on. Here also, you can touch the cost functions, based on we choose our best trajectory to have. Bear with me and you find everything easy and crystal clear ;)

For sure, safety comes first. That is why if we found a slowing vehcile in front of our ego vehicle, and we want to change lanes, we will never take lane change in the existance of other approaching vehicle in adjacent lane. If the lane is free to change, yes then we change lane. Otherwise, we deaccelerate, and follow the slow vehicle infront of us. Other point that enhances safety, in any case, we found ourselves very close to the vehicle ahead, deaccelerate more.
All this deacceleration is surely inside the acceptabe boudries (< 10 m/s^2)


```cplusplus
/*If there is a vehicle in front of us*/
if(car_ahead)
{
	/*And no vehicle to the left, and we have left lane available*/
	if( (!car_left) && (lane > 0) )
	{
		/*Change lane to the left*/
		lane--;
	}
	/* No vehicle to the right, and we have right lane available*/
	else if( (!car_right) && (lane != 2) )
{
		/*Change lane to the right*/
		lane++;
}
	else
	{
		/*Follow mode*/
		ref_vel -= 0.224;
		/*If we are too close .. take care !*/
		if(car_ahead_and_too_close)
		{
			ref_vel -= 0.3936;
		}
	}
}
```

So, if there is no vehicle in front, it is free to drive mode here ..
We accelerate within the acceptable jerk boundries, unill we reach our reference speed. Also, we try to be in the center lane.
```cplusplus
/*Free Drive Mode*/
else
{
	/*If we are not in the middle lane*/
		if(lane != 1)
	{
			/*try to be in the middle lane*/
		if( ( (lane == 0) && (!car_right) ) || ( (lane == 2) && (!car_left) ) )
		{
			lane = 1;
		}
	}

		/*Try to reach your reference velocity*/
if(ref_vel < 49.5)
	{
		ref_vel += 0.224;
	}
}
```
#### Trajectory Generation
Here I would prefer to explain this part in set of steps, each step will be followed by its code snippet as possible .. as follows:

1. For trajectory generation, we firstly creat a list of widely spaced (x,y) waypoints, evenly spaced at 30m. Later we will interpolate these waypoints with a spline and fill it in with more points that control speed. Here i will use `ptsx` and `ptsy` vectors to store the next x and y wayspoints.
2. Here I have two sub-steps
  i. We will reference the starting point as where the car is ..Or at the previous paths end point. 
  ii. We need to check what the previous path size was. If it is either going to be pretty close to empty Or it is going to have some points that I can take advantage of .. 
```cplusplus
double ref_x = car_x;
double ref_y = car_y;
double ref_yaw = car_yaw;

/*Now let's check what the previous path size was.
 * It is either going to be pretty close to emply
 * Or it is going to have some points that I can take advantage of .. */

/*If it is pretty empty, I will use tha car state*/
if(prev_size < 2)
{
	/*Use two points that make the path tangent to the car*/
	double prev_car_x = car_x - cos(car_yaw);
	double prev_car_y = car_y - sin(car_yaw);

	ptsx.push_back(prev_car_x);
	ptsx.push_back(car_x);

	ptsy.push_back(prev_car_y);
	ptsy.push_back(car_y);
}
/*Use previous path's end points as starting reference*/
else
{
	ref_x = previous_path_x[prev_size - 1];
	ref_y = previous_path_y[prev_size - 1];

	double ref_x_prev = previous_path_x[prev_size - 2];
	double ref_y_prev = previous_path_y[prev_size - 2];
	ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

	/*Use two points that make the path tangent to the car*/
	ptsx.push_back(ref_x_prev);
	ptsx.push_back(ref_x);

	ptsy.push_back(ref_y_prev);
	ptsy.push_back(ref_y);
}
```
3. Now that is my starting reference. Instead of just looking at one distance increment, we are looking basically 30, 60, 90. Instead of looping through the and creating 50 of these, we are just creating 3 of them. So, I am going to be pushing more three points but instead of just them being 0.5 meter spaced, they are all the way to 30 ..
```cplusplus
vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
```

4. So after adding the following points, we have:
the two previous points, the location of the car in 30 meters, and then in 60 meters, and then in 90 meters
We have here 5 points ... 
```cplusplus
ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);

ptsy.push_back(next_wp0[1]);
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);
```

5. Now we are doing a transformation to this local car's coordinates
```cplusplus
for(int i = 0 ; i < ptsx.size() ; i++)
{
	/*Shift car angle to 0 degree*/
	double shift_x = ptsx[i] - ref_x;
	double shift_y = ptsy[i] - ref_y;
	ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
	ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
}
```

6. So, I define a spline `s`. Create a spline . Then I simply set some points x , y to that spline.
```cplusplus
tk::spline s;
s.set_points(ptsx,ptsy);
```
More information about spline can be found [here](https://kluge.in-chemnitz.de/opensource/spline/)

7. Here is I am actually building what we were working with ..  Next x values and next y values. These are the actual points of the path planner is going to be using 
```cplusplus
vector<double> next_x_vals;
vector<double> next_y_vals;
```

8. Now we need to build the next future path in .. 
  i. Start will all of the previous path points from last time
```cplusplus
for(int i = 0 ; i < previous_path_x.size() ; i++)
{
	/*Add previous path points to your planner ..
	 * This kinda help in the transition
	 * So you make sure that instead of recreating the path from scratch everytime,
	 * we are here adding some points to it.*/
	next_x_vals.push_back(previous_path_x[i]);
	next_y_vals.push_back(previous_path_y[i]);
}
```

  ii. Here, we are figuring the spacing of adding the points of spline. We calculate how to break up spline points so that we travel at our desired `ref_vel`. We are getting the y-value by simply using spline. Then we need to fill the rest of our path planner after filling it with previous points. Here we will always output 50 points
  
```cplusplus
double target_x = 30.0;
double target_y = s(target_x);
double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
double x_add_on = 0;

for(int i = 1 ; i<= 50 - previous_path_x.size() ; i++)
{
	double N = (target_dist/(0.02*ref_vel/2.24));
	double x_point = x_add_on+(target_x)/N;
	double y_point = s(x_point);

	x_add_on = x_point;

	double x_ref = x_point;
	double y_ref = y_point;

	/*rotate back to normal after rotating it earlier*/
	x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
	y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

	x_point += ref_x;
	y_point += ref_y;

	next_x_vals.push_back(x_point);
	next_y_vals.push_back(y_point);
}
```

9. Then we send the trajectory to the simulator
```cplusplus
json msgJson;
msgJson["next_x"] = next_x_vals;
msgJson["next_y"] = next_y_vals;

auto msg = "42[\"control\","+ msgJson.dump()+"]";

//this_thread::sleep_for(chrono::milliseconds(1000));
ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
```
## Environment:
---
* Ubuntu 16.04 LTS
* Udacity Self-Driving Car Nano-Degree Term2 Simulator
* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4

## Running the Code
---
This project involves the Term 3 Simulator for path planning which can be downloaded from the project repo from [here](https://github.com/udacity/CarND-Path-Planning-Project)

Our main program can be built and ran by doing the following from my project repo top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./path_planning

## Conclusion
---
  * Path planning is one of the important part for self-driving cars. It is one of the hard part, if it is not the hardest
  * Our path planning algorithm here is working find in the highway for at least one round in the simulator (~ minutes) without collision, jerks, or leaving the lane.
  * [Here](https://youtu.be/ACGy2LHYZqo) you can find a demo video for the Path Planning project.


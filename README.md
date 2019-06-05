# CarND-Path-Planning-Project

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.


## Simulator Inputs

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Workflow

Path planning consist of  mainly three topics :
  1. Prediction
  2. Behaviour planning
  3. Trajectory generation.

### 1. Prediction

The predition will help to estimate what happens to all nearby objects in future. like whether there is a possibility for a lane change of adjecent vechicle which helps the ego vechicle to plan for next.

Actual prediction module gives the possible trajectories from the current timeline to the future of each vehicle. In this highway exmaple, we will have only one possible trajectory for each vehicle and we are using simple approach for predictio here. In complex situation we may need to use model, data, or hybrid approach for prediction module.

Current assumptions:
   a) 3 lanes, each of 4 meter wide.(In actual scenarion, number of lanes, distance between the lanes, and total lanes distance can be detected using computer vision technologies).
   b) Following parameter set to true :
         
         bool car_ahead = false;
         bool car_left = false;
         bool car_right = false;
         
All adjuscent car's position is assumed using following if condition:

      if(d > 0 && d < 4) {
          check_car_lane = 0;
      } else if(d > 4 && d < 8) {
          check_car_lane = 1;
      } else if(d > 8 and d < 12) {
          check_car_lane = 2;
      } 

Where ["d"] is car's d position in frenet coordinates

The bool parameters defined above is descided with respect to the ego vechile using :

   if(check_car_lane == lane) {
    //A vehicle is on the same line and check the car is in front of the ego car
    car_ahead |= check_car_s > car_s && (check_car_s - car_s) < 30;

   } else if((check_car_lane - lane) == -1) {
    //A vehicle is on the left lane and check that is in 30 meter range
    car_left |= (car_s+30) > check_car_s  && (car_s-30) < check_car_s;

   } else if((check_car_lane - lane) == 1) {
    //A vehicle is on the right lane and check that is in 30 meter range
    car_right |= (car_s+30) > check_car_s  && (car_s-30) < check_car_s;

   }

["car_s"] is provided by the simulator and where the vehicle will be in future

### 2.  Behaviour planning

This determines how the ego vechile should react during situations like lane change, traffic, at curves, while accelerating, at highways and urban roads etc.
Here we are considering only lane change or reduce speed based on the obstacles and we don't need to consider of the cost function for the time being.

When Prediction set three flags(car_ahead, car_left, and car_right) according to the sensor fusion data,the behavior planner checks for the car_ahed flag set to true. Accordingly the car ego vechicle will descide whether to take left/right turns accelerate/deccelerate.

     if(car_ahead) {
       if(!car_left && lane > 0) {
           lane--;
       } else if(!car_right && lane !=2) {
           lane++;
       } else if(!car_left && lane !=2) {
           lane++;
       }else {
           ref_vel -= speed_diff;
       }
    } else if(ref_vel < max_accel){
    ref_vel += speed_diff;
    }

### 3. Trajectory generation

Based on the output from behaviour planner the best trajectory that fits will be exicuted.
Trajectory generation code strats with finding any previous points.

     int prev_size = previous_path_x.size();

we keep the refen     int prev_size = previous_path_x.size();ce x,y and yaw points.

     double ref_x = car_x;
     double ref_y = car_y;
     double ref_yaw = deg2rad(car_yaw);

Checking any previous points left and it is almost empty, then we use current car's point to find the previous point and add them to the list.


    if ( prev_size < 2 ) {
    //Use two points thats makes path tangent to the car
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
    } 

If there is already previous points, then we just add previous two points. Also we are defininf last know previous point to reference x and y.

     //Redefine the reference point to previous point
     ref_x = previous_path_x[prev_size - 1];
     ref_y = previous_path_y[prev_size - 1];
     double ref_x_prev = previous_path_x[prev_size - 2];
     double ref_y_prev = previous_path_y[prev_size - 2];
     ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
     
     ptsx.push_back(ref_x_prev);
     ptsx.push_back(ref_x);
     
     ptsy.push_back(ref_y_prev);
     ptsy.push_back(ref_y);
     
Now we need to add 3 future points to ptsx, psy vecotrs. as car_s is frenet and we need to conver to the global x,y coordinates using getXY function. In total ptsx, ptsy has got 5 points in total each.

    vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);
    
    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

For trajectory generation, we are using spline instead of polynomial trajectory generation. One of the reason is it is simple to use and requiring no dependencies. once we intialise the spline with ptsx and ptsy.

    tk::spline s;
    s.set_points(ptsx, ptsy);

Then we add all previous points to next_x_vals and next_y_vals as it going to be the final control values pass it to the simulator and it will helps to get a smooth transition to the new points that we calculate later.

    vector<double> next_x_vals;
    vector<double> next_y_vals;
    //For the smooth transition, we are adding previous path points
    for ( int i = 0; i < prev_size; i++ ) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

Now we need to find the all spline points till the horizon(say 30m) value so that spacing the way that ego car can travel at desired speed. Remeber the speed of the car is depend on the spacing between the points. If we know x point(ie 30m in this case), spline will be able to get us correponding spline y points.

    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);

We can calculate the spline points from start to horizon y points by using the formula mentioned in the picture. Our number of points has to be calculated is 50. We already added remaining previous points to the vectors next_x_vals and next_y_vals. We need to get the spline points of 50-previous_points.

              
    for( int i = 1; i < 50 - prev_size; i++ ) {
      double N = target_dist/(0.02*ref_vel/2.24);
      double x_point = x_add_on + target_x/N;
      double y_point = s(x_point);
   
      x_add_on = x_point;

      double x_ref = x_point;
      double y_ref = y_point;

      //Rotate back to normal after rotating it earlier
      x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
      y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

      x_point += ref_x;
      y_point += ref_y;

      next_x_vals.push_back(x_point);
      next_y_vals.push_back(y_point);
      }
      
next_x_vals and next_y_vals now have all 50 points which consist of 3 future points and 47 previous points.


## Results

The result obtained after running the code in simulator
  
![Result](/Results/pathplanning.png)
 


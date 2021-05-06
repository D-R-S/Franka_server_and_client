// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>

#include "examples_common.h"


/**
 * @example generate_cartesian_pose_motion.cpp
 * An example showing how to generate a Cartesian motion.
 * Dan:
 * Original implementation moves alongside a circle (removed)
 * adapted to take the current position as starting postion and move in a linear way to a goal postion (in EE space, 3D - only pos)
 * now works over network using _server.py and _client.py  

 * TODO switch between EEs? one between finger for grasp (exists) and one for cam lense ... (needs to be impl.)

 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */



int main(int argc, char** argv) {
  
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << " goal pose (12 values, (xyz, column major R) in double, optional)" << std::endl;
    return -1;
  }

  try { 
    franka::Robot robot(argv[1]); // instantiate robot
    franka::Gripper gripper(argv[1]); // and gripper
    franka::RobotState robot_state; // define the robot state object
    std::cout << " RECOVERING " << std::endl;
    robot.automaticErrorRecovery();

    // construct variables
    // reach
    std::array<double, 7> q_init;
    std::array<double, 16> p_init;
    std::array<double, 16> initial_pose; // 4x4 T matrix
    std::array<double, 16> goal_pose; 
    int res = atof(argv[14]); // reset pose by joint motion?! (0 - dont,1 - above table/dropoff, 2 - above table corner, 3 - shaft insert pose, 4 only fingers)
    // grasp
    int should_grasp = atof(argv[15]); // 0 - no change , 1 - finger @ width, 2 - close on object, 3 - release
    double grasping_width = atof(argv[16]);
    double theta = atof(argv[17]);
    // force feedback
    bool insert_ = atof(argv[18]);
    double mt = atof(argv[19]);
    double maxtime = 5.0; 
    maxtime = mt;
    double fl = atof(argv[20]);
    double force_limit = 15.0;
    force_limit = fl;

    // end effektor
    int the_ee = atof(argv[21]);
    

    
//    double points_array[202][2];
//    if (insert_ == 1) {    
//      std::ifstream fp("/home/franka1/franka/libfranka/data/points_array/points_array.txt");
//      for(int row = 0; row < 202; row++) {
//        for(int col = 0; col < 2; col++) {
//          fp >> points_array[row][col];         
//        }
//      }
//    }
//    // std::cout << points_array[201][0] << points_array[201][1] << std::endl; // test the array

    // fill with standard values
    goal_pose[12] = 0.4; // Tmatrix pose last column
    goal_pose[13] = 0.0;
    goal_pose[14] = 0.055; 
    goal_pose[15] = 1.0; // Tmatrix last row (0,0,0,1)
    goal_pose[0] = 0.999987; // Tmatrix ori 1st column
    goal_pose[1] = 0.000627365;
    goal_pose[2] = -0.00232607;
    goal_pose[3] = 0;  // Tmatrix last row
    goal_pose[4] = 0.000625041; // Tmatrix ori 2nd column
    goal_pose[5] = -0.99999;
    goal_pose[6] = -0.000999452;
    goal_pose[7] = 0; // Tmatrix last row
    goal_pose[8] = -0.00232671; // Tmatrix ori 3rd column
    goal_pose[9] = 0.000998005;
    goal_pose[10] = -0.999997;
    goal_pose[11] = 0; // Tmatrix last row
    if ((argc >= 5)) { 
        //std::cout << "a goal was entered" << std::endl;
        // overwrite with goal received from fcn call (from _server, _client.py)
        goal_pose[12] = atof(argv[2]);  // position; atof() is  string (char) to double
    	goal_pose[13] = atof(argv[3]);
   	goal_pose[14] = atof(argv[4]); 
        goal_pose[0] = atof(argv[5]); // orientation 1st column
    	goal_pose[1] = atof(argv[6]);
   	goal_pose[2] = atof(argv[7]);
        goal_pose[4] = atof(argv[8]); // orientation 2nd column
    	goal_pose[5] = atof(argv[9]);
   	goal_pose[6] = atof(argv[10]);
        goal_pose[8] = atof(argv[11]); // orientation 3rd column
    	goal_pose[9] = atof(argv[12]);
   	goal_pose[10] = atof(argv[13]);
        
    };
    //std::cout << goal_pose << std::endl;
    //std::cin.ignore();

    setDefaultBehavior(robot); // set collison


    // set end effektor NE is between the two fingers (between the top screws)
    std::array<double, 16> NE_T_EE;
    if (the_ee == 0) {
       // regular!  -> move to: between the bottom screws!
       NE_T_EE = {{1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0035,1.0}};  // T(NE->EE) matrix in column major form! 
    }
    if (the_ee == 1) {
       // between the screws and with -5 mm for shaft insertion mode (counteracting stuff)
      NE_T_EE = {{1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,-0.005,0.0,0.0035,1.0}};  // T(NE->EE) matrix in column major form! 
    }
    if (the_ee == 2) {
      // at the camera reference point (at best...)
      NE_T_EE = {{1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0605 - 0.0015,-0.0315 + 0.00075,0.0035 - 0.053,1.0}};
    }
    std::cout << " SETTING THE END EFFEKTOR TO " << NE_T_EE[12] << " "  << NE_T_EE[13] << " " << NE_T_EE[14] << std::endl;
    robot.setEE(NE_T_EE);
    // load current robot state for adapting inital pos to current state
    size_t count = 0;
    robot.read([&count, &q_init, &p_init](const franka::RobotState& robot_state) {
      q_init = robot_state.q; // joint positions
      p_init = robot_state.O_T_EE_c; // ee pos and ori
      return count++ < 1;
    });
    std::cout << "Joint Pose: " 
                  << q_init[0] << " " << q_init[1] << " " << q_init[2] << " " << q_init[3] << " " 
                  << q_init[4] << " " << q_init[5] << " " << q_init[6] << " " << std::endl;


    //////////////////////////////
    //  PRE CONTROL LOOP - JOINT CONTROL (Reset!)
    // Move the robot to a suitable joint configuration
    std::array<double, 7> q_goal;

    if (res == 1) {q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4 + theta}};} // res = 1, above table dropoff
    if (res == 2) {q_goal = {{M_PI_2, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, 3*M_PI/4}};} // res = 2, above table pickup
    if (res == 4) {q_goal = {{0, 0, 0, -3 * M_PI_4, 0, M_PI_4, -M_PI+M_PI_4}};} // res = 4, shaft insert pose (back)  
    if (res == 5) {q_goal = {{-0.63337, -0.05353, -0.06651, -2.6061, 1.2050, 1.3371, -1.80398}};} // res = 5, shaft insert pose2 (front)
    std::cout << "Initial Orientation and position (Matrix): " << std::endl
              << p_init[0] << " " << p_init[4] << " " << p_init[8] << " " << p_init[12] << std::endl
              << p_init[1] << " " << p_init[5] << " " << p_init[9] << " " << p_init[13] << std::endl
              << p_init[2] << " " << p_init[6] << " " << p_init[10] << " " << p_init[14] << std::endl;
 
    if (res == 1 or res == 2 or res == 4 or res == 5) {
        std::cout << " Resetting / Preparing with joint motion! Reset=" << res << std::endl;
        MotionGenerator motion_generator(0.5, q_goal);
        robot.control(motion_generator);  
        std::cout << "Ready for cartesian motion!" << std::endl;   
    }    


    ////////////////////////////////////////////
    //  Cartesian Control
    //  Get Initial Positions
    count = 0; // read again
    robot.read([&count, &q_init, &p_init](const franka::RobotState& robot_state) {
      q_init = robot_state.q; // joint positions
      p_init = robot_state.O_T_EE_c; // ee pos and ori
      return count++ < 1;
    });
 
    // NON REALTIME COMMANDS
    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
   


    // MAIN CONTROL LOOP - CARTESIAN (EE) CONTROL
    // control loop params
    initial_pose = p_init;
    double time = 0.0;
  
    // max distance to goal
    double max_dist = std::sqrt(std::pow((goal_pose[12] - p_init[12]),2)+std::pow((goal_pose[13] - p_init[13]),2)+std::pow((goal_pose[14] - p_init[14]),2));
    double max_ori_dist = 
          std::sqrt(std::pow((goal_pose[0]-p_init[0]),2)+std::pow((goal_pose[1]-p_init[1]),2)+std::pow((goal_pose[2]-p_init[2]),2)+
                    std::pow((goal_pose[4]-p_init[4]),2)+std::pow((goal_pose[5]-p_init[5]),2)+std::pow((goal_pose[6]-p_init[6]),2)+
                    std::pow((goal_pose[8]-p_init[8]),2)+std::pow((goal_pose[9]-p_init[9]),2)+std::pow((goal_pose[10]-p_init[10]),2));
    bool f1 = false;
    bool f2 = false;

    // dont move if you are already there!
    if (max_dist < 0.0005) {  // half mm accuracy
        std::cout << " Already at goal positon (eps = 0.5 mm)! No translation motion required " << std::endl;
        f1 = true;
    }
    if (max_ori_dist < 0.05) {
        std::cout << " Already at goal orientation (eps = 0.05 rad), No rotation required " << std::endl;
        f2 = true;
    }
    if (res == 3) {
       f1 = true;
       f2 = true;
       std::cout << " Only moveing fingers " << std::endl;
    }
    if (f1 == true and f2 == true) {
       goto gripper_label;
    }

    std::cout << "Initial (xyz) position: " << p_init[12] << " " << p_init[13] << " " << p_init[14] <<std::endl;
    std::cout << "Goal (xyz) position: " << goal_pose[12] << " " << goal_pose[13] << " " << goal_pose[14] <<std::endl;
    // std::cout << "Goal (euler) orientation: " << goal_a << " " << goal_b << " " << goal_c <<std::endl;
    std::cout << "Initial (xyz) distance to goal: " << max_dist << std::endl;
    std::cout << "Enter to start control loop" << std::endl;
    //std::cin.ignore();



    // find exact pos algo... 
    // new idea: find exact position script within python that changes all x,y,z etc and here only interrupt when forces are too high and send back a report that this happened so that a new pose can be assumed.

   
    //////////////////////
    // go :: control loop
    robot.control([&time, &initial_pose, &max_dist, &goal_pose, &insert_, &maxtime, &force_limit](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose {
      
      time += period.toSec();
      double sum_abs_initial_forces = 0.0;
      double sum_abs_current_forces = 0.0;
      int stopped = 0;

      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE_c;
        sum_abs_initial_forces = abs(robot_state.K_F_ext_hat_K[0]) + abs(robot_state.K_F_ext_hat_K[1]) + abs(robot_state.K_F_ext_hat_K[2]);
        std::cout << "Sum of abs initial forces: " << sum_abs_initial_forces << std::endl;
      } 

      std::array<double, 16> new_pose = initial_pose;
      std::array<double, 16> current_pose = robot_state.O_T_EE_c;  //T matrix (12 13 14 are the pos)
      std::array<double, 16> delta;
      std::array<double, 16> pd;

      sum_abs_current_forces =  abs(robot_state.K_F_ext_hat_K[0]) + abs(robot_state.K_F_ext_hat_K[1]) + abs(robot_state.K_F_ext_hat_K[2]);


      // find exact position (insertion) - force interrupt if insertmode, force surpassed and goal below current pose (insertion)
      if ((insert_ == 1) and (sum_abs_current_forces > force_limit) and (current_pose[14] > goal_pose[14])) {
        std::cout << " Collision! Stopping and reporting." << std::endl;
        // goal_pose[12] +=
        // goal_pose[13] +=
        // goal_pose[14] += robot_state.O_T_EE_c[14] + 0.01; // changeing pose leads to discontinuity
        time = maxtime; // TODO this also leads to discont if the motion was fast before.
        stopped = 1;
      }      


      // // // // // //
      // PD - controller: linear motion to a target 
      //// pd - params
      double k_p = 10.0;
      double k_d = 0.01;
      double dt = 1.0/1000.0;  // franka control loop is 1kHz ... so this might be correct
      //std::cout << "dt and time/maxtime " << dt << " " << time/maxtime << std::endl;
      // step size -- used as motion velocity parameters (lower this value to avoid panda reflex stop)   
      double dv_p = 0.00045;
      double dv_o = 0.00045; 
      double max_ori_delta = 1.0;
 

      // distances to goal pos and ori
      delta[12] = (goal_pose[12] - current_pose[12]);
      delta[13] = (goal_pose[13] - current_pose[13]); 
      delta[14] = (goal_pose[14] - current_pose[14]);
     
      delta[0]  = (goal_pose[0]  - current_pose[0]);
      delta[1]  = (goal_pose[1]  - current_pose[1]); 
      delta[2]  = (goal_pose[2]  - current_pose[2]);

      delta[4]  = (goal_pose[4]  - current_pose[4]);
      delta[5]  = (goal_pose[5]  - current_pose[5]); 
      delta[6]  = (goal_pose[6]  - current_pose[6]);

      delta[8]  = (goal_pose[8]  - current_pose[8]);
      delta[9]  = (goal_pose[9]  - current_pose[9]); 
      delta[10] = (goal_pose[10] - current_pose[10]);

      double dist = sqrt(pow(delta[12],2)+pow(delta[13],2)+pow(delta[14],2));
      double ori_delta = sqrt(pow(delta[0],2))+sqrt(pow(delta[1],2))+sqrt(pow(delta[2],2))+  
                         sqrt(pow(delta[4],2))+sqrt(pow(delta[5],2))+sqrt(pow(delta[6],2))+
                         sqrt(pow(delta[8],2))+sqrt(pow(delta[9],2))+sqrt(pow(delta[10],2));
      if (time == 0.0) {
        std::cout << "GO" << std::endl;
        max_ori_delta = ori_delta;
      } 
      std::cout << "time " << time << " Distance " << dist << " delta_ori. " << ori_delta << " Sum|forces| " << sum_abs_current_forces << std::endl;
     

     // scaling by either time or distances
      double timescale = time / maxtime;  // 0 .... 1 with increasing time
      double smooth_timescale = (1 - cos(time/ maxtime * M_PI))/2; 
      //double distscale1 = (max_dist - dist) / max_dist; // 0 ....1 with decreasing dist
      //double distscale2 = dist / max_dist; // 1 ....0 with decreasing dist (dist1*dist2: 0...1/4...0)
      double pos_scale = smooth_timescale;
      double ori_scale = smooth_timescale;
      //// calc pd        
      pd[12] = k_p * delta[12] + k_d * delta[12]/dt;	
      pd[13] = k_p * delta[13] + k_d * delta[13]/dt;
      pd[14] = k_p * delta[14] + k_d * delta[14]/dt;

      pd[0]  = k_p * delta[0]  + k_d * delta[0]/dt;	
      pd[1]  = k_p * delta[1]  + k_d * delta[1]/dt;
      pd[2]  = k_p * delta[2]  + k_d * delta[2]/dt;

      pd[4]  = k_p * delta[4]  + k_d * delta[4]/dt;	
      pd[5]  = k_p * delta[5]  + k_d * delta[5]/dt;
      pd[6]  = k_p * delta[6]  + k_d * delta[6]/dt;

      pd[8]  = k_p * delta[8]  + k_d * delta[8]/dt;	
      pd[9]  = k_p * delta[9]  + k_d * delta[9]/dt;
      pd[10] = k_p * delta[10] + k_d * delta[10]/dt;
      //std::cout << "pd x, y, z " << pd[12] << " " << pd[13] << " " << pd[14] << std::endl;

      // do step 
      new_pose[12] = current_pose[12] + pos_scale  * dv_p * pd[12]; // time/maxtime * delta_x;
      new_pose[13] = current_pose[13] + pos_scale  * dv_p * pd[13]; // time/maxtime * delta_y;
      new_pose[14] = current_pose[14] + pos_scale  * dv_p * pd[14]; // time/maxtime * delta_z;

      new_pose[0]  =  current_pose[0] + ori_scale  * dv_o * pd[0]; 
      new_pose[1]  =  current_pose[1] + ori_scale  * dv_o * pd[1];
      new_pose[2]  =  current_pose[2] + ori_scale  * dv_o * pd[2];

      new_pose[4]  =  current_pose[4] + ori_scale  * dv_o * pd[4];
      new_pose[5]  =  current_pose[5] + ori_scale  * dv_o * pd[5];
      new_pose[6]  =  current_pose[6] + ori_scale  * dv_o * pd[6];

      new_pose[8]  =  current_pose[8]  + ori_scale  * dv_o * pd[8];
      new_pose[9]  =  current_pose[9]  + ori_scale  * dv_o * pd[9];
      new_pose[10] =  current_pose[10] + ori_scale  * dv_o * pd[10];

      //std::cout << new_pose[12] << " " << new_pose[13] << " " << new_pose[14]  << std::endl
      //std::cout << new_pose[0] << " " << new_pose[4] << " " << new_pose[8]  << std::endl
      //          << new_pose[1] << " " << new_pose[5] << " " << new_pose[9]  << std::endl
      //          << new_pose[2] << " " << new_pose[6] << " " << new_pose[10] << std::endl;
     

      ////////////////////
      // finished
      if (time >= maxtime) { 
        // I/O 
        // collect robot state! 
        std::array<double, 16> p_now = robot_state.O_T_EE_c; // final info on EE
        std::array<double, 7> q_now = robot_state.q;         // final info on joints
        // output to terminal
        std::cout << std::endl << "Finished motion, shutting down" << std::endl;
        std::cout << "EE position: " << p_now[12] << " " << p_now[13] << " " << p_now[14] << std::endl;
        std::cout << "Orientation Matrix: " << std::endl
                  << p_now[0] << " " << p_now[4] << " " << p_now[8] << std::endl
                  << p_now[1] << " " << p_now[5] << " " << p_now[9] << std::endl
                  << p_now[2] << " " << p_now[6] << " " << p_now[10] << std::endl; // orientation matrix
        std::cout << "with remaining errors: x,y,z,ori " << delta[12] << " " << delta[13] << " " << delta[14] 
                                                                      << " " << ori_delta << std::endl;
        std::cout << "Corresponding joint angles: " 
                  << q_now[0] << " " << q_now[1] << " " << q_now[2] << " " << q_now[3] << " " 
                  << q_now[4] << " " << q_now[5] << " " << q_now[6] << " " << std::endl;

        // save to file (data<timestamp>.txt) - for savekeeping
        // auto timestamp = std::time(nullptr);
        //std::string _timestamp = std::to_string(timestamp);  
        //std::cout << "Writing to file with timestamp: " << _timestamp << std::endl;
        //std::ofstream myfile; 
        //myfile.open("/home/franka1/franka/libfranka/data/data"+_timestamp+".txt");
        //myfile << p_now[12] << " " << p_now[13] << " " << p_now[14] << "\n";
        //myfile << delta[12] << " " << delta[13] << " " << delta[14] << ori_delta << "\n";
        //myfile << p_now[0]  << " " << p_now[4]  << " " << p_now[8]  << std::endl
        //       << p_now[1]  << " " << p_now[5]  << " " << p_now[9]  << std::endl
        //       << p_now[2]  << " " << p_now[6]  << " " << p_now[10] << std::endl;
        //myfile.close();

        // save to file FOR I/O between this script and _server.py (always same name, delete content in beginning)
        std::ofstream myfileIO; 
        myfileIO.open("/home/franka1/franka/libfranka/data/dataIO.txt", std::ofstream::trunc); // open and delete content
        myfileIO << p_now[12]   << " " << p_now[13] << " " << p_now[14] << " "
                 << delta[12]   << " " << delta[13] << " " << delta[14] << " " << ori_delta << " "
                 << p_now[0]    << " " << p_now[4]  << " " << p_now[8]  << " " 
                 << p_now[1]    << " " << p_now[5]  << " " << p_now[9]  << " "
                 << p_now[2]    << " " << p_now[6]  << " " << p_now[10] << " "
                 << stopped     << std::endl;

        myfileIO.close();
     
        return franka::MotionFinished(new_pose);
      }
      return new_pose;
    }); // end of control loop
    // Grasp object? !
    gripper_label: // goto label
    if (true){ 
	    franka::GripperState gripper_state = gripper.readOnce();
	    // Check for the maximum grasping width.
	    if (gripper_state.max_width < grasping_width) {
	      std::cout << "Object is too large for the current fingers on the gripper. max and grasping width: " << gripper_state.max_width << " " << grasping_width << " Setting grasping to max_width." << std::endl;
	      grasping_width = gripper_state.max_width;   
	    }
	    // sg = 0 dont change
	    if (should_grasp == 0) { 
	      std::cout << std::endl << "Gripper: Change nothing" << std::endl;
	    }
	    // sg = 1 open @ width
	    if (should_grasp == 1) {
	      std::cout << std::endl << "Gripper: Opening to width: " << grasping_width << std::endl;
	      gripper.stop();
	      gripper.move(grasping_width,0.1);
	    } 
	    // sg = 2 close on obj @ width
	    if (should_grasp == 2) { 
	      std::cout << std::endl << "Gripper: Checking if object is already grasped... " << grasping_width << std::endl;
	      // Grasp the object.
	      gripper_state = gripper.readOnce();
	      if (!gripper_state.is_grasped) {
		std::cout << std::endl << "Gripper: Attempt to grasp object... " << grasping_width << std::endl;
		if (!gripper.grasp(grasping_width, 0.1, 60)) {
		  std::cout << "Gripper: Failed to grasp object" << std::endl;
		}
		std::cout << "Gripper: Object grasp successful" << std::endl;
		
	      }
	      else {std::cout << "Gripper: Object already grasped" << std::endl;}
	      // Wait 1s and check afterwards, if the object is still grasped.
	      std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(444));
	      gripper_state = gripper.readOnce();
	      if (!gripper_state.is_grasped) {
		std::cout << "Gripper: Object lost" << std::endl;
	      }
	    }
	    // sg = 3 release // stop action
	    if (should_grasp == 3) { 
	    std::cout << std::endl << "Gripper: Releasing object / stopping gripper" << std::endl;
	    gripper.stop();
	    }
               // grasp feedback
	   bool grasped = false;
	   gripper_state = gripper.readOnce();
	   if (gripper_state.is_grasped) {grasped = true;} 
	   // add grasp success to output
	   std::ofstream myfileIO; 
	   myfileIO.open("/home/franka1/franka/libfranka/data/dataIO.txt", std::ofstream::app);
	   myfileIO << " " << grasped << std::endl;
	   myfileIO.close();
    }

  
  // end of try
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}

#!/usr/bin/env ruby
# Simulation of HDPR on DECOS

require 'orocos'
require 'rock/bundle'
require 'readline'

include Orocos

## Initialize orocos ##
Bundles.initialize

Orocos::Process.run 'navigation', 'control', 'simulation', 'autonomy' do

  ## SETUP ##

  # setup locomotion_control
    puts "Setting up locomotion_control"
    locomotion_control = Orocos.name_service.get 'locomotion_control'
    Orocos.conf.apply(locomotion_control, ['hdpr'], :override => true)
    locomotion_control.configure
    puts "done"

  # setup simulation_vrep
    puts "Setting up simulation_vrep"
    simulation_vrep = Orocos.name_service.get 'simulation'
    Orocos.conf.apply(simulation_vrep, ['hdpr'], :override => true)
    simulation_vrep.port = 19997
    simulation_vrep.configure
    puts "done"

  # setup motion_translator
    puts "Setting up motion_translator"
    motion_translator = Orocos.name_service.get 'motion_translator'
    Orocos.conf.apply(motion_translator, ['hdpr'], :override => true)
    motion_translator.configure
    puts "done"

  # setup read_joint_dispatcher
    puts "Setting up reading joint_dispatcher"
    read_joint_dispatcher = Orocos.name_service.get 'read_joint_dispatcher'
    Orocos.conf.apply(read_joint_dispatcher, ['hdpr_reading'], :override => true)
    read_joint_dispatcher.configure
    puts "done"

  # setup command_joint_dispatcher
    puts "Setting up commanding joint_dispatcher"
    command_joint_dispatcher = Orocos.name_service.get 'command_joint_dispatcher'
    Orocos.conf.apply(command_joint_dispatcher, ['hdpr_commanding'], :override => true)
    command_joint_dispatcher.configure
    puts "done"

  # setup waypoint_navigation
    puts "Setting up waypoint_navigation"
    waypoint_navigation = Orocos.name_service.get 'waypoint_navigation'
    Orocos.conf.apply(waypoint_navigation, ['hdpr_simulation'], :override => true)
    waypoint_navigation.configure
    puts "done"

  # setup command_arbitrer
    puts "Setting up command arbiter"
    arbiter = Orocos.name_service.get 'command_arbiter'
    Orocos.conf.apply(arbiter, ['default'], :override => true)
    arbiter.configure
    puts "done"

  # setup path_planning
    puts "Setting up path planner"
    path_planner = Orocos.name_service.get 'path_planner'
    path_planner.keep_old_waypoints = true
    Orocos.conf.apply(path_planner, ['hdpr','decos_test','slip&obstacles'], :override => true)
    path_planner.write_results = true
    path_planner.configure
    puts "done"
    
  # setup cost_updating
    puts "Setting up cost updating"
    cost_updating = Orocos.name_service.get 'cost_updating'
    Orocos.conf.apply(cost_updating, ['hdpr','decos_test','slip&obstacles'], :override => true)
    cost_updating.configure
    puts "done"

  ## LOGGERS ##
  # Orocos.log_all_ports
    Orocos.log_all_configuration

    logger_path_planner = Orocos.name_service.get 'autonomy_Logger'
    logger_path_planner.file = "path_planner.log"
    logger_path_planner.log(path_planner.trajectory2D)
    logger_path_planner.log(path_planner.actual_total_cost)
    logger_path_planner.log(path_planner.global_Total_Cost_map)
    logger_path_planner.log(path_planner.global_Cost_map)
    logger_path_planner.log(path_planner.local_Risk_map)
    logger_path_planner.log(path_planner.local_Propagation_map)
    logger_path_planner.log(path_planner.local_computation_time)


  ## PORT CONNECTIONS ##
    puts "Connecting ports"

    simulation_vrep.pose.connect_to                       path_planner.pose
    simulation_vrep.goalWaypoint.connect_to               path_planner.goalWaypoint
    simulation_vrep.pose.connect_to                       waypoint_navigation.pose
    simulation_vrep.joints_readings.connect_to            read_joint_dispatcher.joints_readings

    path_planner.trajectory.connect_to                    simulation_vrep.trajectory
    path_planner.trajectory.connect_to	                  waypoint_navigation.trajectory

    locomotion_control.joints_commands.connect_to         command_joint_dispatcher.joints_commands
    command_joint_dispatcher.motors_commands.connect_to   simulation_vrep.joints_commands
    
    read_joint_dispatcher.motors_samples.connect_to       locomotion_control.joints_readings

    waypoint_navigation.motion_command.connect_to         arbiter.follower_motion_command
    arbiter.motion_command.connect_to                     locomotion_control.motion_command

    waypoint_navigation.motion_command.connect_to         cost_updating.motion_command
    simulation_vrep.pose.connect_to                       cost_updating.pose

    path_planner.current_terrain.connect_to               cost_updating.current_terrain
    path_planner.reconnecting_index.connect_to            cost_updating.reconnecting_index
    path_planner.trajectory.connect_to                    cost_updating.trajectory
    
    waypoint_navigation.trajectory_status.connect_to      cost_updating.trajectory_status

    cost_updating.feedback_data.connect_to                path_planner.feedback_data

    simulation_vrep.start
    sleep 1
    read_joint_dispatcher.start
    command_joint_dispatcher.start
    locomotion_control.start
    motion_translator.start
    arbiter.start
    waypoint_navigation.start
    path_planner.start
    cost_updating.start

    logger_path_planner.start

    simulation_vrep.goalWaypoint.disconnect_from          path_planner.goalWaypoint

    trav_writer = path_planner.set_random_travmap.writer
    t1 = Time.now
    r = Random.rand(50)+150
    
    #puts waypoint_navigation.state
    while true
        if Time.now-t1 > r
            trav_writer.write(true)
            r = Random.rand(50)+150
            t1 = Time.now
        end
        if waypoint_navigation.state == :TARGET_REACHED
            break
        end
    end


    #goal_writer = path_planner.goalWaypoint.writer
    #goal = Types::Base::Waypoint.new()
    #goal.position[0] = 110.0
    #goal.position[1] = 60.0
    #goal.heading = -90.00*3.141592/180.0
    #goal_writer.write(goal)
    
    Readline::readline("Press ENTER when the new goal is set\n")

    simulation_vrep.goalWaypoint.connect_to               path_planner.goalWaypoint

    while waypoint_navigation.state == :TARGET_REACHED
    end

    simulation_vrep.goalWaypoint.disconnect_from          path_planner.goalWaypoint

    t1 = Time.now
    r = Random.rand(50)+150

    #puts waypoint_navigation.state
    while true
        if Time.now-t1 > r
            trav_writer.write(true)
            r = Random.rand(50)+150
            t1 = Time.now
        end
        if waypoint_navigation.state == :TARGET_REACHED
            break
        end
    end

    Readline::readline("Press ENTER to exit\n")
end

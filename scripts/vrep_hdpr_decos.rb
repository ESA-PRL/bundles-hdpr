# Simulation of HDPR on DECOS

require 'orocos'
require 'rock/bundle'
require 'readline'

include Orocos

## Initialize orocos ##
Bundles.initialize

Orocos::Process.run 'navigation', 'control', 'simulation', 'autonomy' do

  ## SETUP ##

  # setup simulation_vrep
    puts "Setting up simulation_vrep"
    simulation_vrep = Orocos.name_service.get 'simulation'
    Orocos.conf.apply(simulation_vrep, ['hdpr'], :override => true)
    simulation_vrep.port = 19997
    simulation_vrep.configure
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

  # setup path_planning
    puts "Setting up path planner"
    path_planner = Orocos.name_service.get 'path_planner'
    path_planner.keep_old_waypoints = true
    Orocos.conf.apply(path_planner, ['hdpr','decos'], :override => true)
    path_planner.write_results = false
    path_planner.configure
    puts "done"

  ## LOGGERS ##
  # Orocos.log_all_ports


  ## PORT CONNECTIONS ##
    puts "Connecting ports"

    simulation_vrep.pose.connect_to                       path_planner.pose
    simulation_vrep.goalWaypoint.connect_to               path_planner.goalWaypoint
    simulation_vrep.joints_readings.connect_to            read_joint_dispatcher.joints_readings

    path_planner.trajectory.connect_to                    simulation_vrep.trajectory
    command_joint_dispatcher.motors_commands.connect_to   simulation_vrep.joints_commands
    
    simulation_vrep.start
    sleep 1
    read_joint_dispatcher.start
    command_joint_dispatcher.start
    path_planner.start

    Readline::readline("Press ENTER to exit\n")
end

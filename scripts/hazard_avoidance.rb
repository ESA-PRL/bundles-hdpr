#!/usr/bin/env ruby

require 'orocos'
require 'rock/bundle'
require 'readline'
require 'optparse'
include Orocos

options = {:bb2 => true,
           :v => true,
           :csc => true,
           :logging => false}

OptionParser.new do |opts|
  opts.banner = "Usage: start.rb [options]"
  opts.on('-v', '--vicon state', 'Enable vicon over gps') { |state| options[:v] = state }
  opts.on('-csc', '--customShutterController state', 'Enable/disable custom shutter controller') { |state| options[:csc] = state }
end.parse!

Bundles.initialize

Orocos::Process.run 'autonomy', 'navigation', 'control', 'unit_bb2', 'imu', 'gps', 'gyro', 'unit_vicon', 'unit_shutter_controller', 'unit_hazard_detector', 'fdir' do

    joystick = Orocos.name_service.get 'joystick'
    Orocos.conf.apply(joystick, ['default'], :override => true)
    begin
        joystick.configure
    rescue
        abort("Cannot configure the joystick, is the dongle connected to HDPR?")
    end

    motion_translator = Orocos.name_service.get 'motion_translator'
    Orocos.conf.apply(motion_translator, ['hdpr'], :override => true)
    motion_translator.configure

    locomotion_control = Orocos.name_service.get 'locomotion_control'
    Orocos.conf.apply(locomotion_control, ['hdpr'], :override => true)
    locomotion_control.configure

    command_joint_dispatcher = Orocos.name_service.get 'command_joint_dispatcher'
    Orocos.conf.apply(command_joint_dispatcher, ['hdpr_commanding'], :override => true)
    command_joint_dispatcher.configure

    platform_driver = Orocos.name_service.get 'platform_driver_hdpr'
    Orocos.conf.apply(platform_driver, ['default'], :override => true)
    platform_driver.configure

    read_joint_dispatcher = Orocos.name_service.get 'read_joint_dispatcher'
    Orocos.conf.apply(read_joint_dispatcher, ['hdpr_reading'], :override => true)
    read_joint_dispatcher.configure

    imu_stim300 = TaskContext.get 'imu_stim300'
    Orocos.conf.apply(imu_stim300, ['default', 'HDPR', 'ESTEC', 'stim300_5g'], :override => true)
    imu_stim300.configure
    
    fdir = TaskContext.get 'fdir'
    Orocos.conf.apply(fdir, ['exoter'], :override => true)
    fdir.configure

    if options[:v] == false
        gps = TaskContext.get 'gps'
        Orocos.conf.apply(gps, ['HDPR', 'Netherlands', 'DECOS'], :override => true)
        gps.configure
    
        gps_heading = TaskContext.get 'gps_heading'
        Orocos.conf.apply(gps_heading, ['default'], :override => true)
        gps_heading.configure
    else
        vicon = TaskContext.get 'vicon'
        Orocos.conf.apply(vicon, ['default','hdpr'], :override => true)
        vicon.configure
    end

    puts "Starting BB2"

    camera_firewire_bb2 = TaskContext.get 'camera_firewire_bb2'
    Orocos.conf.apply(camera_firewire_bb2, ['hdpr_bb2','egp_bb2_id','auto_exposure'], :override => true)
    camera_firewire_bb2.configure

    camera_bb2 = TaskContext.get 'camera_bb2'
    Orocos.conf.apply(camera_bb2, ['egp_bb2'], :override => true)
    camera_bb2.configure

    stereo_bb2 = TaskContext.get 'stereo_bb2'
    Orocos.conf.apply(stereo_bb2, ['egp_bb2'], :override => true)
    stereo_bb2.configure

    shutter_controller_bb2 = Orocos.name_service.get 'shutter_controller'
    Orocos.conf.apply(shutter_controller_bb2, ['bb2tenerife'], :override => true)
    shutter_controller_bb2.configure

    waypoint_navigation = Orocos.name_service.get 'waypoint_navigation'
    Orocos.conf.apply(waypoint_navigation, ['hdpr_lab'], :override => true)
    waypoint_navigation.configure

    command_arbiter = Orocos.name_service.get 'command_arbiter'
    Orocos.conf.apply(command_arbiter, ['default'], :override => true)
    command_arbiter.configure

    gyro = TaskContext.get 'dsp1760'
    Orocos.conf.apply(gyro, ['default'], :override => true)
    gyro.configure

    hazard_detector = Orocos.name_service.get 'hazard_detector'
    Orocos.conf.apply(hazard_detector, ['default'], :override => true)
    hazard_detector.configure

    traversability = Orocos.name_service.get 'traversability'
    Orocos.conf.apply(traversability, ['hdpr'], :override => true)
    traversability.configure

    path_planner = Orocos.name_service.get 'path_planner'
    path_planner.keep_old_waypoints = true
    Orocos.conf.apply(path_planner, ['hdpr','prl'], :override => true)
    path_planner.configure
    puts "done"

    joystick.raw_command.connect_to                     motion_translator.raw_command
    joystick.raw_command.connect_to                     command_arbiter.raw_command

    motion_translator.motion_command.connect_to         command_arbiter.joystick_motion_command
    waypoint_navigation.motion_command.connect_to       command_arbiter.follower_motion_command
    command_arbiter.motion_command.connect_to           locomotion_control.motion_command
    command_arbiter.motion_command.connect_to           hazard_detector.motion_command

    locomotion_control.joints_commands.connect_to       command_joint_dispatcher.joints_commands
    command_joint_dispatcher.motors_commands.connect_to platform_driver.joints_commands
    platform_driver.joints_readings.connect_to          read_joint_dispatcher.joints_readings
    read_joint_dispatcher.motors_samples.connect_to     locomotion_control.joints_readings

    camera_firewire_bb2.frame.connect_to                camera_bb2.frame_in
    camera_firewire_bb2.frame.connect_to                shutter_controller_bb2.frame
    camera_firewire_bb2.shutter_value.connect_to        shutter_controller_bb2.shutter_value

    camera_bb2.left_frame.connect_to                    stereo_bb2.left_frame
    camera_bb2.right_frame.connect_to                   stereo_bb2.right_frame

    stereo_bb2.left_frame_sync.connect_to               hazard_detector.camera_frame
    stereo_bb2.distance_frame.connect_to                hazard_detector.distance_frame

    waypoint_navigation.trajectory_status.connect_to    fdir.trajectory_status
    imu_stim300.orientation_samples_out.connect_to      fdir.attitude
    platform_driver.error_in_motor.connect_to           fdir.error_in_motor
    hazard_detector.hazard_detected.connect_to          fdir.hazard_detected
    fdir.fault_detected.connect_to                      command_arbiter.fault_detected
    hazard_detector.local_traversability.connect_to     traversability.local2global_orientation
    traversability.traversability_map.connect_to        path_planner.traversability_map
    path_planner.finished_planning.connect_to           hazard_detector.new_plan
    path_planner.trajectory.connect_to                  waypoint_navigation.trajectory

    if options[:v] == false
        gps.pose_samples.connect_to                         gps_heading.gps_pose_samples
        gps.raw_data.connect_to                             gps_heading.gps_raw_data
        gps_heading.pose_samples_out.connect_to             waypoint_navigation.pose
        gps_heading.pose_samples_out.connect_to             path_planner.pose
        gps_heading.pose_samples_out.connect_to             traversability.pose
        imu_stim300.orientation_samples_out.connect_to      gps_heading.imu_pose_samples
        gyro.orientation_samples.connect_to                 gps_heading.gyro_pose_samples
        command_arbiter.motion_command.connect_to           gps_heading.motion_command
        puts "using gps"
    else
        vicon.pose_samples.connect_to                       waypoint_navigation.pose
        vicon.pose_samples.connect_to                       path_planner.pose
        vicon.pose_samples.connect_to                       traversability.pose
        puts "using vicon"
    end

    Orocos.log_all_configuration

    logger_hazard_detector = Orocos.name_service.get 'unit_hazard_detector_Logger'
    logger_hazard_detector.file = "hazard_detector.log"
    logger_hazard_detector.log(hazard_detector.hazard_detected)
    logger_hazard_detector.log(hazard_detector.hazard_visualization)
    logger_hazard_detector.log(hazard_detector.local_traversability)

    logger_path_planner = Orocos.name_service.get 'autonomy_Logger'
    logger_path_planner.file = "path_planner.log"
    logger_path_planner.log(path_planner.trajectory2D)
    logger_path_planner.log(path_planner.actual_total_cost)
    logger_path_planner.log(path_planner.global_Total_Cost_map)
    logger_path_planner.log(path_planner.global_Cost_map)
    logger_path_planner.log(path_planner.local_Risk_map)
    logger_path_planner.log(path_planner.local_Propagation_map)

    if options[:v] == false
        logger_gps_heading = Orocos.name_service.get 'gps_Logger'
        logger_gps_heading.file = "gps.log"
        logger_gps_heading.log(gps.pose_samples)
        logger_gps_heading.log(gps.raw_data)
        logger_gps_heading.log(gps.time)
        logger_gps_heading.log(gps_heading.pose_samples_out)
    else
        #logger_vicon = Orocos.name_service.get 'vicon_Logger'
        #logger_vicon.file = "vicon.log"
        #logger_vicon.log(vicon.pose_samples)
        #logger_vicon.log(vicon.unlabeled_markers)
    end

    logger_imu = Orocos.name_service.get 'imu_Logger'
    logger_imu.file = "imu.log"
    logger_imu.log(imu_stim300.orientation_samples_out)

    platform_driver.start
    read_joint_dispatcher.start
    command_joint_dispatcher.start
    locomotion_control.start
    command_arbiter.start
    motion_translator.start
    joystick.start
    imu_stim300.start
    gyro.start
    fdir.start
    if options[:v] == false
        gps.start
        gps_heading.start
    else
        vicon.start
    end
    camera_bb2.start
    camera_firewire_bb2.start
    stereo_bb2.start
    shutter_controller_bb2.start
    hazard_detector.start

    if options[:v] == false
        # Race condition with internal gps_heading states. This check is here to only trigger the
        # trajectoryGen when the pose has been properly initialised. Otherwise the trajectory is set wrong.
        puts "Move rover forward to initialise the gps_heading component"
        while gps_heading.ready == false
           sleep 1
        end
        puts "GPS heading calibration done"
    end
    traversability.start
    waypoint_navigation.start
    path_planner.start

    Readline::readline("Press ENTER to send goal pose to planner\n") do
    end

    # start loggers
    if options[:logging]
        if options[:v] == false
            logger_gps_heading.start
        else
            #logger_vicon.start
        end
        logger_hazard_detector.start
        logger_path_planner.start
        logger_imu.start
    end

    #goal.start
    goal_writer = path_planner.goalWaypoint.writer
    goal = Types::Base::Waypoint.new()
    if options[:v] == false
        # Goal Position Test w/ Ricardo
        #goal.position[0] = 40.0
        #goal.position[1] = 115.0
        # Goal Position
        #goal.position[0] = 50.0
        #goal.position[1] = 110.0
        # Goal going to van
        goal.position[0] = 65.0
        goal.position[1] = 55.0
        # Start Position
        #goal.position[0] = 110.0
        #goal.position[1] = 55.0
    else
        goal.position[0] = 7.00
        goal.position[1] = 2.00
    end
    goal.position[2] = 0.00
    #goal.heading = 135.00*3.141592/180.0
    goal.heading = -90.00*3.141592/180.0
    #goal.heading = 0.0
    goal_writer.write(goal)

    Readline::readline("Press Enter to exit\n") do
    end
end

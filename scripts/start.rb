#!/usr/bin/env ruby

require 'orocos'
require 'rock/bundle'
require 'readline'
require 'optparse'
#require 'vizkit'
include Orocos

# Command line options for the script, default values
options = {:bb2 => true, :bb3 => true}

# Options parser
OptionParser.new do |opts|
  opts.banner = "Usage: start.rb [options]"
  opts.on('-bb2', '--bb2 state', 'Enable/disable BB2 camera') { |state| options[:bb2] = state }
  opts.on('-bb3', '--bb3 state', 'Enable/disable BB3 camera') { |state| options[:bb3] = state }
end.parse!

# Initialize bundles to find the configurations for the packages
Bundles.initialize

# Distance to move between 360 picture taking [meters]
$distance_360_picture = 50

# Execute the task
Orocos::Process.run 'hdpr_control', 'hdpr_pancam', 'hdpr_lidar', 'hdpr_tof', 'hdpr_bb2', 'hdpr_bb3', 'hdpr_imu', 'hdpr_gps', 'hdpr_temperature', 'hdpr_shutter_controller', 'hdpr_unit_gyro', 'hdpr_shutter_controller_bumblebee' do
    joystick = Orocos.name_service.get 'joystick'
    # Set the joystick input
    joystick.device = "/dev/input/js0"
    # In case the dongle is not connected exit gracefully
    begin
        # Configure the joystick
        joystick.configure
    rescue
        # Abort the process as there is no joystick to get input from
        abort("Cannot configure the joystick, is the dongle connected to HDPR?")
    end
    
    # Configure the control packages
    motion_translator = Orocos.name_service.get 'motion_translator'
    Orocos.conf.apply(motion_translator, ['default'], :override => true)
    motion_translator.configure
    
    locomotion_control = Orocos.name_service.get 'locomotion_control'
    Orocos.conf.apply(locomotion_control, ['default'], :override => true)
    locomotion_control.configure
    
    command_joint_dispatcher = Orocos.name_service.get 'command_joint_dispatcher'
    Orocos.conf.apply(command_joint_dispatcher, ['commanding'], :override => true)
    command_joint_dispatcher.configure
    
    platform_driver = Orocos.name_service.get 'platform_driver'
    Orocos.conf.apply(platform_driver, ['default'], :override => true)
    platform_driver.configure
    
    read_joint_dispatcher = Orocos.name_service.get 'read_joint_dispatcher'
    Orocos.conf.apply(read_joint_dispatcher, ['reading'], :override => true)
    read_joint_dispatcher.configure
    
    ptu_directedperception = Orocos.name_service.get 'ptu_directedperception'
    Orocos.conf.apply(ptu_directedperception, ['default'], :override => true)
    ptu_directedperception.configure
    
    # Configure the sensor packages
    velodyne_lidar = TaskContext.get 'velodyne_lidar'
    Orocos.conf.apply(velodyne_lidar, ['default'], :override => true)
    velodyne_lidar.configure
    
    tofcamera_mesasr = TaskContext.get 'tofcamera_mesasr'
    Orocos.conf.apply(tofcamera_mesasr, ['default'], :override => true)
    tofcamera_mesasr.configure
    
    imu_stim300 = TaskContext.get 'imu_stim300'
    #Orocos.conf.apply(imu_stim300, ['default', 'HDPR', 'ESTEC', 'stim300_5g'], :override => true)
    Orocos.conf.apply(imu_stim300, ['default', 'HDPR', 'Tenerife', 'stim300_5g'], :override => true)
    imu_stim300.configure
    
    gps = TaskContext.get 'gps'
    #Orocos.conf.apply(gps, ['HDPR', 'Netherlands', 'DECOS'], :override => true)
    Orocos.conf.apply(gps, ['HDPR', 'Spain', 'Tenerife'], :override => true)
    gps.configure
    
    gps_heading = TaskContext.get 'gps_heading'
    Orocos.conf.apply(gps_heading, ['default'], :override => true)
    gps_heading.configure

    if options[:bb2] == true
        puts "Starting BB2"
    
        camera_firewire_bb2 = TaskContext.get 'camera_firewire_bb2'
        Orocos.conf.apply(camera_firewire_bb2, ['bumblebee2'], :override => true)
        camera_firewire_bb2.configure
    
        camera_bb2 = TaskContext.get 'camera_bb2'
        Orocos.conf.apply(camera_bb2, ['default'], :override => true)
        camera_bb2.configure
    end

    if options[:bb3] == true
        puts "Starting BB3"
        
        camera_firewire_bb3 = TaskContext.get 'camera_firewire_bb3'
        Orocos.conf.apply(camera_firewire_bb3, ['bumblebee3'], :override => true)
        camera_firewire_bb3.configure
        
        camera_bb3 = TaskContext.get 'camera_bb3'
        Orocos.conf.apply(camera_bb3, ['default'], :override => true)
        camera_bb3.configure
    end
    
    pancam_left = Orocos.name_service.get 'pancam_left'
    Orocos.conf.apply(pancam_left, ['grashopper2_left'], :override => true)
    pancam_left.configure
    
    pancam_right = Orocos.name_service.get 'pancam_right'
    Orocos.conf.apply(pancam_right, ['grashopper2_right'], :override => true)
    pancam_right.configure

    shutter_controller = Orocos.name_service.get 'shutter_controller'
    Orocos.conf.apply(shutter_controller, ['default'], :override => true)
    shutter_controller.configure

    shutter_controller_bb2 = Orocos.name_service.get 'shutter_controller_bb2'
    Orocos.conf.apply(shutter_controller_bb2, ['bb2tenerife'], :override => true)
    shutter_controller_bb2.configure

    shutter_controller_bb3 = Orocos.name_service.get 'shutter_controller_bb3'
    Orocos.conf.apply(shutter_controller_bb3, ['bb3tenerife'], :override => true)
    shutter_controller_bb3.configure
    
    pancam_panorama = Orocos.name_service.get 'pancam_panorama'
    Orocos.conf.apply(pancam_panorama, ['default'], :override => true)
    pancam_panorama.configure
    
    pancam_360 = Orocos.name_service.get 'pancam_360'
    Orocos.conf.apply(pancam_360, ['default', 'separation_40_30'], :override => true)
    pancam_360.configure
    
     # Setup Waypoint_navigation 
    waypoint_navigation = Orocos.name_service.get 'waypoint_navigation'
    Orocos.conf.apply(waypoint_navigation, ['default','hdpr'], :override => true)
    waypoint_navigation.configure

    # Setup command arbiter
    command_arbiter = Orocos.name_service.get 'command_arbiter'
    Orocos.conf.apply(command_arbiter, ['default'], :override => true)
    command_arbiter.configure
    
	# Add the trajectory generation component
    trajectoryGen = Orocos.name_service.get 'trajectoryGen'
    #Orocos.conf.apply(trajectoryGen, ['decos3DROCS'], :override => true)
    #Orocos.conf.apply(trajectoryGen, ['tenerife0Nominal'], :override => true)
    #Orocos.conf.apply(trajectoryGen, ['tenerife0NominalReverse'], :override => true)
    #Orocos.conf.apply(trajectoryGen, ['tenerife18Track'], :override => true)
    #Orocos.conf.apply(trajectoryGen, ['tenerife2SideTrack'], :override => true)
    Orocos.conf.apply(trajectoryGen, ['tenerife3ValleyCircle'], :override => true)
    #Orocos.conf.apply(trajectoryGen, ['tenerife4Nuerburgring'], :override => true)
    trajectoryGen.configure
    
    # "Fuse" GPS position and IMU orientation to get rover pose
    simple_pose = Orocos.name_service.get 'simple_pose'
    Orocos.conf.apply(simple_pose, ['default'], :override => true)
    simple_pose.configure

    temperature = TaskContext.get 'temperature'
    Orocos.conf.apply(temperature, ['default'], :override => true)
    temperature.configure

    gyro = TaskContext.get 'dsp1760'
    Orocos.conf.apply(gyro, ['default'], :override => true)
    gyro.configure
    
    # Configure the connections between the components
    joystick.raw_command.connect_to                     motion_translator.raw_command
    joystick.raw_command.connect_to                     command_arbiter.raw_command
    
    motion_translator.motion_command.connect_to         command_arbiter.joystick_motion_command
    waypoint_navigation.motion_command.connect_to       command_arbiter.follower_motion_command
    command_arbiter.motion_command.connect_to           locomotion_control.motion_command
    
    locomotion_control.joints_commands.connect_to       command_joint_dispatcher.joints_commands
    command_joint_dispatcher.motors_commands.connect_to platform_driver.joints_commands
    platform_driver.joints_readings.connect_to          read_joint_dispatcher.joints_readings
    #read_joint_dispatcher.joints_samples.connect_to     locomotion_control.joints_readings
    read_joint_dispatcher.motors_samples.connect_to     locomotion_control.joints_readings
    
    if options[:bb2] == true
        camera_firewire_bb2.frame.connect_to                camera_bb2.frame_in
        camera_firewire_bb2.frame.connect_to                shutter_controller_bb2.frame
        camera_firewire_bb2.shutter_value.connect_to        shutter_controller_bb2.shutter_value
    end
    if options[:bb3] == true
        camera_firewire_bb3.frame.connect_to                camera_bb3.frame_in
        camera_firewire_bb3.frame.connect_to                shutter_controller_bb3.frame
        camera_firewire_bb3.shutter_value.connect_to        shutter_controller_bb3.shutter_value
    end
    
    # Waypoint navigation inputs:
    #imu_stim300.orientation_samples_out.connect_to      simple_pose.imu_pose
    #gps.pose_samples.connect_to                         simple_pose.gps_pose
    #simple_pose.pose.connect_to                         waypoint_navigation.pose
    gps.pose_samples.connect_to                         gps_heading.gps_pose_samples
    imu_stim300.orientation_samples_out.connect_to      gps_heading.imu_pose_samples
    #gyro.orientation_samples.connect_to                 gps_heading.gyro_pose_samples
    command_arbiter.motion_command.connect_to           gps_heading.motion_command
    gps.raw_data.connect_to                             gps_heading.gps_raw_data
    trajectoryGen.trajectory.connect_to                 waypoint_navigation.trajectory
    gps_heading.pose_samples_out.connect_to             waypoint_navigation.pose

    
    # PanCam connections to panorama and 360 components (must function exclusivly)
    pancam_panorama.pan_angle_in.connect_to             ptu_directedperception.pan_angle
    pancam_panorama.tilt_angle_in.connect_to            ptu_directedperception.tilt_angle
    pancam_panorama.pan_angle_out.connect_to            ptu_directedperception.pan_set
    pancam_panorama.tilt_angle_out.connect_to           ptu_directedperception.tilt_set
    pancam_left.frame.connect_to                        pancam_panorama.left_frame_in
    pancam_right.frame.connect_to                       pancam_panorama.right_frame_in
    pancam_360.pan_angle_in.connect_to                  ptu_directedperception.pan_angle
    pancam_360.tilt_angle_in.connect_to                 ptu_directedperception.tilt_angle
    pancam_360.pan_angle_out.connect_to                 ptu_directedperception.pan_set
    pancam_360.tilt_angle_out.connect_to                ptu_directedperception.tilt_set
    pancam_left.frame.connect_to                        pancam_360.left_frame_in
    pancam_right.frame.connect_to                       pancam_360.right_frame_in

    # PanCam connections to shutter controller
    pancam_left.frame.connect_to                        shutter_controller.frame
    pancam_left.shutter_value.connect_to                shutter_controller.shutter_value
    pancam_right.shutter_value.connect_to               shutter_controller.shutter_value
    
    # Log all the properties of the components
    Orocos.log_all_configuration
    
    # Define loggers
    logger_control = Orocos.name_service.get 'hdpr_control_Logger'
    logger_control.file = "control.log"
    logger_control.log(platform_driver.joints_readings)
    logger_control.log(command_arbiter.motion_command)
    logger_control.start
    
    logger_pancam = Orocos.name_service.get 'hdpr_pancam_Logger'
    logger_pancam.file = "pancam.log"
    logger_pancam.log(pancam_panorama.left_frame_out)
    logger_pancam.log(pancam_panorama.right_frame_out)
    logger_pancam.log(pancam_panorama.pan_angle_out_degrees)
    logger_pancam.log(pancam_panorama.tilt_angle_out_degrees)
    logger_pancam.log(pancam_360.left_frame_out)
    logger_pancam.log(pancam_360.right_frame_out)
    logger_pancam.log(pancam_360.pan_angle_out_degrees)
    logger_pancam.log(pancam_360.tilt_angle_out_degrees)
    logger_pancam.log(pancam_360.set_id)
    logger_pancam.log(shutter_controller.shutter_value)
    logger_pancam.log(ptu_directedperception.pan_angle)
    logger_pancam.log(ptu_directedperception.tilt_angle)
    logger_pancam.start
    
    if options[:bb2] == true
        logger_bb2 = Orocos.name_service.get 'hdpr_bb2_Logger'
        logger_bb2.file = "bb2.log"
        logger_bb2.log(camera_firewire_bb2.frame)
        logger_bb2.log(shutter_controller_bb2.shutter_value)
        logger_bb2.start
    end
    
    if options[:bb3] == true
        logger_bb3 = Orocos.name_service.get 'hdpr_bb3_Logger'
        logger_bb3.file = "bb3.log"
        logger_bb3.log(camera_firewire_bb3.frame)
        logger_bb2.log(shutter_controller_bb3.shutter_value)
        logger_bb3.start
    end
    
    logger_tof = Orocos.name_service.get 'hdpr_tof_Logger'
    logger_tof.file = "tof.log"
    logger_tof.log(tofcamera_mesasr.distance_frame)
    logger_tof.log(tofcamera_mesasr.ir_frame)
    logger_tof.log(tofcamera_mesasr.pointcloud)
    logger_tof.log(tofcamera_mesasr.tofscan)
    logger_tof.start
    
    logger_lidar = Orocos.name_service.get 'hdpr_lidar_Logger'
    logger_lidar.file = "lidar.log"
    logger_lidar.log(velodyne_lidar.ir_frame)
    logger_lidar.log(velodyne_lidar.laser_scans)
    logger_lidar.log(velodyne_lidar.range_frame)
    logger_lidar.log(velodyne_lidar.azimuth_frame)
    logger_lidar.log(velodyne_lidar.velodyne_time)
    logger_lidar.log(velodyne_lidar.accumulated_velodyne_time)
    logger_lidar.log(velodyne_lidar.estimated_clock_offset)
    logger_lidar.log(velodyne_lidar.velodyne_time)
    logger_lidar.start
    
    #logger_gps = Orocos.name_service.get 'hdpr_gps_Logger'
    #logger_gps.file = "gps.log"
    #logger_gps.log(gps.pose_samples)
    #logger_gps.log(gps.raw_data)
    #logger_gps.log(gps.time)
    #logger_gps.log(gps_heading.pose_samples_out)
    #logger_gps.start
    
    logger_imu = Orocos.name_service.get 'hdpr_imu_Logger'
    logger_imu.file = "imu.log"
    logger_imu.log(imu_stim300.inertial_sensors_out)
    logger_imu.log(imu_stim300.temp_sensors_out)
    logger_imu.log(imu_stim300.orientation_samples_out)
    logger_imu.log(imu_stim300.compensated_sensors_out)
    logger_imu.log(imu_stim300.packet_counter)
    logger_imu.start

    logger_temperature = Orocos.name_service.get 'hdpr_temperature_Logger'
    logger_temperature.file = "temperature.log"
    logger_temperature.log(temperature.temperature_samples)
    logger_temperature.start

    logger_gyro = Orocos.name_service.get 'hdpr_unit_gyro_Logger'
    logger_gyro.file = "gyro.log"
    logger_gyro.log(gyro.rotation)
    logger_gyro.log(gyro.rotation_raw)
    logger_gyro.log(gyro.orientation_samples)
    logger_gyro.log(gyro.bias_samples)
    logger_gyro.log(gyro.bias_values)
    logger_gyro.log(gyro.temperature)
    logger_gyro.log(gyro.sequence_counter)
    logger_gyro.start

    logger_waypoint = Orocos.name_service.get 'hdpr_gps_Logger'
    logger_waypoint.file = "waypoint_navigation.log"
    logger_waypoint.log(trajectoryGen.trajectory)
    logger_waypoint.log(waypoint_navigation.trajectory_status)
    logger_waypoint.log(waypoint_navigation.currentWaypoint)
    logger_waypoint.log(waypoint_navigation.motion_command)
    logger_waypoint.log(gps.pose_samples)
    logger_waypoint.log(gps.raw_data)
    logger_waypoint.log(gps.time)
    logger_waypoint.log(gps_heading.pose_samples_out)
    logger_waypoint.start

    # Start the components
    platform_driver.start
    read_joint_dispatcher.start
    command_joint_dispatcher.start
    locomotion_control.start
    ptu_directedperception.start
    pancam_left.start
    pancam_right.start
    motion_translator.start
    joystick.start
    velodyne_lidar.start
    tofcamera_mesasr.start
    imu_stim300.start
    gyro.start
    gps.start
    gps_heading.start
    if options[:bb2] == true
        camera_bb2.start
        camera_firewire_bb2.start
        shutter_controller_bb2.start
    end
    if options[:bb3] == true
        camera_bb3.start
        camera_firewire_bb3.start
        shutter_controller_bb3.start
    end
    temperature.start
    command_arbiter.start

    # Race condition with internal gps_heading states. This check is here to only trigger the 
    # trajectoryGen when the pose has been properly initialised. Otherwise the trajectory is set wrong.
    puts "Move rover forward to initialise the gps_heading component"
    while gps_heading.ready == false
        sleep 1
    end
    puts "GPS heading calibration done"

    # Trigger the trojectory generation, waypoint_navigation must be running at this point
    waypoint_navigation.start
    trajectoryGen.start
    trajectoryGen.trigger    
    # Waypoint navigation needs to be stopped at the beginning
    waypoint_navigation.stop
    
    # Get a reader instance for the GPS
    reader_gps_position = gps.pose_samples.reader
    
    # Initialise GPS position
    $last_gps_position = 0
    $distance = 0
    
    # 2-state machine toggling between waypoint navigation and 360 image taking
    # If the user is controlling the platform it will still switch between the states
    while true
        if pancam_360.state == :RUNNING
            puts "Still taking a picture, waiting 1 seconds"
            sleep 1
        else
            puts "360 degree picture done"
            pancam_panorama.start
            sleep 1

            shutter_controller.start
            sleep 1

            # Start waypoint navigation
            waypoint_navigation.start
            sleep 1

            # Wait for the rover to move for a defined distance in meters
            while $distance < $distance_360_picture or $distance.nan?
                sample = reader_gps_position.read_new
                if sample
                    # Initialise GPS position
                    if $last_gps_position == 0
                        $last_gps_position = sample
                    end
                    
                    # Evaluate distance from last position
                    dx = sample.position[0] - $last_gps_position.position[0]
                    dy = sample.position[1] - $last_gps_position.position[1]
                    # Cumulative distance
                    $distance = Math.sqrt(dx*dx + dy*dy)
                    #puts "Distance: #{$distance}"
                end
            end
            $distance = 0
            $last_gps_position = sample
            
            # Stop sending waypoint navigation commands, this should also stop the rover now so the following lines are not required
            waypoint_navigation.stop
            sleep 1
            
            # Stop the panorama taking
            pancam_panorama.stop
            sleep 1

            # Stop shutter controller for the duration of a 360
            shutter_controller.stop
            sleep 1

            puts "Taking new 360 degree picture"
            pancam_360.start
            sleep 1

        end
    end
    
    # Show camera output
    #Vizkit.display pancam_panorama.left_frame_out
    #Vizkit.display pancam_panorama.right_frame_out
    #Vizkit.display camera_bb2.left_frame
    #Vizkit.display camera_bb3.left_frame
    #Vizkit.display tofcamera_mesasr.distance_frame
    #Vizkit.display velodyne_lidar.ir_interp_frame    
    #Vizkit.exec
    
    # Not needed when Vizkit is running
    Readline::readline("Press Enter to exit\n") do
    end
end 

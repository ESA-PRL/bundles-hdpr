#!/usr/bin/env ruby

require 'orocos'
require 'rock/bundle'
require 'readline'
require 'optparse'
#require 'vizkit'
include Orocos

# Command line options for the script, default values
options = {:bb2 => true, :bb3 => true, :v => false}

# Options parser
OptionParser.new do |opts|
  opts.banner = "Usage: start.rb [options]"
  opts.on('-bb2', '--bb2 state', 'Enable/disable BB2 camera') { |state| options[:bb2] = state }
  opts.on('-bb3', '--bb3 state', 'Enable/disable BB3 camera') { |state| options[:bb3] = state }
  opts.on('-v', '--vicon state', 'Enable vicon over gps') { |state| options[:v] = state }
end.parse!

# Initialize bundles to find the configurations for the packages
Bundles.initialize

## Transformation for the transformer
Bundles.transformer.load_conf(Bundles.find_file('config', 'transforms_scripts.rb'))

# Execute the task
Orocos::Process.run 'hdpr_control', 'hdpr_pancam', 'hdpr_lidar', 'hdpr_tof', 'hdpr_bb2', 'hdpr_bb3', 'hdpr_imu', 'hdpr_gps', 'hdpr_temperature', 'hdpr_shutter_controller', 'hdpr_unit_gyro', 'hdpr_stereo', 'hdpr_tmtchandling', 'hdpr_vicon_unit', 'hdpr_shutter_controller_bumblebee', 'hdpr_unit_visual_odometry' do
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
   
    trigger_lidar = TaskContext.get 'trigger_lidar'

    dem_generation_lidar = TaskContext.get 'dem_generation_lidar'
    Orocos.conf.apply(dem_generation_lidar, ['velodyne'], :override => true)
    dem_generation_lidar.configure

    tofcamera_mesasr = TaskContext.get 'tofcamera_mesasr'
    Orocos.conf.apply(tofcamera_mesasr, ['default'], :override => true)
    tofcamera_mesasr.configure
    
    trigger_tof = TaskContext.get 'trigger_tof'

    dem_generation_tof = TaskContext.get 'dem_generation_tof'
    Orocos.conf.apply(dem_generation_tof, ['mesa'], :override => true)
    dem_generation_tof.configure
   
    imu_stim300 = TaskContext.get 'imu_stim300'
    #Orocos.conf.apply(imu_stim300, ['default', 'HDPR', 'ESTEC', 'stim300_5g'], :override => true)
    Orocos.conf.apply(imu_stim300, ['default', 'HDPR', 'Tenerife', 'stim300_5g'], :override => true)
    imu_stim300.configure

    # Setup visual odometry
    puts "Setting up visual odometry"
    visual_odometry = TaskContext.get 'viso2'
    Orocos.conf.apply(visual_odometry, ['bumblebee'], :override => true)
    Bundles.transformer.setup(visual_odometry)
    visual_odometry.configure

    viso2_with_imu = TaskContext.get 'viso2_with_imu'
    Orocos.conf.apply(viso2_with_imu, ['hdpr_autonomy'], :override => true)
    Bundles.transformer.setup(viso2_with_imu)
        viso2_with_imu.configure
        puts "done"


    
    if options[:v] == false
   	gps = TaskContext.get 'gps'
    	#Orocos.conf.apply(gps, ['HDPR', 'Netherlands', 'DECOS'], :override => true)
    	Orocos.conf.apply(gps, ['HDPR', 'Spain', 'Tenerife_Teleop'], :override => true)
    	gps.configure
    else
   	vicon = TaskContext.get 'vicon'
    	Orocos.conf.apply(vicon, ['default','hdpr'], :override => true)
    	vicon.configure
    end	

    gps_heading = TaskContext.get 'gps_heading'
    Orocos.conf.apply(gps_heading, ['default'], :override => true)
    gps_heading.configure

    if options[:bb2] == true
        puts "Startng BB2"
    
        camera_firewire_bb2 = TaskContext.get 'camera_firewire_bb2'
        Orocos.conf.apply(camera_firewire_bb2, ['bumblebee2'], :override => true)
        camera_firewire_bb2.configure
    
        camera_bb2 = TaskContext.get 'camera_bb2'
        Orocos.conf.apply(camera_bb2, ['default'], :override => true)
        camera_bb2.configure

        trigger_bb2 = TaskContext.get 'trigger_bb2'
       
        stereo_bb2 = TaskContext.get 'stereo_bb2'
        Orocos.conf.apply(stereo_bb2, ['hdpr_bb2'], :override => true)
        stereo_bb2.configure
    
        dem_generation_bb2 = TaskContext.get 'dem_generation_bb2'
        Orocos.conf.apply(dem_generation_bb2, ['hdpr_bb2'], :override => true)
        dem_generation_bb2.configure
    end

    if options[:bb3] == true
        puts "Startng BB3"
        
        camera_firewire_bb3 = TaskContext.get 'camera_firewire_bb3'
        Orocos.conf.apply(camera_firewire_bb3, ['bumblebee3'], :override => true)
        camera_firewire_bb3.configure
        
        camera_bb3 = TaskContext.get 'camera_bb3'
        Orocos.conf.apply(camera_bb3, ['default'], :override => true)
        camera_bb3.configure

        trigger_bb3 = TaskContext.get 'trigger_bb3'

        stereo_bb3 = TaskContext.get 'stereo_bb3'
        Orocos.conf.apply(stereo_bb3, ['hdpr_bb3_left_right'], :override => true)
        stereo_bb3.configure
    
        dem_generation_bb3 = TaskContext.get 'dem_generation_bb3'
        Orocos.conf.apply(dem_generation_bb3, ['hdpr_bb3'], :override => true)
        dem_generation_bb3.configure
    end
    
    pancam_left = Orocos.name_service.get 'pancam_left'
    Orocos.conf.apply(pancam_left, ['grashopper2_left'], :override => true)
    pancam_left.configure
    
    pancam_right = Orocos.name_service.get 'pancam_right'
    Orocos.conf.apply(pancam_right, ['grashopper2_right'], :override => true)
    pancam_right.configure

    trigger_pancam = TaskContext.get 'trigger_pancam'
    
    stereo_pancam = TaskContext.get 'stereo_pancam'
    Orocos.conf.apply(stereo_pancam, ['panCam'], :override => true)
    stereo_pancam.configure
    
    dem_generation_pancam = TaskContext.get 'dem_generation_pancam'
    Orocos.conf.apply(dem_generation_pancam, ['panCam'], :override => true)
    dem_generation_pancam.configure

    shutter_controller = Orocos.name_service.get 'shutter_controller'
    Orocos.conf.apply(shutter_controller, ['default'], :override => true)
    shutter_controller.configure

    shutter_controller_bb2 = Orocos.name_service.get 'shutter_controller_bb2'
    Orocos.conf.apply(shutter_controller_bb2, ['bb2tenerife'], :override => true)
    shutter_controller_bb2.configure

    shutter_controller_bb3 = Orocos.name_service.get 'shutter_controller_bb3'
    Orocos.conf.apply(shutter_controller_bb3, ['bb3tenerife'], :override => true)
    shutter_controller_bb3.configure
    
    #pancam_panorama = Orocos.name_service.get 'pancam_panorama'
    #Orocos.conf.apply(pancam_panorama, ['default'], :override => true)
    #pancam_panorama.configure
    
    panoramica = Orocos.name_service.get 'panoramica'
    Orocos.conf.apply(panoramica, ['default', 'separation_40_x'], :override => true)
    panoramica.configure
    
    trigger_panoramica = TaskContext.get 'trigger_panoramica'

    stereo_panoramica = TaskContext.get 'stereo_panoramica'
    Orocos.conf.apply(stereo_panoramica, ['panCam'], :override => true)
    stereo_panoramica.configure

    dem_generation_panoramica = TaskContext.get 'dem_generation_panoramica'
    Orocos.conf.apply(dem_generation_panoramica, ['panCam'], :override => true)
    dem_generation_panoramica.configure

    # Setup Waypoint_navigation 
    waypoint_navigation = Orocos.name_service.get 'waypoint_navigation'
    Orocos.conf.apply(waypoint_navigation, ['default','hdpr'], :override => true)
    waypoint_navigation.configure

    # Setup command arbiter
    command_arbiter = Orocos.name_service.get 'command_arbiter'
    Orocos.conf.apply(command_arbiter, ['default'], :override => true)
    command_arbiter.configure  
	    
    temperature = TaskContext.get 'temperature'
    Orocos.conf.apply(temperature, ['default'], :override => true)
    temperature.configure

    gyro = TaskContext.get 'dsp1760'
    Orocos.conf.apply(gyro, ['default'], :override => true)
    gyro.configure
 
    # setup telemetry_telecommand
    telemetry_telecommand = Orocos.name_service.get 'telemetry_telecommand'
    Orocos.conf.apply(telemetry_telecommand, ['default'], :override => true)
    Bundles.transformer.setup(telemetry_telecommand)
    telemetry_telecommand.configure
   
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
        camera_bb2.left_frame.connect_to                    trigger_bb2.frame_left_in
        camera_bb2.right_frame.connect_to                   trigger_bb2.frame_right_in
        trigger_bb2.frame_left_out.connect_to               stereo_bb2.left_frame
        trigger_bb2.frame_right_out.connect_to              stereo_bb2.right_frame
        trigger_bb2.frame_left_out.connect_to               dem_generation_bb2.left_frame_rect
        stereo_bb2.distance_frame.connect_to                dem_generation_bb2.distance_frame
        camera_firewire_bb2.frame.connect_to                shutter_controller_bb2.frame
        camera_firewire_bb2.shutter_value.connect_to        shutter_controller_bb2.shutter_value
        #stereo_bb2.left_frame_sync.connect_to               dem_generation_bb2.left_frame_rect
        #stereo_bb2.right_frame_sync.connect_to              dem_generation_bb2.right_frame_rect
    end
    if options[:bb3] == true
        camera_firewire_bb3.frame.connect_to                camera_bb3.frame_in
        camera_bb3.left_frame.connect_to                    trigger_bb3.frame_left_in
        camera_bb3.right_frame.connect_to                   trigger_bb3.frame_right_in
        trigger_bb3.frame_left_out.connect_to               stereo_bb3.left_frame
        trigger_bb3.frame_right_out.connect_to              stereo_bb3.right_frame
        trigger_bb3.frame_left_out.connect_to               dem_generation_bb3.left_frame_rect
        stereo_bb3.distance_frame.connect_to                dem_generation_bb3.distance_frame
        camera_firewire_bb3.frame.connect_to                shutter_controller_bb3.frame
        camera_firewire_bb3.shutter_value.connect_to        shutter_controller_bb3.shutter_value
        #stereo_bb3.left_frame_sync.connect_to               dem_generation_bb3.left_frame_rect
        #stereo_bb3.right_frame_sync.connect_to              dem_generation_bb3.right_frame_rect
   end
    
    # Waypoint navigation inputs:
    imu_stim300.orientation_samples_out.connect_to      gps_heading.imu_pose_samples    
    gyro.orientation_samples.connect_to      gps_heading.gyro_pose_samples    
    command_arbiter.motion_command.connect_to           gps_heading.motion_command
    telemetry_telecommand.locomotion_command.connect_to           gps_heading.motion_command

    #telemetry_telecommand.update_pose.connect_to viso2_with_imu.reset_pose

    if options[:v] == false
                camera_bb2.left_frame.connect_to                    visual_odometry.left_frame
                camera_bb2.right_frame.connect_to                   visual_odometry.right_frame
                imu_stim300.orientation_samples_out.connect_to          viso2_with_imu.pose_samples_imu
                visual_odometry.delta_pose_samples_out.connect_to       viso2_with_imu.delta_pose_samples_in
                gyro.orientation_samples.connect_to                             viso2_with_imu.pose_samples_imu_extra


    	viso2_with_imu.pose_samples_out.connect_to             telemetry_telecommand.current_pose
	puts "using gps"
    else
    	vicon.pose_samples.connect_to             	waypoint_navigation.pose
    	vicon.pose_samples.connect_to             	telemetry_telecommand.current_pose
	puts "using vicon"
    end

    # PanCam connections to panorama and 360 components (must function exclusively)
    #pancam_panorama.pan_angle_in.connect_to             ptu_directedperception.pan_angle
    #pancam_panorama.tilt_angle_in.connect_to            ptu_directedperception.tilt_angle
    #pancam_panorama.pan_angle_out.connect_to            ptu_directedperception.pan_set
    #pancam_panorama.tilt_angle_out.connect_to           ptu_directedperception.tilt_set
    #pancam_left.frame.connect_to                        pancam_panorama.left_frame_in
    #pancam_right.frame.connect_to                       pancam_panorama.right_frame_in
    panoramica.pan_angle_in.connect_to                  ptu_directedperception.pan_angle
    panoramica.tilt_angle_in.connect_to                 ptu_directedperception.tilt_angle
    panoramica.pan_angle_out.connect_to                 ptu_directedperception.pan_set
    panoramica.tilt_angle_out.connect_to                ptu_directedperception.tilt_set
    pancam_left.frame.connect_to                        panoramica.left_frame_in
    pancam_right.frame.connect_to                       panoramica.right_frame_in
    panoramica.left_frame_out.connect_to                trigger_panoramica.frame_left_in
    panoramica.right_frame_out.connect_to               trigger_panoramica.frame_right_in
    trigger_panoramica.frame_left_out.connect_to        stereo_panoramica.left_frame
    trigger_panoramica.frame_right_out.connect_to       stereo_panoramica.right_frame
    trigger_panoramica.frame_left_out.connect_to        dem_generation_panoramica.left_frame_rect            
    stereo_panoramica.distance_frame.connect_to         dem_generation_panoramica.distance_frame
    #stereo_panoramica.left_frame_sync.connect_to        dem_generation_panoramica.left_frame_rect
    #stereo_panoramica.right_frame_sync.connect_to       dem_generation_panoramica.right_frame_rect
    dem_generation_panoramica.sync_out.connect_to	    panoramica.sync_in

    # PanCam connections to shutter controller
    pancam_left.frame.connect_to                        shutter_controller.frame
    pancam_left.shutter_value.connect_to                shutter_controller.shutter_value
    pancam_right.shutter_value.connect_to               shutter_controller.shutter_value
    pancam_left.frame.connect_to                        trigger_pancam.frame_left_in
    pancam_right.frame.connect_to                       trigger_pancam.frame_right_in
    trigger_pancam.frame_left_out.connect_to            stereo_pancam.left_frame
    trigger_pancam.frame_right_out.connect_to           stereo_pancam.right_frame
    trigger_pancam.frame_left_out.connect_to            dem_generation_pancam.left_frame_rect            
    stereo_pancam.distance_frame.connect_to             dem_generation_pancam.distance_frame
    #stereo_pancam.left_frame_sync.connect_to            dem_generation_pancam.left_frame_rect
    #stereo_pancam.right_frame_sync.connect_to           dem_generation_pancam.right_frame_rect

    # ToF connections
    tofcamera_mesasr.ir_frame.connect_to                trigger_tof.frame_left_in
    #tofcamera_mesasr.distance_frame.connect_to          trigger_tof.distance_frame_in
    tofcamera_mesasr.pointcloud.connect_to              trigger_tof.pointcloud_in
    trigger_tof.frame_left_out.connect_to               dem_generation_tof.left_frame_rect
    trigger_tof.distance_frame_out.connect_to           dem_generation_tof.range_interp_frame
    trigger_tof.pointcloud_out.connect_to               dem_generation_tof.pointcloud

    # Lidar connections
    velodyne_lidar.ir_interp_frame.connect_to           trigger_lidar.frame_left_in
    #velodyne_lidar.range_interp_frame.connect_to        trigger_lidar.distance_frame_in
    velodyne_lidar.laser_scans.connect_to               trigger_lidar.laser_scan_in
    trigger_lidar.frame_left_out.connect_to             dem_generation_lidar.left_frame_rect
    trigger_lidar.distance_frame_out.connect_to         dem_generation_lidar.range_interp_frame
    trigger_lidar.laser_scan_out.connect_to             dem_generation_lidar.laser_scans
   
    # Telemetry Telecommand connections
    telemetry_telecommand.locomotion_command.connect_to locomotion_control.motion_command
    telemetry_telecommand.mast_pan.connect_to           ptu_directedperception.pan_set
    telemetry_telecommand.mast_tilt.connect_to          ptu_directedperception.tilt_set
    telemetry_telecommand.trajectory.connect_to         waypoint_navigation.trajectory
    telemetry_telecommand.trajectory_speed.connect_to   waypoint_navigation.speed_input
    #waypoint_navigation.motion_command.connect_to locomotion_control.motion_command # uncomment if command_arbitrer is removed
    waypoint_navigation.trajectory_status.connect_to    telemetry_telecommand.trajectory_status
    telemetry_telecommand.current_pan.connect_to        ptu_directedperception.pan_angle
    telemetry_telecommand.current_tilt.connect_to       ptu_directedperception.tilt_angle
#    telemetry_telecommand.current_imu.connect_to        imu_stim300.orientation_samples_out
    read_joint_dispatcher.joints_samples.connect_to     telemetry_telecommand.joint_samples
    temperature.temperature_samples.connect_to          telemetry_telecommand.motor_temperatures

    telemetry_telecommand.mast_trigger.connect_to       trigger_pancam.telecommand_in
    telemetry_telecommand.front_trigger.connect_to      trigger_bb3.telecommand_in
    telemetry_telecommand.haz_front_trigger.connect_to  trigger_bb2.telecommand_in
    telemetry_telecommand.tof_trigger.connect_to        trigger_tof.telecommand_in
    telemetry_telecommand.lidar_trigger.connect_to      trigger_lidar.telecommand_in
    telemetry_telecommand.panoramica_trigger.connect_to trigger_panoramica.telecommand_in
    telemetry_telecommand.panorama_tilt.connect_to      panoramica.trigger_tilt

    trigger_lidar.telecommands_out.connect_to           dem_generation_lidar.telecommands_in
    trigger_tof.telecommands_out.connect_to             dem_generation_tof.telecommands_in
    trigger_bb2.telecommands_out.connect_to             dem_generation_bb2.telecommands_in
    trigger_bb3.telecommands_out.connect_to             dem_generation_bb3.telecommands_in
    trigger_pancam.telecommands_out.connect_to          dem_generation_pancam.telecommands_in
    trigger_panoramica.telecommands_out.connect_to      dem_generation_panoramica.telecommands_in
    dem_generation_pancam.telemetry_out.connect_to      telemetry_telecommand.telemetry_product, :type => :buffer, :size => 10
    dem_generation_panoramica.telemetry_out.connect_to  telemetry_telecommand.telemetry_product, :type => :buffer, :size => 10
    dem_generation_bb3.telemetry_out.connect_to         telemetry_telecommand.telemetry_product, :type => :buffer, :size => 10
    dem_generation_bb2.telemetry_out.connect_to         telemetry_telecommand.telemetry_product, :type => :buffer, :size => 10
    dem_generation_tof.telemetry_out.connect_to         telemetry_telecommand.telemetry_product, :type => :buffer, :size => 10
    dem_generation_lidar.telemetry_out.connect_to       telemetry_telecommand.telemetry_product, :type => :buffer, :size => 20

    # Configure the sensor trigger after the ports are connected
    trigger_lidar.configure
    trigger_tof.configure
    trigger_bb2.configure
    trigger_bb3.configure
    trigger_pancam.configure
    trigger_panoramica.configure

    # Log all the properties of the components
    Orocos.log_all_configuration
    
    # Define loggers
    logger_control = Orocos.name_service.get 'hdpr_control_Logger'
    logger_control.file = "control.log"
    logger_control.log(platform_driver.joints_readings)
    logger_control.log(command_arbiter.motion_command)
    
    logger_pancam = Orocos.name_service.get 'hdpr_pancam_Logger'
    logger_pancam.file = "pancam.log"
    #logger_pancam.log(pancam_panorama.left_frame_out)
    #logger_pancam.log(pancam_panorama.right_frame_out)
    #logger_pancam.log(pancam_panorama.pan_angle_out_degrees)
    #logger_pancam.log(pancam_panorama.tilt_angle_out_degrees)
    #logger_pancam.log(pancam_360.left_frame_out)
    #logger_pancam.log(pancam_360.right_frame_out)
    #logger_pancam.log(pancam_360.pan_angle_out_degrees)
    #logger_pancam.log(pancam_360.tilt_angle_out_degrees)
    #logger_pancam.log(pancam_360.set_id)
    logger_pancam.log(shutter_controller.shutter_value)
    
    if options[:bb2] == true
        logger_bb2 = Orocos.name_service.get 'hdpr_bb2_Logger'
        logger_bb2.file = "bb2.log"
        logger_bb2.log(camera_firewire_bb2.frame)
        logger_bb2.log(shutter_controller_bb2.shutter_value)
    end
    
    if options[:bb3] == true
        logger_bb3 = Orocos.name_service.get 'hdpr_bb3_Logger'
        logger_bb3.file = "bb3.log"
        logger_bb3.log(camera_firewire_bb3.frame)
        logger_bb3.log(shutter_controller_bb3.shutter_value)
    end

    logger_tof = Orocos.name_service.get 'hdpr_tof_Logger'
    logger_tof.file = "tof.log"
    logger_tof.log(tofcamera_mesasr.distance_frame)
    logger_tof.log(tofcamera_mesasr.ir_frame)
    logger_tof.log(tofcamera_mesasr.pointcloud)
    logger_tof.log(tofcamera_mesasr.tofscan)
    
    logger_lidar = Orocos.name_service.get 'hdpr_lidar_Logger'
    logger_lidar.file = "lidar.log"
    logger_lidar.log(velodyne_lidar.ir_frame)
    logger_lidar.log(velodyne_lidar.laser_scans)
    logger_lidar.log(velodyne_lidar.range_frame)
    logger_lidar.log(velodyne_lidar.velodyne_time)
    logger_lidar.log(velodyne_lidar.accumulated_velodyne_time)
    logger_lidar.log(velodyne_lidar.estimated_clock_offset)

    if options[:v] == false        
    	logger_gps = Orocos.name_service.get 'hdpr_gps_Logger'
    	logger_gps.file = "gps.log"
    	logger_gps.log(gps.pose_samples)
    	logger_gps.log(gps.raw_data)
    	logger_gps.log(gps.time)
    	logger_gps.log(gps_heading.pose_samples_out)
    end

    logger_imu = Orocos.name_service.get 'hdpr_imu_Logger'
    logger_imu.file = "imu.log"
    logger_imu.log(imu_stim300.inertial_sensors_out)
    logger_imu.log(imu_stim300.temp_sensors_out)
    logger_imu.log(imu_stim300.orientation_samples_out)
    logger_imu.log(imu_stim300.compensated_sensors_out)

    logger_temperature = Orocos.name_service.get 'hdpr_temperature_Logger'
    logger_temperature.file = "temperature.log"
    logger_temperature.log(temperature.temperature_samples)

    #logger_gyro = Orocos.name_service.get 'hdpr_unit_gyro_Logger'
    #logger_gyro.file = "gyro.log"
    #logger_gyro.log(gyro.rotation)
    #logger_gyro.log(gyro.orientation_samples)
    #logger_gyro.log(gyro.bias_samples)
    #logger_gyro.log(gyro.bias_values)

    # No loggers needed for triggers, stereo, dem_generation or telemetry_telecommand

    #Orocos.log_all_ports
    
    # Start loggers
#    logger_control.start
#    logger_pancam.start
    if options[:bb2] == true
#        logger_bb2.start
    end
    if options[:bb3] == true
#        logger_bb3.start
    end
#    logger_tof.start
#    logger_lidar.start
#    logger_gps.start
#    logger_imu.start
#    logger_temperature.start
#    logger_gyro.start

    # Start the components
    platform_driver.start
    read_joint_dispatcher.start
    command_joint_dispatcher.start
    locomotion_control.start
    ptu_directedperception.start
    command_arbiter.start
    pancam_left.start
    pancam_right.start
    shutter_controller.start
    trigger_pancam.start
    stereo_pancam.start
    dem_generation_pancam.start
    panoramica.start
    trigger_panoramica.start
    stereo_panoramica.start
    dem_generation_panoramica.start
    motion_translator.start
    joystick.start
    velodyne_lidar.start
    trigger_lidar.start
    dem_generation_lidar.start
    tofcamera_mesasr.start
    trigger_tof.start
    dem_generation_tof.start
    imu_stim300.start
    gyro.start
    temperature.start
    if options[:v] == false
    	gps.start
    else
	vicon.start
    end
    gps_heading.start
    if options[:bb2] == true
        camera_bb2.start
        camera_firewire_bb2.start
        trigger_bb2.start
        stereo_bb2.start
        dem_generation_bb2.start
        shutter_controller_bb2.start
    end
    if options[:bb3] == true
        camera_bb3.start
        camera_firewire_bb3.start
        trigger_bb3.start
        stereo_bb3.start
        dem_generation_bb3.start
        shutter_controller_bb3.start
    end
    telemetry_telecommand.start
    visual_odometry.start
    viso2_with_imu.start

    # Race condition with internal gps_heading states. This check is here to only trigger the 
    # trajectoryGen when the pose has been properly initialised. Otherwise the trajectory is set wrong.
    #puts "Move rover forward to initialise the gps_heading component"
    #while gps_heading.ready == false
    #   sleep 1
    #end
    #puts "GPS heading calibration done"

    # Trigger the trojectory generation, waypoint_navigation must be running at this point
    waypoint_navigation.start
            
    Readline::readline("Press Enter to exit\n") do
    end
end 


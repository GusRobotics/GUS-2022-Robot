// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// General Resources
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CAN;

// Solenoids
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;

// Data Display Tools
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Rev Resources
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// Cross the Road Resources
import com.ctre.phoenix.sensors.PigeonIMU;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.ControlMode;

// Camera Stuff
import edu.wpi.first.cameraserver.CameraServer;

// Limelight
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {

  // Auto options
  private static final String kDefaultAuto = "Two Ball Auto";
  private static final String kTwoBallExtraDistance = "Two Ball Extra Distance Auto";
  private static final String kFourBall = "Four Ball Auto";
  private static final String kShootInPlace = "Shoot in Place";
  private static final String kPidTuneAuto = "PID Tune Auto";
  private static final String kPIDTurnAuto = "Turn Test Auto";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  private double shooter_power_1;
  private double shooter_power_2;
  private final SendableChooser<Double> power_chooser = new SendableChooser<>();
  private final SendableChooser<Double> power_chooser_2 = new SendableChooser<>();

  // Robot Mechanism Status Variables
   private boolean index_on = false;
   private boolean shooter_on = false;
   private boolean shooter_high = false; 

   // General time stamp in global scope because of iterative stuff
   private double time_stamp = 0;

  // INITIALIZE ELECTRONICS
  // Controller
  private static XboxController joy_base = new XboxController(0);
  private static XboxController joy_co = new XboxController(1);
  private static XboxController joy_climb = new XboxController(2);

  // Add PDB for data reading (optional)
  //PowerDistributionPanel examplePD = new PowerDistribution(0, ModuleType.kAutomatic);

  // Initialize all motors
  CANSparkMax m_drive_left = new CANSparkMax(config.drive_left1_ID, MotorType.kBrushless);
  CANSparkMax m_drive_left2 = new CANSparkMax(config.drive_left2_ID, MotorType.kBrushless);
  CANSparkMax m_drive_left3 = new CANSparkMax(config.drive_left3_ID, MotorType.kBrushless);
  CANSparkMax m_drive_right = new CANSparkMax(config.drive_right1_ID, MotorType.kBrushless);
  CANSparkMax m_drive_right2 = new CANSparkMax(config.drive_right2_ID, MotorType.kBrushless);
  CANSparkMax m_drive_right3 = new CANSparkMax(config.drive_right3_ID, MotorType.kBrushless);
  CANSparkMax m_shooter = new CANSparkMax(config.shooter1_ID, MotorType.kBrushless);
  CANSparkMax m_shooter2 = new CANSparkMax(config.shooter2_ID, MotorType.kBrushless);
  CANSparkMax m_index = new CANSparkMax(config.index_ID, MotorType.kBrushless);
  CANSparkMax m_intake = new CANSparkMax(config.intake_ID, MotorType.kBrushless);
  CANSparkMax m_climber_left = new CANSparkMax(config.climber_left_ID, MotorType.kBrushless);
  CANSparkMax m_climber_right = new CANSparkMax(config.climber_right_ID, MotorType.kBrushless);

  // Initialize all sensors 
  PigeonIMU gyro = new PigeonIMU(config.pigeon_ID);
  AnalogInput dist_sensor_1 = new AnalogInput(config.index_dist_sensor_channel);
  NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

  // Initialize pneumatic system
  Compressor compressor = new Compressor(config.pcm_ID, PneumaticsModuleType.CTREPCM);
  Solenoid drive_gear_shift = new Solenoid(config.pcm_ID, PneumaticsModuleType.CTREPCM, config.drive_channel);
  Solenoid intake_actuator = new Solenoid(config.pcm_ID, PneumaticsModuleType.CTREPCM, config.intake_channel);
  Solenoid climber_actuator = new Solenoid(config.pcm_ID, PneumaticsModuleType.CTREPCM, config.climber_channel);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Auto chooser
    m_chooser.setDefaultOption("Two ball Auto", kDefaultAuto);
    m_chooser.setDefaultOption("Two Ball Extra Distance Auto", kTwoBallExtraDistance);
    m_chooser.addOption("Four Ball", kFourBall);
    m_chooser.addOption("Shoot in Place", kShootInPlace);
    m_chooser.addOption("PID Tuner", kPidTuneAuto);
    m_chooser.addOption("Turn Test", kPIDTurnAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // First Cycle Power Chooser
    power_chooser.setDefaultOption("Low Power", config.low_shot_power);
    power_chooser.addOption("High Power", config.high_shot_power);
    SmartDashboard.putData("Cycle 1 Shot", power_chooser);

    // Second Cycle Power Chooser
    power_chooser_2.setDefaultOption("Low Power", config.low_shot_power);
    power_chooser_2.addOption("High Power", config.high_shot_power);
    SmartDashboard.putData("Cycle 2 Shot", power_chooser_2);

    // Set parallel motors to follow the leader
    m_drive_left2.follow(m_drive_left);
    m_drive_left3.follow(m_drive_left);
    m_drive_right2.follow(m_drive_right);
    m_drive_right3.follow(m_drive_right);
    // m_shooter2.follow(m_shooter);

    // For all motors, reset to factory defaults
    m_drive_left.restoreFactoryDefaults();
    m_drive_left2.restoreFactoryDefaults();
    m_drive_left3.restoreFactoryDefaults();
    m_drive_right.restoreFactoryDefaults();
    m_drive_right2.restoreFactoryDefaults();
    m_drive_right3.restoreFactoryDefaults();
    m_shooter.restoreFactoryDefaults();
    m_shooter2.restoreFactoryDefaults();
    m_index.restoreFactoryDefaults();
    m_climber_left.restoreFactoryDefaults();
    m_climber_right.restoreFactoryDefaults();

    // Invert backwards motors
    m_drive_right.setInverted(true);
    m_drive_right2.setInverted(true);
    m_drive_right3.setInverted(true);
    // m_climber_right.setInverted(true);
    // m_climber_left.setInverted(true);

    // Set current limits
    m_drive_left.setSmartCurrentLimit(config.drive_current_limit);
    m_drive_left2.setSmartCurrentLimit(config.drive_current_limit);
    m_drive_left3.setSmartCurrentLimit(config.drive_current_limit);
    m_drive_right.setSmartCurrentLimit(config.drive_current_limit);
    m_drive_right2.setSmartCurrentLimit(config.drive_current_limit);
    m_drive_right3.setSmartCurrentLimit(config.drive_current_limit);
    m_shooter.setSmartCurrentLimit(config.shooter_current_limit);
    m_shooter2.setSmartCurrentLimit(config.shooter_current_limit);
    m_intake.setSmartCurrentLimit(config.intake_current_limit);
    m_index.setSmartCurrentLimit(config.index_current_limit);
    m_climber_left.setSmartCurrentLimit(config.climber_current_limit);
    m_climber_right.setSmartCurrentLimit(config.climber_current_limit);

    // Set index and intake to brake
    m_index.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_climber_left.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_climber_right.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Start the compressor- this is the only thing needed for the compressor
    compressor.enableDigital();

    // Limelight settings
    limelight.getEntry("camMode").setNumber(1);
    limelight.getEntry("ledMode").setNumber(1);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */

  PrecisionDrive auto_drive;
  int auto_stage;
  double start_time;

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    shooter_power_1 = power_chooser.getSelected();
    shooter_power_2 = power_chooser.getSelected();


    auto_drive = new PrecisionDrive(m_drive_left, m_drive_right, gyro);
    auto_stage = 0;

    // Set robot in high gear
    drive_gear_shift.set(false);

    start_time = Timer.getFPGATimestamp();
  }

  boolean done = false;
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("Auto Stage", auto_stage);
    SmartDashboard.putNumber("Auto Time", Timer.getFPGATimestamp() - start_time);
    if(Timer.getFPGATimestamp() - start_time <= 15) {
    switch (m_autoSelected) {
      case kPidTuneAuto:
        SmartDashboard.putString("Mode", "Test");
        // Put custom auto code here
        switch(auto_stage) {
          case 0:
            auto_drive.setDistance(3);
            auto_stage++;
            break;
          case 1:
            if(auto_drive.pidStraight()) {
              auto_stage++;
            }
            SmartDashboard.putString("Test Status", "In Progress");
            break;
          default:
            SmartDashboard.putString("Test Status", "Done :)");
            m_drive_left.set(0);
            m_drive_right.set(0);
            break;
        }
        break;

      case kPIDTurnAuto:
        switch(auto_stage) {
          case 0:
            auto_drive.setAngle(90);
            auto_stage++;
            break;
          case 1:
            if(auto_drive.pidTurn()) {
              auto_stage++;
            }
            break;
          default:
            SmartDashboard.putString("Test Status", "Done");

        }
        break;
      
      case kDefaultAuto:
        // Two ball auto
        switch(auto_stage) {
          case 0:
            // Actuate and start intake
            intake_actuator.set(true);
            m_intake.set(1);

            auto_drive.setDistance(3);
            auto_stage++;
            break;

          case 1:
            // Go back and get ball, start to rev shooter
            if(auto_drive.pidStraight()) {
              time_stamp = Timer.getFPGATimestamp();
              auto_drive.setDistance(-5.9);
              m_shooter.set(shooter_power_1);
              m_shooter2.set(shooter_power_1);
              m_intake.set(0);
              auto_stage++;
            }
            break;
          
          case 2:
            // Drive towards goal
            if(auto_drive.pidStraight()) {
              time_stamp = Timer.getFPGATimestamp();
              auto_drive.stop();
              m_index.set(1);
              auto_stage++;
            }

            if(Timer.getFPGATimestamp() - time_stamp > 0.5) {
              intake_actuator.set(false);
            }
            break;

          case 3:
            // Index to shoot
            if(Timer.getFPGATimestamp() - time_stamp > 0.65) {
              auto_drive.setDistance(0);
              m_index.set(0);
              m_shooter.set(0);
              m_shooter2.set(0);
              auto_stage++;
            }
            break;

          default:
            // Finished
            SmartDashboard.putString("Status", "Done");
            auto_drive.stop();
            m_index.set(0);
            m_shooter.set(0);
            m_shooter2.set(0);
            break;
          }
        break;

        case kTwoBallExtraDistance:
        // Two ball auto
        switch(auto_stage) {
          case 0:
            // Actuate and start intake
            intake_actuator.set(true);
            m_intake.set(1);

            auto_drive.setDistance(4);
            auto_stage++;
            break;

          case 1:
            // Go back and get ball, start to rev shooter
            if(auto_drive.pidStraight()) {
              time_stamp = Timer.getFPGATimestamp();

              if(shooter_power_1 == config.high_shot_power) {
                // Drive back to line for high ball
                auto_drive.setDistance(-1);
              }
              else {
                // Back to fender for low ball
                auto_drive.setDistance(-6.9);
              }
              
              m_shooter.set(shooter_power_1);
              m_shooter2.set(shooter_power_1);
              m_intake.set(0);
              auto_stage++;
            }
            break;
          
          case 2:
            // Drive towards goal
            if(auto_drive.pidStraight()) {
              time_stamp = Timer.getFPGATimestamp();
              auto_drive.stop();
              auto_stage++;
            }

            if(Timer.getFPGATimestamp() - time_stamp > 0.5) {
              intake_actuator.set(false);
            }
            break;

          case 3:
            // Delay
            if(shooter_power_1 == config.high_shot_power) {
              if(Timer.getFPGATimestamp() - time_stamp > 1) {
                time_stamp = Timer.getFPGATimestamp();
                m_index.set(0.5);
                auto_stage++;
              }
            }
            else {
              time_stamp = Timer.getFPGATimestamp();
              m_index.set(0.5);
              auto_stage++;
            }
            break;

          case 4:
            // Index to shoot
            if(Timer.getFPGATimestamp() - time_stamp > 0.3) {
              time_stamp = Timer.getFPGATimestamp();
              auto_drive.setDistance(0);
              m_index.set(0);
              auto_stage++;
            }
            break;

          case 5:
            // Fire
            if(Timer.getFPGATimestamp() - time_stamp > 2) {
              m_index.set(0.5);
            }
            if(Timer.getFPGATimestamp() - time_stamp > 3) {
              m_index.set(0);
              m_shooter.set(0);
              m_shooter2.set(0);
              auto_stage++;
            }
            break;

          default:
            // Finished
            SmartDashboard.putString("Status", "Done");
            auto_drive.stop();
            m_index.set(0);
            m_shooter.set(0);
            m_shooter2.set(0);
            break;
          }
        break;

      case kFourBall:
        // Four ball auto
        switch(auto_stage) {
          case 0:
            // Actuate and start intake
            intake_actuator.set(true);
            m_intake.set(1);

            auto_drive.setDistance(3);
            auto_stage++;
            break;

          case 1:
            // Go back and get ball
            if(auto_drive.pidStraight()) {
              time_stamp = Timer.getFPGATimestamp();
              auto_drive.setDistance(-5.4);
              m_shooter.set(shooter_power_1);
              m_shooter2.set(shooter_power_1);
              m_intake.set(0);
              auto_stage++;
            }
            break;
          
          case 2:
            // Drive towards goal
            if(auto_drive.pidStraight()) {
              time_stamp = Timer.getFPGATimestamp();
              auto_drive.stop();
              m_index.set(1);
              auto_stage++;
            }

            if(Timer.getFPGATimestamp() - time_stamp > 0.5) {
              intake_actuator.set(false);
            }
            break;

          case 3:
            // Shoot
            if(Timer.getFPGATimestamp() - time_stamp > 0.65) {
              auto_drive.setDistance(0);
              m_index.set(0);
              m_shooter.set(0);
              m_shooter2.set(0);
              auto_stage++;
            }
            break;

          case 4:
            // Drive back
            if(auto_drive.pidStraight()) {
              drive_gear_shift.set(true);
              auto_drive.setAngle(-75);
              auto_drive.stop();
              auto_stage++;
            }
            break;
          
          case 5:
            // Turn clockwise
            if(auto_drive.pidTurn()) {
              drive_gear_shift.set(false);
              auto_drive.setDistance(17);
              m_intake.set(1);
              intake_actuator.set(true);
              auto_stage++;
            }
            break;
            
        case 6:
          // Go forward to collect two balls, index while driving
          if(auto_drive.pidStraight()) {
            time_stamp = Timer.getFPGATimestamp();

            if(shooter_power_2 == config.low_shot_power) {
              auto_drive.setDistance(-14.25);
            }
            else {
              auto_drive.setDistance(-9);
            }
            auto_drive.setAllowedError(0.3);
            m_index.set(0);
            auto_stage++;
          }
          else if(dist_sensor_1.getValue() < config.dist1_threshold) {
            m_index.set(config.index_power);
          }
          else {
            m_index.set(0);
          }
          break;
        
        case 7:
          // Go back
          if(auto_drive.pidStraight()) {
            drive_gear_shift.set(true);

            if(shooter_power_2 == config.low_shot_power) {
              auto_drive.setAngle(50);
            }
            else {
              auto_drive.setAngle(45);
            }
            m_intake.set(0);
            m_index.set(0);
            auto_stage++;
          }
          else if(Timer.getFPGATimestamp() - time_stamp > 2) {
            m_intake.set(0);
            intake_actuator.set(false);
            m_shooter.set(shooter_power_2);
            m_shooter2.set(shooter_power_2);
          }
          else if(shooter_power_2 == config.high_shot_power) {
            m_shooter.set(shooter_power_2);
            m_shooter2.set(shooter_power_2);
          }

          if(dist_sensor_1.getValue() < config.dist1_threshold) {
            m_index.set(config.index_power);
          }
          else {
            m_index.set(0);
          }
          break;

        case 8:
          // Turn counterclockwise
          if(auto_drive.pidTurn()) {
            drive_gear_shift.set(false);
            time_stamp = Timer.getFPGATimestamp();
            m_drive_left.set(0.15);
            m_drive_right.set(0.15);
            // m_index.set(1);
            auto_stage++;
          }
          break;
        
        case 9:
          // Shoot
          if(shooter_power_2 == config.low_shot_power && Timer.getFPGATimestamp() - time_stamp > 2) {
            m_index.set(0);
            m_shooter.set(0);
            m_shooter2.set(0);
            auto_stage++;
          }
          else if(shooter_power_2 == config.high_shot_power && Timer.getFPGATimestamp() - time_stamp > 0.3) {
            auto_drive.stop();
            m_index.set(0);
            auto_stage++;
          }
          else if(Timer.getFPGATimestamp() - time_stamp > 0.2) {
            auto_drive.stop();
          }
          break;

        case 10:
          // Fire
          if(Timer.getFPGATimestamp() - time_stamp > 2) {
            m_index.set(0.5);
          }
          if(Timer.getFPGATimestamp() - time_stamp > 3) {
            m_index.set(0);
            m_shooter.set(0);
            m_shooter2.set(0);
            auto_stage++;
          }
          break;

        default:
          // Finished
          SmartDashboard.putString("Status", "Done");
          auto_drive.stop();
          m_index.set(0);
          m_shooter.set(0);
          m_shooter2.set(0);
          break;
        }
      break;

      case kShootInPlace:
        switch(auto_stage){
          case 0:
            m_shooter.set(shooter_power_1);
            m_shooter2.set(shooter_power_1);

            if(Timer.getFPGATimestamp() - start_time > 3) {
              time_stamp = Timer.getFPGATimestamp();
              auto_stage++;
            }
            break;
          
          case 1:
            m_index.set(1);

            if(Timer.getFPGATimestamp() - time_stamp > 2) {
              m_shooter.set(0);
              m_shooter2.set(0);
              m_index.set(0);
              auto_drive.setDistance(8);
              auto_stage++;
            }
            break;

          case 2:
            if(auto_drive.pidStraight()) {
              auto_drive.stop();
              auto_stage++;
            }
            break;
          
          default:
            auto_drive.stop();
            m_index.set(0);
            m_intake.set(0);
            m_shooter.set(0);
            m_shooter2.set(0);
            break;
        }
        break;

      default:
        SmartDashboard.putString("!", "No Auto");
        break;
    }
  }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    time_stamp = Timer.getFPGATimestamp();

    start_time = Timer.getFPGATimestamp();

    limelight.getEntry("ledMode").setNumber(1);
    m_index.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // Limelight on
    limelight.getEntry("camMode").setNumber(0);
    limelight.getEntry("ledMode").setNumber(3);

    double limelight_target = limelight.getEntry("tv").getDouble(0.0);
    SmartDashboard.putNumber("Target detected", limelight_target);


    double limelight_x = limelight.getEntry("tx").getDouble(0.0);
    double limelight_y = limelight.getEntry("ty").getDouble(0.0);

    SmartDashboard.putNumber("Target x", limelight_x);
    SmartDashboard.putNumber("Target y", limelight_y);

    double range = 2;
    double kP = 0.04;

    if(Math.abs(limelight_x) < range) {
      SmartDashboard.putBoolean("Lined Up", true);
    }
    else {
      SmartDashboard.putBoolean("Lined Up", false);
    }
    

    SmartDashboard.putNumber("Time Remaining", 135 - (Timer.getFPGATimestamp() - start_time));
   
    // Run Drive (Choose controller in use and run tank drive with thresolds)
    if(joy_base.getXButton() && Math.abs(limelight_x) > range) {
      m_drive_left.set(-limelight_x * kP);
      m_drive_right.set(limelight_x * kP);
    }
    else if(Math.abs(joy_base.getRightY()) > config.controller_threshold || Math.abs(joy_base.getLeftY()) > config.controller_threshold) {
      // Drive controller
      if(Math.abs(joy_base.getLeftY()) > config.controller_threshold) {
        m_drive_left.set(joy_base.getLeftY());
      }
      else {
        m_drive_left.set(0);
      }
      if(Math.abs(joy_base.getRightY()) > config.controller_threshold) {
        m_drive_right.set(joy_base.getRightY());
      }
      else {
        m_drive_right.set(0);
      }
    }
    else if(Math.abs(joy_climb.getRightY()) > config.controller_threshold || Math.abs(joy_base.getLeftY()) > config.controller_threshold){
      // Climb controller
      if(Math.abs(joy_climb.getLeftY()) > config.controller_threshold) {
        m_drive_left.set(joy_climb.getLeftY());
      }
      else {
        m_drive_left.set(0);
      }
      if(Math.abs(joy_climb.getRightY()) > config.controller_threshold) {
        m_drive_right.set(joy_climb.getRightY());
      }
      else {
        m_drive_right.set(0);
      }
    }
    else {
      m_drive_left.set(0);
      m_drive_right.set(0);
    }

    // Drive Shift (Hold for high gear, default is low gear)
    if(joy_base.getRightTriggerAxis() > 0.8) {
      drive_gear_shift.set(false);
    }
    else {
      drive_gear_shift.set(true);
    }

    // Intake Actuation (Hold for extension, default is retracted)
    if (joy_base.getLeftTriggerAxis() > 0.8) {
      intake_actuator.set(true);
    }
    else {
      intake_actuator.set(false);
    }
    
    // Intake (Hold Left Top for intake, Hold back for outtake)
    if(joy_co.getLeftBumper()) {
      m_intake.set(1);
    }
    else if(joy_co.getBackButton()) {
      m_intake.set(-1);
    }
    else {
      m_intake.set(0);
    }
   
    // Shooter (Deterine power from button)
    if(joy_co.getLeftTriggerAxis() > 0.8) {
      // Low Power
      m_shooter.set(config.low_shot_power);
      m_shooter2.set(config.low_shot_power);
      shooter_on = true;
      shooter_high = false;
    }
    else if(joy_co.getRightTriggerAxis() > 0.8) {
      // High Power
      m_shooter.set(config.high_shot_power);
      m_shooter2.set(config.high_shot_power);
      shooter_on = true;
      shooter_high = true;
    }
    else if(joy_co.getAButton()) {
      m_shooter.set(config.high_shot_far_power);
      m_shooter2.set(config.high_shot_far_power);
      shooter_on = true;
      shooter_high = true;
    }
    else {
      m_shooter.set(0);
      m_shooter2.set(0);
      shooter_on = false;
    }

    SmartDashboard.putNumber("Distance Sensor 1 Value", dist_sensor_1.getValue());

    // Index (Auto index brings ball from intake to before the shooter. Only index from that point if the shooter is running)
    if(joy_base.getLeftBumper()) {
      // Base Controls
      if(dist_sensor_1.getValue() < config.dist1_threshold) {
        // Index freely when the sensor is not triggered by a ball
        m_index.set(config.index_power);
      }
      else if (shooter_on) {
        // Index freely when the shooter is on with the high shot exception
        if(shooter_high) {
          m_index.set(config.high_shot_index_power);
        }
        else {
          m_index.set(config.index_power);
        }
        
      }
      else {
        // Do not index if the sensor is not triggered and the shooter is not on
        m_index.set(0);
      }
    }
    else if(joy_co.getPOV() == 180) {
     // Co - Reverse
      m_index.set(-1); 
      index_on = false;
    }
    else if(joy_co.getYButton() && dist_sensor_1.getValue() < config.dist1_threshold && !index_on) {
      index_on = true;
    }
    else if(index_on) {
      if(dist_sensor_1.getValue() >= config.dist1_threshold) {
        index_on = false;
      }
      else {
        m_index.set(1);
      }
    }
    else {
      m_index.set(0);
    }
    SmartDashboard.putBoolean("Index on", index_on);

    // Right Climber Arm
    if(joy_climb.getLeftBumper()) {
      m_climber_right.set(1);
    }
    else if(joy_climb.getLeftTriggerAxis() > 0.8) {
      m_climber_right.set(-1);
    }
    else {
      m_climber_right.set(0);
    }

    // Left Climber Arm
    if(joy_climb.getRightBumper()) {
      m_climber_left.set(1);
    }
    else if(joy_climb.getRightTriggerAxis() > 0.8) {
      m_climber_left.set(-1);
    }
    else {
      m_climber_left.set(0);
    }

    // Pneumatics
    if (joy_climb.getBButton()) {
      climber_actuator.set(true);
    }
    else {
      climber_actuator.set(false);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    limelight.getEntry("camMode").setNumber(1);
    limelight.getEntry("ledMode").setNumber(1);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    SmartDashboard.putNumber("Distance Sensor 1 Value", dist_sensor_1.getValue());
    limelight.getEntry("camMode").setNumber(1);
    limelight.getEntry("ledMode").setNumber(1);
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
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
  private static final String kTwoBallLowAuto = "Two Ball Low Auto";
  private static final String kTwoBallHighAuto = "Two Ball Extra Distance Auto";
  private static final String kFourBallHighAuto = "Four Ball Auto";
  private static final String kShootInPlace = "Shoot in Place";
  private static final String kPidTuneAuto = "PID Tune Auto";
  private static final String kPIDTurnAuto = "Turn Test Auto";

  private String m_auto_selected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private String start_position;
  private final SendableChooser<String> start_position_chooser = new SendableChooser<>();

  private double shot_power;
  private final SendableChooser<Double> shot_power_chooser = new SendableChooser<>();

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

  // Create drivetrain
  PrecisionDrive auto_drive;
  int auto_stage;
  double start_time;

  // Create shooter
  Shooter shooter = new Shooter(m_shooter, m_shooter2);

  // Initialize all sensors 
  PigeonIMU gyro = new PigeonIMU(config.pigeon_ID);
  AnalogInput dist_sensor_1 = new AnalogInput(config.index_dist_sensor_channel);
  Limelight limelight = new Limelight(NetworkTableInstance.getDefault().getTable("limelight"));

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
    m_chooser.setDefaultOption("Two Ball Low Auto", kTwoBallLowAuto);
    m_chooser.setDefaultOption("Two Ball High Auto", kTwoBallHighAuto);
    m_chooser.addOption("Four Ball High Auto", kFourBallHighAuto);
    m_chooser.addOption("Shoot in Place", kShootInPlace);
    m_chooser.addOption("PID Tuner", kPidTuneAuto);
    m_chooser.addOption("Turn Test", kPIDTurnAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // Start position chooser
    start_position_chooser.setDefaultOption("Hanger Side", config.hangerSide);
    start_position_chooser.addOption("Wall Side", config.wallSide);
    SmartDashboard.putData("Start Position", start_position_chooser);

    // Shot power chooser for relevant autonomous
    shot_power_chooser.setDefaultOption("Low Shot", config.low_shot_power);
    shot_power_chooser.addOption("High Shot", config.high_shot_power);
    SmartDashboard.putData("Shot Power", shot_power_chooser);

    // Set parallel motors to follow the leader
    m_drive_left2.follow(m_drive_left);
    m_drive_left3.follow(m_drive_left);
    m_drive_right2.follow(m_drive_right);
    m_drive_right3.follow(m_drive_right);

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

  /** Run code without user input. Specify autonomous routine with SendableChoosers on Shuffleboard*/
  @Override
  public void autonomousInit() {
    m_auto_selected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_auto_selected);

    start_position = start_position_chooser.getSelected();
    shot_power = shot_power_chooser.getSelected();

    auto_drive = new PrecisionDrive(m_drive_left, m_drive_right, gyro);
    auto_stage = 0;

    // Set robot in high gear
    drive_gear_shift.set(false);

    start_time = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("Auto Stage", auto_stage);
    SmartDashboard.putNumber("Auto Time", Timer.getFPGATimestamp() - start_time);

    if(Timer.getFPGATimestamp() - start_time <= 15) {
        if(m_auto_selected.equals(kPidTuneAuto)) {
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
        }
        
        else if(m_auto_selected.equals(kPIDTurnAuto)) {
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
        }
        
        else if(m_auto_selected.equals(kTwoBallLowAuto)) {
          switch(auto_stage) {
            case 0:
              // Actuate and start intake
              intake_actuator.set(true);
              m_intake.set(1);

              if(start_position.equals(config.hangerSide)) {
                auto_drive.setDistance(4);
              }
              else {
                auto_drive.setDistance(3);
              }
              auto_stage++;
              break;

            case 1:
              // Go back and get ball, start to rev shooter
              if(auto_drive.pidStraight()) {
                time_stamp = Timer.getFPGATimestamp();

                if(start_position.equals(config.hangerSide)) {
                  auto_drive.setDistance(-6.9);
                }
                else {
                  auto_drive.setDistance(-5.9);
                }

                shooter.setPowerLow();
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
                shooter.stop();
                auto_stage++;
              }
              break;

            default:
              // Finished
              SmartDashboard.putString("Status", "Done");
              auto_drive.stop();
              m_index.set(0);
              shooter.stop();
              break;
          }
        }

        else if(m_auto_selected.equals(kTwoBallHighAuto) || m_auto_selected.equals(kFourBallHighAuto)) {
          if(m_auto_selected.equals(kTwoBallHighAuto) && auto_stage > 4) {
            auto_stage = -1;
          }

          switch(auto_stage) {
            case 0:
              // Actuate and start intake
              intake_actuator.set(true);
              m_intake.set(1);

              shooter.setPowerLow();

              if(start_position.equals(config.hangerSide)) {
                auto_drive.setDistance(4);
              }
              else {
                auto_drive.setDistance(3);
              }
              auto_stage++;
              break;

            case 1:
              // Initialize if the robot drives forward for a low shot or shoots long
              if(auto_drive.pidStraight()) {
                time_stamp = Timer.getFPGATimestamp();
                auto_drive.stop();
                m_intake.set(0);
                auto_stage++;
              }
              break;

            case 2:
              // Delay
              if(Timer.getFPGATimestamp() - time_stamp > 0.5) {
                time_stamp = Timer.getFPGATimestamp();
                m_index.set(0.5);
                auto_stage++;
              }
              break;

            case 3:
              // Index to shoot
              if(Timer.getFPGATimestamp() - time_stamp > 2) {
                time_stamp = Timer.getFPGATimestamp();
                auto_drive.setAngle(-110);
                drive_gear_shift.set(true);
                intake_actuator.set(false);
                m_index.set(0);
                shooter.stop();
                auto_stage++;
              }
              break;

            case 4:
              // Turn to third ball
              if(auto_drive.pidTurn()) {
                auto_drive.setDistance(12);
                drive_gear_shift.set(false);
                shooter.setPowerLow();
                m_intake.set(1);
                intake_actuator.set(true);
                auto_stage++;
              }
              break;

            case 5:
              // Drive to third ball
              if(auto_drive.pidStraight()) {
                time_stamp = Timer.getFPGATimestamp();
                auto_drive.stop();
                auto_drive.setAngle(50);
                drive_gear_shift.set(true);
                auto_stage++;
              }
              break;

            case 6:
              // Turn to goal
              if(Timer.getFPGATimestamp() - time_stamp > 0.3 && auto_drive.pidTurn()) {
                auto_drive.stop();
                time_stamp = Timer.getFPGATimestamp();
                m_index.set(0.5);
                drive_gear_shift.set(false);
                auto_stage++;
              }
              break;

            case 7:
              // Fire
              if(Timer.getFPGATimestamp() - time_stamp > 0.5) {
                m_index.set(0);
                shooter.stop();
                auto_stage++;
              }
              break;

            case 8:
              // Go to fourth and fifth balls
              auto_stage++;
              break;

            default:
              // Finished
              SmartDashboard.putString("Status", "Done");
              auto_drive.stop();
              m_intake.set(0);
              m_index.set(0);
              shooter.stop();
              break;
            }
        }

        else if(m_auto_selected.equals(kShootInPlace)) {
          switch(auto_stage){
            case 0:
              shooter.setPower(shot_power);

              if(Timer.getFPGATimestamp() - start_time > 3) {
                time_stamp = Timer.getFPGATimestamp();
                auto_stage++;
              }
              break;
            
            case 1:
              m_index.set(1);

              if(Timer.getFPGATimestamp() - time_stamp > 2) {
                shooter.stop();
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
              shooter.stop();
              break;
          }
        }

        else {
          SmartDashboard.putString("!", "No Auto");
        }
      }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    time_stamp = Timer.getFPGATimestamp();
    start_time = Timer.getFPGATimestamp();
    m_index.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    limelight.setTrackerCamera();
    limelight.ledOn(true);

    SmartDashboard.putNumber("Target x", limelight.getTargetX());
    SmartDashboard.putNumber("Target y", limelight.getTargetY());
    SmartDashboard.putBoolean("Lined Up", limelight.isAlignedToShoot());
    SmartDashboard.putNumber("Distance to Hub", limelight.getDistanceToHub());

    double kP = 0.04;
    
    SmartDashboard.putNumber("Time Remaining", 135 - (Timer.getFPGATimestamp() - start_time));
   
    // Run Drive (Choose controller in use and run tank drive with thresolds)
    if(joy_base.getXButton() && !limelight.isAlignedToShoot()) {
      // Auto align to targets
      m_drive_left.set(-limelight.getTargetX() * kP);
      m_drive_right.set(limelight.getTargetX() * kP);
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
      shooter.setPowerLow();
      shooter_on = true;
      shooter_high = false;
    }
    else if(joy_co.getRightTriggerAxis() > 0.8) {
      // High Power
      shooter.setPowerHigh();
      shooter_on = true;
      shooter_high = true;
    }
    else if(joy_co.getAButton()) {
      shooter.setPowerAuto(limelight.getDistanceToHub());
      shooter_on = true;
      shooter_high = true;
    }
    else {
      shooter.stop();
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
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    SmartDashboard.putNumber("Distance Sensor 1 Value", dist_sensor_1.getValue());
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
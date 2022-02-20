// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
Intake Controls: [UPDATE COMPLETE]
Base Controller (PS4):
 - Drive: Joysticks, tank
 - Drive Shift: TOGGLE L1. This should start in high gear
 - Intake Actuation: HOLD L2 to extend
 - Index: HOLD R1 for index up
Co Controller (Logitech):
 - Intake - TOGGLE LB for intake/off, HOLD BACK for outtake
 - Shooter - HOLD LT for low, HOLD RT for high
 - Index - RB to index to point, HOLD D-DOWN for index down

 - Mode button issues?
*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// General Resources
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.AnalogInput;

// Solenoids
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;

// Data Display Tools
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.swing.plaf.basic.BasicComboPopup.InvocationKeyHandler;

// Rev Resources
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// Cross the Road Resources
import com.ctre.phoenix.sensors.PigeonIMU;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.ControlMode;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // ROBOT CONSTANTS
  // Constant CAN IDs
  // **Make sure to match these when downloading the firmware and other stuff for the neos**
  // Reserved IDs: (RoboRio, 0), (PDB, 1)
  private static final int drive_left1_ID = 2;
  private static final int drive_left2_ID = 3;
  private static final int drive_left3_ID = 4; 
  private static final int drive_right1_ID = 5;
  private static final int drive_right2_ID = 6;
  private static final int drive_right3_ID = 7;
  private static final int shooter1_ID = 8;
  private static final int shooter2_ID = 9;
  private static final int index_ID = 10;
  private static final int intake_ID = 11;
  private static final int pcm_ID = 18;
 
  // Solenoid channels
  private static final int drive_channel = 0;
  private static final int intake_channel = 1;

  // Analog channels
  private static final int dist1_channel = 0;
  

  // Current limit
  private static final int drive_current_limit = 50;
  private static final int intake_current_limit = 25;
  private static final int shooter_current_limit = 50;
  private static final int index_current_limit = 50;
 
  // Constant Robot Stats (IN FEET)
  private static final double rev_distance_conversion = 10/42.35;
  private static final int dist1_threshold = 1000;
  private static final double low_shot_power = 0.42;
  private static final double high_shot_power = 0.72;
 
  // Robot Mechanism Status Variables
   private boolean drive_high_gear = true;
   private double last_drive_shift = 0;
   private boolean intake_out = false;
   private boolean run_intake = false;
   private boolean intake_released = false;
   private boolean index_on = false;
   private boolean shooter_on = false;
   private boolean intake_actuation_primed = true;

   // General time stamp in global scope because of iterative stuff
   private double time_stamp = 0;

  // INITIALIZE ELECTRONICS
  // Controller
  //x-box: private static XboxController joy_base = new XboxController(0);
  private static PS4Controller joy_base = new PS4Controller(0);
  private static XboxController joy_co = new XboxController(1);

  // Add PDB for data reading (optional)
  //PowerDistributionPanel examplePD = new PowerDistribution(0, ModuleType.kAutomatic);

  // Drive motors
  CANSparkMax m_drive_left = new CANSparkMax(drive_left1_ID, MotorType.kBrushless);
  CANSparkMax m_drive_left2 = new CANSparkMax(drive_left2_ID, MotorType.kBrushless);
  CANSparkMax m_drive_left3 = new CANSparkMax(drive_left3_ID, MotorType.kBrushless);
  CANSparkMax m_drive_right = new CANSparkMax(drive_right1_ID, MotorType.kBrushless);
  CANSparkMax m_drive_right2 = new CANSparkMax(drive_right2_ID, MotorType.kBrushless);
  CANSparkMax m_drive_right3 = new CANSparkMax(drive_right3_ID, MotorType.kBrushless);

  // Shooter motors
  CANSparkMax m_shooter = new CANSparkMax(shooter1_ID, MotorType.kBrushless);
  CANSparkMax m_shooter2 = new CANSparkMax(shooter2_ID, MotorType.kBrushless);

  // Index motor
  CANSparkMax m_index = new CANSparkMax(index_ID, MotorType.kBrushless);

  // Intake motors
  CANSparkMax m_intake = new CANSparkMax(intake_ID, MotorType.kBrushless);

  // Gyro
  // PigeonIMU gyro = new PigeonIMU(pigeon_ID);

  // Infrared distance sensor
  AnalogInput dist_sensor_1 = new AnalogInput(dist1_channel);
  
  // Initialize drive train
  DifferentialDrive drivebase = new DifferentialDrive(m_drive_left, m_drive_right);

  // Solenoids
  Compressor compressor = new Compressor(pcm_ID, PneumaticsModuleType.CTREPCM);
  Solenoid drive_gear_shift = new Solenoid(pcm_ID, PneumaticsModuleType.CTREPCM, drive_channel);
  Solenoid intake_actuator = new Solenoid(pcm_ID, PneumaticsModuleType.CTREPCM, intake_channel);


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // Set secondary left motors to follow the leader
    m_drive_left2.follow(m_drive_left);
    m_drive_left3.follow(m_drive_left);

    // Set secondary right motors to follow the leader
    m_drive_right2.follow(m_drive_right);
    m_drive_right3.follow(m_drive_right);

    // Set secondary shooter motor to follow the leader
    m_shooter2.follow(m_shooter);

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

    // Invert right leader
    m_drive_right.setInverted(true);

    // Set current limits

    // Drive current limits
    m_drive_left.setSmartCurrentLimit(drive_current_limit);
    m_drive_left2.setSmartCurrentLimit(drive_current_limit);
    m_drive_left3.setSmartCurrentLimit(drive_current_limit);
    m_drive_right.setSmartCurrentLimit(drive_current_limit);
    m_drive_right2.setSmartCurrentLimit(drive_current_limit);
    m_drive_right3.setSmartCurrentLimit(drive_current_limit);
    
    // Shooter current limit
    m_shooter.setSmartCurrentLimit(shooter_current_limit);
    m_shooter2.setSmartCurrentLimit(shooter_current_limit);
   
    // Intake current limit
    m_intake.setSmartCurrentLimit(intake_current_limit);
   
    // Index current limit
    m_index.setSmartCurrentLimit(index_current_limit);

    m_index.setIdleMode(CANSparkMax.IdleMode.kBrake);

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

  /**
   * Strategies
   * Time Based (Dead Reckining) - susceptible to friction, momentum, etc.
   * Simple Encoder (Bang Bang Control) - susceptible to end momentum
   * PID - high speed and accuracy
   * 
   * General PID Notes
   * Set point is where the system is going towards
   */

  
    /**
  double initial_encoder;
  // Target
  double set_point = 10;
  // Constant PID drive values
  final double kP = 0.06;
  final double kI = 0; // 0.1?
  final double kD = 0;
  double integral = 0;
  double derivative = 0;
  // !!! Temporary measure
  double last_error = set_point;
  double last_time = Timer.getFPGATimestamp();
  */

  PrecisionDrive auto_drive;
  int auto_stage;

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    auto_drive = new PrecisionDrive(m_drive_left, m_drive_right, rev_distance_conversion);
    auto_stage = 0;

    // Reset encoder
    // initial_encoder = m_drive_left.getEncoder().getPosition();
    // SmartDashboard.putNumber("Initial Encoder", m_drive_left.getEncoder().getPosition() - initial_encoder);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        switch(auto_stage) {
          case 0:
            // Set distance to drive back and get ball
            auto_drive.setDistanceControl(-3);

            // Actuate intake
            intake_actuator.set(true);

            // Turn intake on
            m_intake.set(1);

            // High gear
            drive_gear_shift.set(false);

            auto_stage++;
            break;

          case 1:
            // Go back and get ball
            boolean done2 = auto_drive.pidControl(m_drive_left.getEncoder().getPosition()*rev_distance_conversion);

            SmartDashboard.putBoolean("Done c2", done2);

            if(done2) {
              auto_drive.setDistanceControl(5.5);
              m_intake.set(0);
              intake_actuator.set(false);
              auto_stage++;
            }
            break;

          case 2:
            // Go back and get ball
            boolean done3 = auto_drive.pidControl(m_drive_left.getEncoder().getPosition()*rev_distance_conversion);

            SmartDashboard.putBoolean("Done c3", done3);

            if(done3) {
              time_stamp = Timer.getFPGATimestamp();
              m_shooter.set(low_shot_power);
              m_drive_left.set(0);
              m_drive_right.set(0);
              auto_stage++;
            }
            break;

          case 3:
            // Turn
            if(Timer.getFPGATimestamp() - time_stamp < 0.2) {
              m_drive_left.set(-0.2);
              m_drive_right.set(0.2);
            }
            else {
              m_drive_left.set(0);
              m_drive_left.set(0);
              auto_stage++;
            }
            break;

          case 4:
            // Drive for time (0.5 seconds)
            double time_elapsed = Timer.getFPGATimestamp() - time_stamp;

            if(time_elapsed < 0.5) {
              double p = 0.6 - 0.3 * time_elapsed;
              m_drive_left.set(p);
              m_drive_right.set(p);
            }
            else if(time_elapsed < 1.25){
              m_drive_left.set(0);
              m_drive_right.set(0);
            }
            else {
              time_stamp = Timer.getFPGATimestamp();
              auto_stage++;
            }
            break;

          case 5:
            if (Timer.getFPGATimestamp() - time_stamp < 2) {
              m_index.set(0.75);
            }
            else {
              m_index.set(0);
              m_shooter.set(0);
              auto_drive.setDistanceControl(-2.6);
              auto_stage++;
            }
            break;

          case 6:
            // Go back to align with balls 3 and 4
            boolean done6 = auto_drive.pidControl(m_drive_left.getEncoder().getPosition()*rev_distance_conversion);

            SmartDashboard.putBoolean("Done c6", done6);

            if(done6) {
              time_stamp = Timer.getFPGATimestamp();
              m_drive_left.set(0);
              m_drive_right.set(0);
              auto_stage++;
            }
            break;

          case 7:
            // Turn
            if(Timer.getFPGATimestamp() - time_stamp < 0.3) {
              m_drive_left.set(-0.3);
              m_drive_right.set(0.3);
            }
            else {
              m_drive_left.set(0);
              m_drive_left.set(0);
              m_intake.set(1);
              intake_actuator.set(true);
              auto_drive.setDistanceControl(-1);
              auto_stage++;
            }
            break;
          case 8:
            // Drive forwards for 10 feet
            boolean done8 = auto_drive.pidControl(m_drive_left.getEncoder().getPosition()*rev_distance_conversion);

            SmartDashboard.putBoolean("Done c8", done8);

            if(done8) {
              m_drive_left.set(0);
              m_drive_right.set(0);
              m_intake.set(0);
              auto_stage++;
            }
            break;

          default:
            SmartDashboard.putString("Status", "Done");
            break;
        }


      /**
        // Remember to reset encoders before starting this or things could get messy
        double error = set_point - (m_drive_left.getEncoder().getPosition()-initial_encoder)*rev_distance_conversion;
        // Find time elapsed
        double dt = Timer.getFPGATimestamp() - last_time;
        // Update integral
        integral += error * dt;
        // Potentially reset integral on passing set point, also consider capping it or reseting it if it gets too big
        if(error * last_error <= 0) {
          integral = 0;
        }
        // Update derivative
        derivative = (error - last_error)/dt;
        // Determine output speed
        double outputSpeed = error * kP + integral * kI + derivative * kD;
        // Set motors to calculated speed
        m_drive_left.set(outputSpeed);
        m_drive_right.set(outputSpeed);
        // Update variables
        last_time = Timer.getFPGATimestamp();
        last_error = error;
        
        // Wait a set time
        Timer.delay(0.005);
        // Print data to shuffleboard (graphing would be great)
        SmartDashboard.putNumber("encoder change, ", (m_drive_left.getEncoder().getPosition() - initial_encoder));
        SmartDashboard.putNumber("position", (m_drive_left.getEncoder().getPosition() - initial_encoder)*rev_distance_conversion);
        SmartDashboard.putNumber("power", outputSpeed);
        SmartDashboard.putNumber("error", error);
        */

        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    run_intake = false;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
   
    // Run drive (tank)
    // no change if controller changes
    drivebase.tankDrive(joy_base.getLeftY(), joy_base.getRightY());
   
   // Drive shift
   // private boolean drive_high_gear = true;
   // private double last_drive_shift = 0;
   // x-box: if (joy_base.getLeftBumper()) { 
   if (joy_base.getL1Button()) {
     // Ensure that button does not instantaneously shift multiple times with 0.5 second buffer
     if (Timer.getFPGATimestamp() - last_drive_shift > 0.5) {
       drive_high_gear = (!drive_high_gear);
       last_drive_shift = Timer.getFPGATimestamp();
     }
   }
   drive_gear_shift.set(drive_high_gear);

    // Intake actuation (toggle down, otherwise up)

    //x-box: if (joy_base.getLeftTriggerAxis() > 0.8) {
    if(joy_base.getL2Button()) {
      if(!intake_out) {
        intake_actuator.set(true);
        intake_out = true;
      }
    }
    else {
      intake_actuator.set(false);
      intake_out = false;
    }
    
   
   // Intake wheels (toggle on, hold to reverse, stop after reverse)
   if(joy_co.getLeftBumper() && intake_released) {
      run_intake = (!run_intake);
      intake_released = false;
   }
   else if(!joy_co.getLeftBumper()){
     intake_released = true;
   }

   // Intake when toggled
   if(run_intake) {
     m_intake.set(1);
   }

   // Outtake when pressed 
   if(joy_co.getBackButton()) {
      m_intake.set(-1);
      run_intake = false;
   }
   else if(!run_intake) {
      m_intake.set(0);
   }
   
    // Shooter
    if(joy_co.getLeftTriggerAxis() > 0.8) {
      // Low Power
      m_shooter.set(low_shot_power);
      shooter_on = true;
    }
    else if(joy_co.getRightTriggerAxis() > 0.8) {
      // High Power
      m_shooter.set(high_shot_power);
      shooter_on = true;
    }
    else {
      m_shooter.set(0);
      shooter_on = false;
    }

    // Index
    //x-box: if(joy_base.getRightBumper()) {
    if(joy_base.getR1Button()) {
      // Base Controls
      if(dist_sensor_1.getValue() < dist1_threshold) {
        m_index.set(0.8);
      }
      else if (shooter_on) {
        m_index.set(0.8);
      }
      else {
        m_index.set(0);
      }
    }
    else if(joy_co.getPOV() == 180) {
     // Co - Reverse
      m_index.set(-1); 
    }
    else if(joy_co.getRightBumper() && dist_sensor_1.getValue() < dist1_threshold && !index_on) {
      index_on = true;
    }
    else if(index_on) {
      if(dist_sensor_1.getValue() >= dist1_threshold) {
        index_on = false;
      }
      else {
        m_index.set(0.8);
      }
    }
    else {
      m_index.set(0);
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
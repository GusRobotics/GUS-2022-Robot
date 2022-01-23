// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// My imports
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

// Neo Library
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// Neo encoder
import com.revrobotics.RelativeEncoder;

// Talon SRX Resources
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;


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

  // Constant CAN IDs
  // **Make sure to match these when downloading the firmware and other stuff for the neos**
  // Reserved IDs: (RoboRio, 0), (PDB, 1), (PCB, 13)

  private static final int drive_left1_ID = 2;
  private static final int drive_left2_ID = 3;
  private static final int drive_left3_ID = 4; 
  private static final int drive_right1_ID = 5;
  private static final int drive_right2_ID = 6;
  private static final int drive_right3_ID = 7;
  private static final int shooter1_ID = 8;
  private static final int shooter2_ID = 9;
  private static final int index_ID = 10;
  //private static final int intake_ID = ?;

  // Constant Robot Stats
  private static final int wheel_radius = 5;
  private static final double wheel_circumference = 2*wheel_radius*Math.PI;

  // Create objects for major subsystems
  private static PS4Controller joy_base = new PS4Controller(0);

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
  // CANSparkMax m_intake = new CANSparkMax(intake_ID, MotorType.kBrushless);
  
  // Initialize drive train
  DifferentialDrive drivebase = new DifferentialDrive(m_drive_left, m_drive_right);


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

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
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
        // Test PID - drive 5 feet
        


        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Run drive
    drivebase.tankDrive(joy_base.getLeftY(), joy_base.getRightY());

    // Intake
    /** 
    if(joy_base.getSquareButton()) {
      m_intake.set(1);
    }
    else if(joy_base.getTriangleButton()) {
      m_intake.set(-1);
    }
    else {
      m_intake.set(0);
    }
    */

    // Shooter
    if(joy_base.getCircleButton()) {
      // .45, 7ft to back bumper
      m_shooter.set(.25);
    }
    else {
      m_shooter.set(0);
    }

    // Index
    if(joy_base.getL1Button()) {
      m_index.set(1);
    }
    else if(joy_base.getR1Button()) {
      m_index.set(-1);
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
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
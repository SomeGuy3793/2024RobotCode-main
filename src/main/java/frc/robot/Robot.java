
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.OIConstants;
//import frc.robot.subsystems.DriveSubsystem;
//import com.revrobotics.CANSparkBase.IdleMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
private static final String version = "Jeanelle's laptop";

  // The following constants are array indices for tuning values
  public static final int kDriveRatio = 0;
  public static final int kRotateRatio = 1;
  public static final int kItems = 2; // set to last one above + 1

  // Following are for buttons that are either
  // pressed (getRawButton returns true) or not (getRawButton returns false).
  public static final int kAButton = 1;
  public static final int kBButton = 2;
  public static final int kXButton = 3;
  public static final int kYButton = 4;
  public static final int kLeftBumper = 5;
  public static final int kRightBumper = 6;
  public static final int kBackButton = 7;
  public static final int kStartButton = 8;
  public static final int kLeftStickPress = 9;
  public static final int kRighttStickPress = 10;

  // following are for triggers/joy sticks
  // getRawAxis returns values from -1.0000 to 1.0000
  public static final int kLeftStickXaxis = 0;
  public static final int kleftStickYaxis = 1;
  public static final int kLeftTrigger = 2; // 0.00 to 1.00
  public static final int kRightTrigger = 3; // 0.00 to 1.00
  public static final int kRightStickXaxis = 4;
  public static final int kRightStickYaxis = 5;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  //private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  public int tCurIndex = 0;

  // the arrays that get indexed by tCurIndex
  public double[] taCurValue = new double[kItems];
  public double[] taDelta = new double[kItems];
  public String[] taLabel = new String[kItems]; // seen only on SmartDashboard

  boolean aEnabled = true; // state data for auto values tuning routines
  boolean bEnabled = true;
  boolean xEnabled = true;
  boolean yEnabled = true;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    int index;
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    SmartDashboard.putString("version", version);

    // initialize arrays used to tune values used in auto

    index = kDriveRatio;
    taCurValue[index] = 0.5;
    taDelta[index] = 0.1;
    taLabel[index] = "translate speed ";

    index = kRotateRatio;
    taCurValue[index] = 0.5;
    taDelta[index] = 0.1;
    taLabel[index] = "rotation speed ";

    SmartDashboard.putString("changing(2,3)", taLabel[tCurIndex]);
    SmartDashboard.putNumber("curValue(4,1)", taCurValue[tCurIndex]);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    //m_autonomousCommand = null;

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();

      //m_robotDrive.setCoast();

    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // m_robotDrive.drive(
    //     -MathUtil.applyDeadband(m_driverController.getLeftY() * taCurValue[kDriveRatio], OIConstants.kDriveDeadband),
    //     -MathUtil.applyDeadband(m_driverController.getLeftX() * taCurValue[kDriveRatio], OIConstants.kDriveDeadband),
    //     -MathUtil.applyDeadband(m_driverController.getRightX() * taCurValue[kRotateRatio], OIConstants.kDriveDeadband),
    //     true, false); // was true for ratelimit
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  // Only tweak test values using A, B, X, Y buttons if we are disabled
  // Y increments value, A decrements value
  // X goes backwards through list of values, B goes forward
  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {

    // we want distinct press and release of the X-box buttons
    if (yEnabled && m_driverController.getRawButton(kYButton)) {
      yEnabled = false;
      tweakValueAtIndex(1.0); // increase
    } else { // debounce the button
      if (!m_driverController.getRawButton(kYButton)) {
        yEnabled = true;
      }
    }
    if (aEnabled && m_driverController.getRawButton(kAButton)) {
      aEnabled = false;
      tweakValueAtIndex(-1.0); // decrease
    } else { // debounce the button
      if (!m_driverController.getRawButton(kAButton)) {
        aEnabled = true;
      }
    }
    // playing with the index
    if (bEnabled && m_driverController.getRawButton(kBButton)) {
      bEnabled = false;
      tweakTheIndex(1); // increase
    } else {
      if (!m_driverController.getRawButton(kBButton)) {
        bEnabled = true;
      }
    }
    if (xEnabled && m_driverController.getRawButton(kXButton)) {
      xEnabled = false;
      tweakTheIndex(-1); // decrease
    } else {
      if (!m_driverController.getRawButton(kXButton)) {
        xEnabled = true;
      }
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  // start of auto tuning routines
  /////////////////////////////////////////////////////////////////////////////
  // delta is either 1 or -1
  public void tweakTheIndex(int delta) {
    // calculate new index, look for wrap
    tCurIndex += delta;
    if (tCurIndex >= kItems)
      tCurIndex = 0;
    if (tCurIndex < 0)
      tCurIndex = kItems - 1;
    SmartDashboard.putString("changing(2,3)", taLabel[tCurIndex]);
    SmartDashboard.putNumber("curValue(4,1)", taCurValue[tCurIndex]);
  }

  // multiplier is either 1 or -1
  public void tweakValueAtIndex(double multiplier) {
    taCurValue[tCurIndex] += taDelta[tCurIndex] * multiplier;
    SmartDashboard.putNumber("curValue(4,1)", taCurValue[tCurIndex]);
  }

}
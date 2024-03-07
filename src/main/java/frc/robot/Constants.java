// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// https://software-metadata.revrobotics.com/REVLib-2024.json
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class desiredEncoderValue {
    public static final double kSpeakerArmAngle=0;
    public static final double kAmpArmAngle=0;
    public static final double kIntakeArmAngle=0;

    public static final double kClimberPositionTop=0;
    public static final double kClimberPositionBottom=0;
  }

  public static final class ArmConstants{
    // Read about trapezoidal profile constants 
    // public static final double kSportGearRatio = 20.0;
    // public static final double kSportPinionPitchInches = 1.125;
    // public static final double kChainCenterDistanceInches = 6; 
    public static final double kArmGearRatio = 211.909; 

    // SysID values (in radians and radians/sec)
    public static final double kSVolts = 0.11356;
    public static final double kGVolts = 0.29175;
    public static final double kVVoltSecondPerRad = 1.5928;
    public static final double kAVoltSecondSquaredPerRad = 0.030171;
    public static final double kMaxVelocityRadPerSecond = 5;
    public static final double kMaxAccelerationRadPerSecSquared = 8;
    public static final double kArmOffsetRads = Units.degreesToRadians(0); //Starting angle 
    public static final double kArmMaxRads = Units.degreesToRadians(0); //Ending angle

    public static final double kArmEncoderPositionFactor = ((2 * Math.PI) / kArmGearRatio); // radians
    public static final double kArmEncoderVelocityFactor = ((2 * Math.PI) / kArmGearRatio) / 60.0; // radians per second

    public static final double kArmEncoderPositionPIDMinInput = kArmOffsetRads; // radians
    public static final double kArmEncoderPositionPIDMaxInput = (1.5 * Math.PI); // Guess 
    public static final int kArmMotorCurrentLimit = 40; // amps
    // CHANGE 
    public static final double kP = 0.78697; //10000x
    public static final double kPVel = 5.534E-11;
    public static final double kI = 0;
    public static final double kD = 0.00002194;

    public static final double kDVel = 0;
    public static final double kFF = 0;
    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;
    public static final double kMaxPositionErrorRad = 0.7854;
    public static final double kMaxVelocityErrorRadPerSec = 1.8656;
    public static final double kControlEffortVolts = 7;

    public static final double kMaxArmSpeedRpm = NeoMotorConstants.kFreeSpeedRpm / kArmGearRatio ;
    public static final double kMaxArmRadiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(kMaxArmSpeedRpm);

    //public static final double kMaxArmSpeed = 0.2; 
    public static final double kArmSlewRate = 2;
    public static final double kArmDeadband = 0.1;
  }

  public static final class operatorStuff{

    public static final int kTop_ID=41;
    public static final int kBot_ID=42;

    public static final int kIntake_ID=43;

    public static final int kClimberLeft_ID=46;
    public static final int kClimberRight_ID=47;

    public static final int kArmLeft_ID=44;
    public static final int kArmRight_ID=45;


    public static final double kKeepNoteInSpeed=0.20;

    public static final double kArmSpeed=.30;
    public static final double kClimberSpeed=.20;
    public static final double kIdleSpeed=0;
  }

  public static final class IntakeShooter{
    public static final double kShootingSpeed=.75;
    public static final double kIntakeToShootingSpeed=.9;
    public static final double kIntakeSpeed=.45;
  }

  public static final class Swerve {
    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(27); // 1/29/23
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(27);
    public static final Translation2d flModule = new Translation2d(kWheelBase / 2, kTrackWidth / 2);
    public static final Translation2d frModule = new Translation2d(kWheelBase / 2, -kTrackWidth / 2);
    public static final Translation2d blModule = new Translation2d(-kWheelBase / 2, kTrackWidth / 2);
    public static final Translation2d brModule = new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);
    public static final double kMaxSpeedMetersPerSecond = 4.8;  //was 4.8 tested with 0.8
    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(1.0,0,0),
      new PIDConstants(1.0,0,0),
      kMaxSpeedMetersPerSecond,
      flModule.getNorm(),
      new ReplanningConfig()
    );
//module
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 2;  //was 4.8 tested with 0.8
    public static final double kMaxAngularSpeed = Math.PI; // radians per second  was 2 Pi tested .25 PI

    public static final double kDirectionSlewRate = 1.2; // radians per second was 1.2 tested 0.2
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%) was 1.8 tested .3
    public static final double kRotationalSlewRate = 1.0; // percent per second (1 = 100%) was 2.0 tested .25

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(27); // 1/29/23
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(27);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 35; // 1/29/23
    public static final int kRearLeftDrivingCanId = 34;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 36;

    public static final int kFrontLeftTurningCanId = 31;
    public static final int kRearLeftTurningCanId = 32;
    public static final int kFrontRightTurningCanId = 37;
    public static final int kRearRightTurningCanId = 33;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762; // 3 inch wheels
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.4;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    
    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kCoast;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kCoast;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 4;
    public static final double kNoteShotDeadband = .2;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3.0;  //was 3.0 tested 0.5
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0; //was 3.0, .25 tested
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI; // was just pi tested PI / 8
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI; //was just pi tested PI/64

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
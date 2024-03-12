// TrapezoidalPrfile subsystem modified based on Team 1108 Template
//https://github.com/frc1108/Robot2023/blob/12fe3c9d7fb5bbf415d46095e169d892660d4aa7/src/main/java/frc/robot/subsystems/ArmSubsystem.java#L111C1-L120C2

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.operatorStuff;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class ArmSubsystem extends TrapezoidProfileSubsystem{
    private CANSparkMax m_leftArm = new CANSparkMax(operatorStuff.kArmLeft_ID, MotorType.kBrushless);
    private CANSparkMax m_rightArm = new CANSparkMax(operatorStuff.kArmRight_ID, MotorType.kBrushless);
  

    private final AbsoluteEncoder m_rightArmEncoder;
    private final SparkPIDController m_pid;


    private final ArmFeedforward m_feedforward = new ArmFeedforward(ArmConstants.kSVolts, ArmConstants.kGVolts,ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);
    private double m_goal = ArmConstants.kArmOffsetRads;
    // used to limit acceleration so arm doesn't die
    private final SlewRateLimiter m_armSlew = new SlewRateLimiter(ArmConstants.kArmSlewRate); // add arm slew rate to constants file


    public ArmSubsystem() {
    super(
        new TrapezoidProfile.Constraints(
            ArmConstants.kMaxVelocityRadPerSecond,
            ArmConstants.kMaxAccelerationRadPerSecSquared),
            ArmConstants.kArmOffsetRads);

      m_leftArm.restoreFactoryDefaults();
      m_rightArm.restoreFactoryDefaults();

      // Setup the encoder and pid controller
      m_rightArmEncoder = m_rightArm.getAbsoluteEncoder();

      m_rightArmEncoder.setPositionConversionFactor(ArmConstants.kArmEncoderPositionFactor);
      m_rightArmEncoder.setZeroOffset(ArmConstants.kArmOffsetRads);
      m_rightArmEncoder.setVelocityConversionFactor(ArmConstants.kArmEncoderVelocityFactor);


      // just need to read the left arm for pid, but theoretically they should be reading the same values
      m_pid = m_rightArm.getPIDController();
      m_pid.setFeedbackDevice(m_rightArmEncoder);


      m_pid.setP(ArmConstants.kP);
      m_pid.setI(ArmConstants.kI);
      m_pid.setD(ArmConstants.kD);
      m_pid.setFF(ArmConstants.kFF);
      m_pid.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);


      m_leftArm.setIdleMode(IdleMode.kBrake);
      m_rightArm.setIdleMode(IdleMode.kBrake);


      m_leftArm.burnFlash();
      m_rightArm.burnFlash();
    }

    public void stop(){
      m_leftArm.set(0);
      m_rightArm.set(0);
    }
   
    public void periodic(){
      super.setGoal(m_goal);
      super.periodic();

      SmartDashboard.getNumber("current arm position", m_rightArmEncoder.getPosition());
    }


    // Arm PID controls
      public void useState(TrapezoidProfile.State setpoint) {
      // Calculate the feedforward from the sepoint
      double feedforward = m_feedforward.calculate(setpoint.position,setpoint.velocity);
     
      // Add the feedforward to the PID output to get the motor output
      m_pid.setReference(setpoint.position, // - ArmConstants.kArmOffsetRads,
                         ControlType.kPosition, 0, feedforward);
    }
   
    public Command setArmGoalCommand(double goal) {
    return Commands.runOnce(() -> setArmGoal(goal), this);
    }


    public void set(double speed) {
      m_leftArm.set(m_armSlew.calculate(speed));
    }


    public double getPositionRadians() {
      return m_rightArmEncoder.getPosition(); // + ArmConstants.kArmOffsetRads;
    }


    public double getArmGoal() {
      return m_goal;
    }


    public void setArmGoal(double goal) {
      m_goal = MathUtil.clamp(goal,ArmConstants.kArmOffsetRads-0.1,ArmConstants.kArmMaxRads+0.1);
    }


    public void setEncoderPosition(double position) {
      m_rightArmEncoder.setZeroOffset(position);
    }
  }
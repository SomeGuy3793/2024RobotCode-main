// TrapezoidalPrfile subsystem modified based on Team 1108 Template
//https://github.com/frc1108/Robot2023/blob/12fe3c9d7fb5bbf415d46095e169d892660d4aa7/src/main/java/frc/robot/subsystems/ArmSubsystem.java#L111C1-L120C2


// believe it is possible to convert to separate command file
package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.operatorStuff;


// https://software-metadata.revrobotics.com/REVLib-2024.json
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import com.revrobotics.SparkAbsoluteEncoder.Type;


import java.util.function.DoubleSupplier;


import com.revrobotics.CANSparkBase.ControlType;
//import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;


//import edu.wpi.first.wpilibj.ADXRS450_Gyro; // small FRC gyro in SPI slot
// https://dev.studica.com/releases/2024/NavX.json


//import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;




public class ArmSubsystemPID extends TrapezoidProfileSubsystem{
    private CANSparkMax m_leftArm = new CANSparkMax(operatorStuff.kArmLeft_ID, MotorType.kBrushless);
    private CANSparkMax m_rightArm = new CANSparkMax(operatorStuff.kArmRight_ID, MotorType.kBrushless);
   
    private final RelativeEncoder m_leftArmEncoder;
    private final RelativeEncoder m_rightArmEncoder;
    private final SparkPIDController m_pid;


    private final ArmFeedforward m_feedforward = new ArmFeedforward(ArmConstants.kSVolts, ArmConstants.kGVolts,ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);
    private double m_goal = ArmConstants.kArmOffsetRads;
    // used to limit acceleration so arm doesn't die
    private final SlewRateLimiter m_armSlew = new SlewRateLimiter(ArmConstants.kArmSlewRate); // add arm slew rate to constants file


    public ArmSubsystemPID() {
    super(
        new TrapezoidProfile.Constraints(
            ArmConstants.kMaxVelocityRadPerSecond,
            ArmConstants.kMaxAccelerationRadPerSecSquared),
            ArmConstants.kArmOffsetRads);


      m_leftArm.restoreFactoryDefaults();
      m_rightArm.restoreFactoryDefaults();


      // Setup the encoder and pid controller
      m_leftArmEncoder = m_leftArm.getEncoder();
      m_rightArmEncoder = m_rightArm.getEncoder();
      m_leftArmEncoder.setPosition(ArmConstants.kArmOffsetRads);
      m_rightArmEncoder.setPosition(ArmConstants.kArmOffsetRads);
      m_leftArmEncoder.setPositionConversionFactor(ArmConstants.kArmEncoderPositionFactor);
      m_rightArmEncoder.setPositionConversionFactor(ArmConstants.kArmEncoderPositionFactor);
      m_leftArmEncoder.setVelocityConversionFactor(ArmConstants.kArmEncoderVelocityFactor);
      m_rightArmEncoder.setVelocityConversionFactor(ArmConstants.kArmEncoderVelocityFactor);


      // just need to read the left arm for pid, but theoretically they should be reading the same values
      m_pid = m_leftArm.getPIDController();
      m_pid.setFeedbackDevice(m_leftArmEncoder);


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


      SmartDashboard.getNumber("left arm encoder", m_leftArmEncoder.getPosition());
      SmartDashboard.getNumber("right arm encoder", m_rightArmEncoder.getPosition());
      SmartDashboard.getNumber("current arm position", m_leftArmEncoder.getPosition());
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


// public CommandBase manualArmOrHold(double speed) {
//   return Commands.either(setArmGoalCommand(getPositionRadians()).withName("auto arm"),
//                  Commands.run(()->set(speed)).withName("manual arm"),
//                  ()->(Math.abs(speed) < ArmConstants.kArmDeadband));
// }


public double getPositionRadians() {
  return m_leftArmEncoder.getPosition(); // + ArmConstants.kArmOffsetRads;
}


public Command setArmManual(DoubleSupplier speed) {
  return Commands.run(()->setArmGoal(getArmGoal()+speed.getAsDouble()/(2*Math.PI)),this);
}


public double getArmGoal() {
  return m_goal;
}


public void setArmGoal(double goal) {
  m_goal = MathUtil.clamp(goal,ArmConstants.kArmOffsetRads-0.1,ArmConstants.kArmMaxRads+0.1);
}


public void resetDownPosition() {
  m_leftArmEncoder.setPosition(ArmConstants.kArmOffsetRads);
  m_goal = ArmConstants.kArmOffsetRads;
}


public void resetUpPosition() {
  m_leftArmEncoder.setPosition(ArmConstants.kArmMaxRads);
  m_goal = ArmConstants.kArmMaxRads;
}


public void setEncoderPosition(double position) {
  m_leftArmEncoder.setPosition(position);
}


}

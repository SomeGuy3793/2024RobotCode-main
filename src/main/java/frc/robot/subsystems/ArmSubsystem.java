package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.operatorStuff;

// https://software-metadata.revrobotics.com/REVLib-2024.json
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;

public class ArmSubsystem extends SubsystemBase{
    private final CANSparkMax m_leftArm = new CANSparkMax(operatorStuff.kArmLeft_ID, MotorType.kBrushless);
    private final CANSparkMax m_rightArm = new CANSparkMax(operatorStuff.kArmRight_ID, MotorType.kBrushless);
    private final RelativeEncoder armAngle = m_leftArm.getEncoder();
    public ArmSubsystem(){
        m_leftArm.setIdleMode(IdleMode.kBrake);
        m_rightArm.setIdleMode(IdleMode.kBrake);
    }

    public void periodic(){
        
    }
}
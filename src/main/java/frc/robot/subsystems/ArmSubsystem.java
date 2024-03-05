package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.operatorStuff;
// https://software-metadata.revrobotics.com/REVLib-2024.json
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.controller.PIDController;


//import edu.wpi.first.wpilibj.ADXRS450_Gyro; // small FRC gyro in SPI slot
// https://dev.studica.com/releases/2024/NavX.json

import edu.wpi.first.wpilibj.SPI;

//dunno
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem {

    public static CANSparkMax m_leftArm;
    public static CANSparkMax m_rightArm; 
    public RelativeEncoder leftArmEncoder; 
    public RelativeEncoder rightArmEncoder; 

    private static ArmSubsystem m_Instance = null; 

    public static final PIDController armPID = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);

    public ArmSubsystem(int leftArmID, int rightArmID){
      m_leftArm = new CANSparkMax(leftArmID, MotorType.kBrushless);
      m_rightArm = new CANSparkMax(rightArmID, MotorType.kBrushless);
      leftArmEncoder = m_leftArm.getEncoder(SparkRelativeEncoder.Type.kHallSensor,42);
      rightArmEncoder = m_rightArm.getEncoder(SparkRelativeEncoder.Type.kHallSensor,42);
      m_leftArm.setIdleMode(IdleMode.kBrake);
      m_rightArm.setIdleMode(IdleMode.kBrake);
    }
    
    
    
    public static ArmSubsystem getInstance() {
        if (m_Instance == null) {
            m_Instance = new ArmSubsystem(operatorStuff.kArmLeft_ID, operatorStuff.kArmRight_ID);
        }
    
        return m_Instance;
    }

    
}

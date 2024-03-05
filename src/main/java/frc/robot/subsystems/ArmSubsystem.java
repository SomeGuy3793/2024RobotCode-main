package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
// https://software-metadata.revrobotics.com/REVLib-2024.json
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkRelativeEncoder;


//import edu.wpi.first.wpilibj.ADXRS450_Gyro; // small FRC gyro in SPI slot
// https://dev.studica.com/releases/2024/NavX.json

import edu.wpi.first.wpilibj.SPI;

//dunno
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax m_leftArm;
    private CANSparkMax m_rightArm;
    AbsoluteEncoder armAbsoluteEncoder= m_leftArm.getAbsoluteEncoder(Type.kDutyCycle);
    private static ArmSubsystem m_ArmInstance = null;
    
    
public ArmSubsystem(int ArmIdLeft, int ArmIDRight){
 
armAbsoluteEncoder.setPositionConversionFactor(Constants.ModuleConstants.kTurningEncoderPositionFactor);

m_leftArm = new CANSparkMax(Constants.operatorStuff.kArmLeft_ID, MotorType.kBrushless);
m_rightArm = new CANSparkMax(Constants.operatorStuff.kArmRight_ID, MotorType.kBrushless);


   m_leftArm.setIdleMode(IdleMode.kBrake);
   m_rightArm.setIdleMode(IdleMode.kBrake);
    

}



//should self explanatory
public Command armSpeakerAngle(){
  return this.startEnd(
    // When the command is initialized, set the wheels to the intake speed values
    () -> {
      /*while (leftArmAbsoluteEncoder.getPosition()>Constants.desiredEncoderValue.speakerArmAngle){
      setArm(-(Constants.operatorStuff.kArmSpeed));
      }*/
      while (armAbsoluteEncoder.getPosition()<Constants.desiredEncoderValue.kSpeakerArmAngle){
        setArm(Constants.operatorStuff.kArmSpeed);
        }
    },
    // When the command stops, stop the wheels
    () -> {
      stop();
    });
}

public Command armIntakeAngle(){
  return this.startEnd(
    // When the command is initialized, set the wheels to the intake speed values
    () -> {
      while (armAbsoluteEncoder.getPosition()>Constants.desiredEncoderValue.kIntakeArmAngle){
      setArm(-(Constants.operatorStuff.kArmSpeed));
      }
      /*while (leftArmAbsoluteEncoder.getPosition()<Constants.desiredEncoderValue.intakeArmAngle){
        setArm(Constants.operatorStuff.kArmSpeed);
        }*/
    },
    // When the command stops, stop the wheels
    () -> {
      stop();
    });
}

public Command armAmpAngle(){
  return this.startEnd(
    // When the command is initialized, set the wheels to the intake speed values
    () -> {
      /*while (leftArmAbsoluteEncoder.getPosition()>Constants.desiredEncoderValue.ampArmAngle){
        setArm(-(Constants.operatorStuff.kArmSpeed));
        }*/
        while (armAbsoluteEncoder.getPosition()<Constants.desiredEncoderValue.kAmpArmAngle){
          setArm(Constants.operatorStuff.kArmSpeed);
          }
    },
    // When the command stops, stop the wheels
    () -> {
      stop();
    });
}

public Command armClockWise(){
  return this.startEnd(
    // When the command is initialized, set the wheels to the intake speed values
    () -> {
      setArm(Constants.operatorStuff.kArmSpeed);
    },
    // When the command stops, stop the wheels
    () -> {
      stop();
    });
}


public Command armCounterClockWise(){
  return this.startEnd(
    // When the command is initialized, set the wheels to the intake speed values
    () -> {
      setArm(-(Constants.operatorStuff.kArmSpeed));
    },
    // When the command stops, stop the wheels
    () -> {
      stop();
    });
}
    //a
    public void setArm(double speed){
      m_leftArm.set(-speed);
      m_rightArm.set(speed);
    }
    public void stop(){
      m_leftArm.set(0);
      m_rightArm.set(0);
    }
    
    public void periodic(){
      SmartDashboard.getNumber("arm encoder",armAbsoluteEncoder.getPosition());
    };

    public static ArmSubsystem getInstance() {
      if (m_ArmInstance == null) {
          m_ArmInstance = new ArmSubsystem(Constants.operatorStuff.kArmLeft_ID, Constants.operatorStuff.kArmRight_ID);
      }

      return m_ArmInstance;
  }
    
}
package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
// https://software-metadata.revrobotics.com/REVLib-2024.json
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkAbsoluteEncoder;
// import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// create not being used anywhere in program --> just a backup, probably doesn't work and will break robot :) 
public class ArmSubsystemTeleop extends SubsystemBase{

    private CANSparkMax m_leftArm = new CANSparkMax(Constants.operatorStuff.kArmLeft_ID, MotorType.kBrushless);
    private CANSparkMax m_rightArm = new CANSparkMax(Constants.operatorStuff.kArmRight_ID, MotorType.kBrushless);;
    AbsoluteEncoder armAbsoluteEncoder= m_leftArm.getAbsoluteEncoder(Type.kDutyCycle);
   
    public ArmSubsystemTeleop(){
 
    armAbsoluteEncoder.setPositionConversionFactor(Constants.ModuleConstants.kTurningEncoderPositionFactor);

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
    
}
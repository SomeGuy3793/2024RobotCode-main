package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.operatorStuff;
import frc.robot.Constants.IntakeShooter;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Ultrasonic;
// import com.revrobotics.SparkAbsoluteEncoder.Type;
// import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;


public class IntakeSubsystem extends SubsystemBase{

    public static CANSparkMax m_intake = new CANSparkMax(operatorStuff.kIntake_ID, MotorType.kBrushless);
    // may have to change ports later on electrical board, prob
    public static final Ultrasonic m_sensorMid = new Ultrasonic(1, 2);
    public static final Ultrasonic m_sensorIntake = new Ultrasonic(1, 2);

    boolean isMidSensor;
    
    boolean isIntakeSensor;
    double sensorIntakeValue; 
    
    public IntakeSubsystem(){
        m_intake.setIdleMode(IdleMode.kBrake);
    }

    
    public void setIntakeSpeed(double speed){
        m_intake.set(speed);
        SmartDashboard.putNumber("Intake speed", speed);
       }

       public void stopIntake(){
        m_intake.set(0);
        SmartDashboard.putNumber("Intake has been stopped", 0);
       }

// smart dashboard
       public static double getMidSensor(){
        return m_sensorMid.getRangeInches();
       }

       public static double getIntakeSensor(){
        return m_sensorIntake.getRangeInches();
       }
//ends here

       public void detectSensor(){
        double sensValueMid = m_sensorMid.getRangeInches();
        double sensValueIntake = m_sensorIntake.getRangeInches();
       

    if(sensValueMid < 0.5){
            m_intake.set(0);
            isMidSensor = true;
        }
        else 
            m_intake.set(IntakeShooter.kIntakeSpeed);
            isMidSensor = false; 
       

    if(sensValueIntake < 0.5){
        m_intake.set(0);
        isIntakeSensor = true;
      }
       else 
        m_intake.set(IntakeShooter.kIntakeSpeed);
        isIntakeSensor = false; 
   }

       public void periodic(){
        detectSensor();
        SmartDashboard.putNumber("Mid Sensor reading", getMidSensor());
        SmartDashboard.putBoolean("Mid Note status", isMidSensor);
        SmartDashboard.putNumber("Intake Sensor reading", getIntakeSensor());
        SmartDashboard.putBoolean("Intake Note status", isIntakeSensor);
        }
        }
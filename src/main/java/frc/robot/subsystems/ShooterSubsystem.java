package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.operatorStuff;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.SparkAbsoluteEncoder.Type;
// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

public class ShooterSubsystem extends SubsystemBase{

    private final CANSparkMax m_topShooter = new CANSparkMax(operatorStuff.kTop_ID, MotorType.kBrushless);; 
    private final CANSparkMax m_bottomShooter = new CANSparkMax(operatorStuff.kBot_ID, MotorType.kBrushless);; 

    public ShooterSubsystem(){
        m_topShooter.setIdleMode(IdleMode.kBrake);
        m_bottomShooter.setIdleMode(IdleMode.kBrake);
    }

    public void shootIn(double speed){
        m_topShooter.set(speed);
        m_bottomShooter.set(speed);

        SmartDashboard.putNumber("top shooter speed", speed);
        SmartDashboard.putNumber("bottom shooter speed", speed);
    }

       public void shootOut(double speed){
        m_topShooter.set(-speed);
        m_bottomShooter.set(-speed);

        SmartDashboard.putNumber("top shooter speed", -speed);
        SmartDashboard.putNumber("bottom shooter speed", -speed);
    }

    public void setShooterSpeed(double speed){
        m_topShooter.set(speed);
        m_bottomShooter.set(speed);
    }

    public void stopShoot(){
        m_topShooter.set(0);
        m_bottomShooter.set(0);
        SmartDashboard.putNumber("Motors stopped", 0);
    }
}

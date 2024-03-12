package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterSpeed extends InstantCommand{
    private final ShooterSubsystem m_shooter; 
    private final double m_topSpeed; 
    private final double m_bottomSpeed; 

    public SetShooterSpeed(ShooterSubsystem shooter, double topSpeed, double bottomSpeed){
        m_shooter = shooter; 
        m_topSpeed = topSpeed; 
        m_bottomSpeed = bottomSpeed;  
        addRequirements(m_shooter);
    }

    public void initialize(){
        m_shooter.setBottomShooterSpeed(m_bottomSpeed);
        m_shooter.setTopShooterSpeed(m_topSpeed);
    }
    
}

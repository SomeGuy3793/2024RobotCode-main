package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterSpeed extends InstantCommand{
    private final ShooterSubsystem m_shooter; 
    private final double m_speed; 

    public SetShooterSpeed(ShooterSubsystem shooter, double speed){
        m_shooter = shooter; 
        m_speed = speed; 
        addRequirements(m_shooter);
    }

    public void initialize(){
        m_shooter.setShooterSpeed(m_speed);
    }
    
}

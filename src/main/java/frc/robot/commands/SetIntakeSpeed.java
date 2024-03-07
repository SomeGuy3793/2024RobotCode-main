package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeSpeed extends InstantCommand{
    private final IntakeSubsystem m_intake; 
    private final double m_speed; 

    public SetIntakeSpeed(IntakeSubsystem intake, double speed){
        m_intake = intake; 
        m_speed = speed; 
        addRequirements(m_intake);
    }

    public void initialize(){
        m_intake.setIntakeSpeed(m_speed);
    }

}

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmAngle extends InstantCommand{
    private final ArmSubsystem m_arm; 
    private final double m_goal; 
    
    public SetArmAngle(ArmSubsystem arm, double goal){
      m_arm = arm;
      m_goal = goal;
      addRequirements(m_arm);

    }

    public void initialize(){
        m_arm.setArmGoalCommand(m_goal);
    }
}
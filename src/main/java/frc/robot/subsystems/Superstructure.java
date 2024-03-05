// probably won't be using this 

package frc.robot.subsystems;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.desiredEncoderValue;

public class Superstructure extends SubsystemBase{
    private final ArmSubsystemPID m_arm;
    private final ShooterIntakeSubsystem m_shootingOrIntaking;
  
    /** Creates a new Superstructure. */
    public Superstructure(ArmSubsystemPID arm, ShooterIntakeSubsystem shootingOrIntaking) {
    m_arm = arm;
    m_shootingOrIntaking = shootingOrIntaking;
    }
    //timing may be a lil off but heres an example or something
    
    //something liked this
    public  Command shootingToIntakeDelay() {
         return Commands.runOnce(() -> {
             m_shootingOrIntaking.shooterOut();
        }, m_shootingOrIntaking).andThen(
                Commands.waitSeconds(0.2)
                .andThen(() -> { m_shootingOrIntaking.intaking();
                     }).andThen(Commands.waitSeconds(0.05))
                        .andThen(Commands.runOnce(() -> {
                        m_shootingOrIntaking.stop();
                                        })));
                                    }
                

    public Command notePickupAuto(){
        return Commands.sequence(
            Commands.print("note pick up")
            // write these functions 
            // bring arm down 
            // pick up note 
            // bring arm up 

        );
    }

    public Command speakerScoreAuto(){
        return Commands.sequence(
            Commands.print("speaker shot")
             // bring arm up 
            // run shooter 
            // run intake 
            // wait a period of time 
            // stop intake and shooter / bring arm down 
        );
    }

}

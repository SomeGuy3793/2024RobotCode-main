package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystemTeleop;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.subsystems.ArmSubsystemPID;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

// import edu.wpi.first.wpilibj.Joystick.AxisType;
// import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import  edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.operatorStuff;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.POVButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.Joystick;

public class moveArmPID extends PIDCommand{
  
    //private final ArmSubsystemPID m_armPID = ArmSubsystemPID.getInstance();
    public moveArmPID(double setPoint, ArmSubsystem m_arm) {
      super(
          // The controller that the command will use
          new PIDController(Constants.ArmConstants.kP, Constants.ArmConstants.kI, Constants.ArmConstants.kD),
          // This should return the measurement
          () -> m_arm.leftArmEncoder.getPosition(),
          // This should return the setpoint (can also be a constant)
          () -> setPoint,
          // This uses the output
          output -> {
            ArmSubsystemPID.setArmGoalCommand(output);
          }
      // Use addRequirements() here to declare subsystem dependencies.
      // Configure additional PID options by calling `getController` here.
      );
      getController().setTolerance(1.0);
        
    
   
             } 
            }
            


// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants;
// // https://software-metadata.revrobotics.com/REVLib-2024.json
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.SparkAbsoluteEncoder.Type;
// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkRelativeEncoder;


// //import edu.wpi.first.wpilibj.ADXRS450_Gyro; // small FRC gyro in SPI slot
// // https://dev.studica.com/releases/2024/NavX.json

// import edu.wpi.first.wpilibj.SPI;

// //dunno
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj2.command.Command;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;


// public class ShooterIntake extends SubsystemBase{
    
//     private CANSparkMax top;
//     private CANSparkMax bot;
//     private CANSparkMax intake;

// public ShooterIntake(){
 

//  top= new CANSparkMax(Constants.operatorStuff.kTop_ID, MotorType.kBrushless);
//  bot= new CANSparkMax(Constants.operatorStuff.kBot_ID, MotorType.kBrushless);
//  intake= new CANSparkMax(Constants.operatorStuff.kIntake_ID, MotorType.kBrushless);
//  top.setIdleMode(Constants.coastOrBrake.kMotorBrakeMode);
//  bot.setIdleMode(Constants.coastOrBrake.kMotorBrakeMode);
//  intake.setIdleMode(Constants.coastOrBrake.kMotorBrakeMode);
 

// }

 
//     //bring it in
//     public Command keepIn(){
//       return this.startEnd(
//         // When the command is initialized, set the wheels to the intake speed values
//         () -> {
//           setWheels(-(Constants.operatorStuff.kKeepNoteInSpeed));
//           setIntake(-(Constants.operatorStuff.kKeepNoteInSpeed));
//         },
//         // When the command stops, stop the wheels
//         () -> {
//           stop();
//         });
//     }
//     //shooter
//     public Command speaker(){
//       return this.startEnd(
//         // When the command is initialized, set the wheels to the intake speed values
//         () -> {
//           setWheels(Constants.operatorStuff.kShootingSpeed);
//           setIntake(-(Constants.operatorStuff.kIntakeToShootingSpeed));

//         },
//         // When the command stops, stop the wheels
//         () -> {
//           stop();
//         });
//     }
//     //intake
//     public Command intaking(){
//       return this.startEnd(
//         // When the command is initialized, set the wheels to the intake speed values
//         () -> {
          
//           setIntake(Constants.operatorStuff.kIntakeSpeed);
//         },
//         // When the command stops, stop the wheels
//         () -> {
//           stop();
//         });
//     }
//     //Y
//     public void stop(){
//       top.set(0);
//       bot.set(0);
//       intake.set(0);

//    }
//    public void setWheels(double speed){
//     top.set(speed);
//     bot.set(speed);
//    }
   
//    public void setIntake(double speed){
//     intake.set(speed);
//    }

   
// }
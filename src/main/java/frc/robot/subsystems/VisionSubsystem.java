package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.VisionConstants;
import org.photonvision.PhotonUtils;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.controller.PIDController;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    

    import edu.wpi.first.math.util.Units;
    
    public class VisionSubsystem extends SubsystemBase{

      

      PhotonCamera camera = new PhotonCamera("photonvision");

      private PhotonPipelineResult mCameraResult;

      

      
      
      //from ground
       // Constants such as camera and target height stored. Change per robot and goal!
    final double cameraHeightMeters = Units.inchesToMeters(24);
    final double targetHeightMeters = Units.feetToMeters(5);
    // Angle between horizontal and the camera.
    final double cameraPitchRadians = Units.degreesToRadians(0);
     // How far from the target we want to be
     final double goalRangeMeters = Units.feetToMeters(3);
    
     final double ANGULAR_P = 0.1;
     final double ANGULAR_D = 0.0;
     PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D); 

     private void updateCameraResults() {
      mCameraResult = camera.getLatestResult();
    }
  
    // Checks if camera sees targets, must use to stop null exceptions!
    public Boolean hasTargets() {
      return (mCameraResult.hasTargets());
    }
    private PhotonTrackedTarget getBestTarget() {
      return (mCameraResult.getBestTarget());
    }
    private double getTargetArea() {
      return (getBestTarget().getArea());
    }
  
    private double getTargetPitch() {
      return (getBestTarget().getPitch());
    }
  
    private double getTargetYaw() {
      return (getBestTarget().getYaw());
    }
  
    // Returns the april tag ID number
    public int getTargetID() {
      return (getBestTarget().getFiducialId());
    }

    public double getDistance() {
      double distance =
          (targetHeightMeters
                  - cameraHeightMeters)
              / Math.tan(cameraPitchRadians + getTargetPitch());
      return distance;
    }

    // private AprilTagFieldLayout mAprilTagFieldLayout =
    // AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
// get pose of specific april tag from java file
    public void turnTarget(){
     if (hasTargets()){
      
     }
    }

    //  public void detectTarget(){
    //   var result = camera.getLatestResult();
    //   if (result.hasTargets()) {
    //     // First calculate range
    //     double range =
    //             PhotonUtils.calculateDistanceToTargetMeters(
    //               cameraHeightMeters,
    //               targetHeightMeters,
    //               cameraPitchRadians,
    //                     Units.degreesToRadians(result.getBestTarget().getPitch()));
    //   }
    //   if (result.hasTargets()) {
    //     // Calculate angular turn power
    //     // -1.0 required to ensure positive PID controller effort _increases_ yaw
        
    // } else {
    //     // If we have no targets, stay still.
       
    // }
    //  } 

      //unused?
      public static final double FIELD_WIDTH = 16.5; // m
      public static final double FIELD_HEIGHT = 8.1; // m
    
      public static final double DEBOUNCE_TIME = 0.1;

      public void periodic(){
        if (hasTargets()) {

          SmartDashboard.putString("Target ID", getTargetID() + "");
          SmartDashboard.putString("Target Area", getTargetArea() + "%");
          SmartDashboard.putString("Target Pitch", getTargetPitch() + "");
          SmartDashboard.putString("Target Yaw", getTargetYaw() + "");
          SmartDashboard.putNumber("Camera Height", cameraHeightMeters);
          SmartDashboard.putNumber("Camera Pitch", cameraPitchRadians);
          SmartDashboard.putString("Camera Name", "photonvision");
          
    
        } else {
          SmartDashboard.putString("Target ID", "No ID Found!");
          SmartDashboard.putString("Target Pitch", "-1");
          SmartDashboard.putString("Target Area", "0" + "%");
        }
      }
    }
    


package frc.robot.subsystems.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.camConstants;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;
import org.ejml.equation.Variable;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;

public class PhotonVision {

  // NetworkTable PhotonVision =
  // NetworkTableInstance.getDefault().getTable("photonvision"); // not in use
  // right now, camera server below should work.

  PhotonCamera camera = new PhotonCamera(camConstants.kLimelightNetworkID);

  // private NetworkTableEntry result = PhotonVision.getEntry("PipelineIndex");
  // double photonresult = result.getDouble(0);

  // PhotonPoseEstimator estimatePose = new
  // PhotonPoseEstimator(camConstants.tagPlayground,
  // PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, camConstants.camOnBoard);
  PhotonPoseEstimator estimatePose = new PhotonPoseEstimator(RobotContainer.tagPlayground,
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
      camera,
      camConstants.robotToCamera);

  // Sets the pose that you want to be refered to
  public void setReferencePose(Pose2d pose) {
    estimatePose.setReferencePose(pose);
  }
}

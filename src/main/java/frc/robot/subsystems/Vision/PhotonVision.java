package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.camConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class PhotonVision {

  // NetworkTable PhotonVision = NetworkTableInstance.getDefault().getTable("photonvision"); // not
  // in use right now, camera server below should work.

  PhotonCamera camera = new PhotonCamera(camConstants.kLimelightNetworkID);

  // private NetworkTableEntry result = PhotonVision.getEntry("PipelineIndex");
  // double photonresult = result.getDouble(0);

  // PhotonPoseEstimator estimatePose = new PhotonPoseEstimator(camConstants.tagPlayground,
  // PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, camConstants.camOnBoard);
  PhotonPoseEstimator estimatePose =
      new PhotonPoseEstimator(
          camConstants.tagPlayground,
          null,
          camera,
          camConstants.robotToCamera); // need to figure out why PoseStrategy.pose is not working

  // Sets the pose that you want to be refered to
  public void setReferencePose(Pose2d pose) {
    estimatePose.setReferencePose(pose);
  }
}

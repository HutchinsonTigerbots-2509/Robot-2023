package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.camConstants;
import frc.robot.RobotContainer;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision extends SubsystemBase {
  private static final NetworkTable PhotonTable =
      NetworkTableInstance.getDefault().getTable("photonvision");
  private static final NetworkTable PhotonCamera =
      NetworkTableInstance.getDefault().getTable("Front Web Cam");
  public static final NetworkTableEntry PhotonX = PhotonTable.getEntry("targetPixelsX");
  public static final PhotonCamera FrontWebCam = new PhotonCamera("FrontWebCam");

  PhotonCamera camera = camConstants.FrontWebCam;

  private Optional<EstimatedRobotPose> poseOnField = Optional.empty();
  // private  Optional<EstimatedRobotPose> getPose;

  PhotonPipelineResult result = camera.getLatestResult();
  PhotonTrackedTarget target = this.result.getBestTarget();
  // int getTargetID = target.getFiducialId();
  boolean chasTargets = this.result.hasTargets();

  // private Boolean _hasPose = false;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // SmartDashboard.putString("Field Position of X",
    // RobotContainer.aprilTagField.getTagPose(this.targetID).toString());
    var results = FrontWebCam.getLatestResult();
  }

  /**
   * Constructs a packet with the given data.
   *
   * @param data The packet data.
   * @return
   */
  PhotonPoseEstimator estimatePose =
      new PhotonPoseEstimator(
          RobotContainer.aprilTagField,
          PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
          camera,
          camConstants.robotToCamera);

  // Sets the pose that you want to be referred to
  public void setReferencePose(Pose2d pose) {
    estimatePose.setReferencePose(pose);
  }

  public Optional<EstimatedRobotPose> currentPose() {
    return poseOnField;
  }

  public void PoseEstimating() {

    // SmartDashboard.putNumber("Target Seen", this.targetID); // Replaced with just the fudicial
    // ID.
    SmartDashboard.putBoolean("Has Target", chasTargets);
    // this.getPose = estimatePose.update();

  }

  public double fetchTargetX() {
    NetworkTableEntry mTableX = PhotonTable.getEntry("FrontWebCam/targetPixelsX");
    double mTargetX = mTableX.getDouble(0);
    return mTargetX;
  }

  public NetworkTableEntry getDistance() {
    NetworkTableEntry mPTableTP = PhotonTable.getEntry(camConstants.kPhotonTargetPose);
    Pose3d targetPose = (Pose3d) mPTableTP.getValue().getValue(); // Returns the april tag's pose

    SmartDashboard.putNumber("relative X", targetPose.getX());
    SmartDashboard.putNumber("relative Y", targetPose.getY());

    // Need to figure out distance from pose

    return mPTableTP;
  }
}

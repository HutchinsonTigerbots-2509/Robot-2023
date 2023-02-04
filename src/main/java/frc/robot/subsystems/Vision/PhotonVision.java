package frc.robot.subsystems.Vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.camConstants;
import frc.robot.RobotContainer;

public class PhotonVision extends SubsystemBase{
  NetworkTable PhotonTable = NetworkTableInstance.getDefault().getTable("photonvision");



  PhotonCamera camera = new PhotonCamera(camConstants.kPhotonCameraID);
  private Optional<EstimatedRobotPose> poseOnField = Optional.empty();
  private  Optional<EstimatedRobotPose> getPose;

  PhotonPipelineResult result = camera.getLatestResult();
  PhotonTrackedTarget target = result.getBestTarget();
  int targetID = target.getFiducialId();
  boolean hasTargets = result.hasTargets();


  private Boolean _hasPose = false;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Field Position of X", RobotContainer.aprilTagField.getTagPose(targetID).toString());
  }

/**
 * Constructs a packet with the given data.
 *
 * @param data The packet data.
 * @return 
 */

  // private NetworkTableEntry result = PhotonVision.getEntry("PipelineIndex");
  // double photonresult = result.getDouble(0);

  // PhotonPoseEstimator estimatePose = new
  // PhotonPoseEstimator(camConstants.tagPlayground,
  // PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, camConstants.camOnBoard);
  PhotonPoseEstimator estimatePose = new PhotonPoseEstimator(RobotContainer.aprilTagField,
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


    // tell the robot when it sees an april tag
    // if (_hasPose == true){
    //   poseOnField = this.getPose;
    // } else if (_hasPose == false) {
    //   poseOnField = Optional.empty();
    // } else {
    //   poseOnField = Optional.empty();
    // }
    
    SmartDashboard.putNumber("Target Seen", targetID); // Replaced with just the fudicial ID.
    SmartDashboard.putBoolean("Has Target", hasTargets);
    this.getPose = estimatePose.update();
    
  }


  /** Clears the packet and resets the read and write positions. */


  // See if the robot is looking at an april tag 
  
  // REPLACED WITH hasTargets
  
  // public Boolean HasPose(){
  //   if(this.getPose == null) return false;
  //  if (this.getPose.isPresent()){
  //     _hasPose = true;
  //   } else {
  //     _hasPose = false;
  //   }

  //   return _hasPose;
  // }

  public double fetchTargetX() {
    NetworkTableEntry mPTableX = PhotonTable.getEntry(camConstants.kPhotonTargetXID);
    double mPTargetX = mPTableX.getDouble(0.0);
    return mPTargetX;
  }

  public NetworkTableEntry getDistance() {
    NetworkTableEntry mPTableTP = PhotonTable.getEntry(camConstants.kPhotonTargetPose);
    Pose3d targetPose = (Pose3d)mPTableTP.getValue().getValue(); // Returns the april tag's pose

    SmartDashboard.putNumber("relative X", targetPose.getX());
    SmartDashboard.putNumber("relative Y", targetPose.getY());


    // Need to figure out distance from pose 

    return mPTableTP;
  }
}

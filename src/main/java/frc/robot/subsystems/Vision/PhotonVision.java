package frc.robot.subsystems.Vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.camConstants;
import frc.robot.RobotContainer;

public class PhotonVision {

  int size;
  byte[] packetData;

  // NetworkTable PhotonVision =
  // NetworkTableInstance.getDefault().getTable("photonvision"); // not in use
  // right now, camera server below should work.

  PhotonCamera camera = new PhotonCamera(camConstants.kPhotonCameraID);
  private Optional<EstimatedRobotPose> poseOnField = Optional.empty();
  private  Optional<EstimatedRobotPose> getPose;


  private Boolean _hasPose = false;
  public void Packet(int size) {
    this.size = size;
    packetData = new byte[size];
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
  PhotonPoseEstimator estimatePose = new PhotonPoseEstimator(RobotContainer.tagPlayground,
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
    if (_hasPose == true){
      poseOnField = this.getPose;
    } else if (_hasPose == false) {
      poseOnField = Optional.empty();
    } else {
      poseOnField = Optional.empty();
    }
    
    SmartDashboard.putBoolean("Has Pose", HasPose());
    this.getPose = estimatePose.update();
    
  }


  public void Packet(byte[] data) {
    packetData = data;
    size = packetData.length;
  }
  
  /** Clears the packet and resets the read and write positions. */


  // See if the robot is looking at an april tag
  public Boolean HasPose(){
    if(this.getPose == null) return false;
   if (this.getPose.isPresent()){
      _hasPose = true;
    } else {
      _hasPose = false;
    }

    return _hasPose;
  }

  PhotonPipelineResult camResults = new PhotonPipelineResult();


  public void clearCache() {
    byte[] packetData = new byte[size];
    int readPos = 0;
    int writePos = 0;
  }

  public PhotonPipelineResult getCurrentResults() {
 return null; 
    
  }
}

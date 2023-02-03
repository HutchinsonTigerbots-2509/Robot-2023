package frc.robot.subsystems.Vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.camConstants;
import frc.robot.RobotContainer;

public class PhotonVision extends SubsystemBase{

  int size;
  byte[] packetData;
  NetworkTable PhotonTable = NetworkTableInstance.getDefault().getTable("photonvision");

  private NetworkTableEntry rawBytes = PhotonTable.getEntry("rawBytes");


  PhotonCamera camera = new PhotonCamera(camConstants.kPhotonCameraID);
  private Optional<EstimatedRobotPose> poseOnField = Optional.empty();
  private  Optional<EstimatedRobotPose> getPose;
  


  private Boolean _hasPose = false;

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

  public double fetchTargetX() {
    NetworkTableEntry mPTableX = PhotonTable.getEntry(camConstants.kPhotonTargetXID);
    double mPTargetX = mPTableX.getDouble(1.0);
    return mPTargetX;
  }

    
}

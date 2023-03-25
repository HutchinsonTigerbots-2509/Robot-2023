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

import frc.robot.Constants.camConstants;

public class PhotonVision extends SubsystemBase {
  private static final NetworkTable PhotonCamera = NetworkTableInstance.getDefault().getTable("FrontWebCam");

  public static final NetworkTableEntry PhotonX = PhotonCamera.getEntry("FrontWebCam/targetPixelsX");
  public static final NetworkTableEntry PhotonY = PhotonCamera.getEntry("FrontWebCam/targetPixelsY");

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double fetchTargetX() {
    NetworkTableEntry mTableX = PhotonX; // camConstants.PhotonTable.getEntry("FrontWebCam/targetPixelsX");
    double mTargetX = mTableX.getDouble(0);
    return mTargetX;
  }

  public double fetchTargetY() {
    NetworkTableEntry mTableY = PhotonY;
    double mTargetY = mTableY.getDouble(0);
    return mTargetY;
  }

}

package frc.robot.subsystems;

import java.util.Optional;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

@Logged
public class PhotonCam extends SubsystemBase {
  private PhotonCamera cam;
  private StructPublisher<Pose3d> publisher;

  // The field from AprilTagFields will be different depending on the game.
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  // Construct PhotonPoseEstimator
  private PhotonPoseEstimator photonPoseEstimator;

  // Loggers hate him! Watch Matrix crash Epilogue with this one simple trick!
  @NotLogged
  private Matrix<N3, N1> confidenceMatrix = new Matrix<N3, N1>(new SimpleMatrix(new double[] { 100, 100, 10000 }));

  private Alert cameraOffline;

  public PhotonCam(String camera, Transform3d robotToCam) {
    cam = new PhotonCamera(camera);
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        robotToCam);

    cameraOffline = new Alert("Camera "+camera+" is offline!", AlertType.kError);

    publisher = NetworkTableInstance.getDefault().getStructTopic(camera, Pose3d.struct).publish();

    //SmartDashboard.putNumber("Photon XY Confidence", 1000);
    //SmartDashboard.putNumber("Photon R Confidence", 10000);
    //SmartDashboard.putBoolean("Photon use calculated", true);
  }

  public void periodic() {
    // Alert if camera is offline
    cameraOffline.set(!cam.isConnected());

    // Process results detected since last run
    for (PhotonPipelineResult result : cam.getAllUnreadResults()) {
      Optional<EstimatedRobotPose> o = photonPoseEstimator.update(result);

      if (o.isPresent()) {
        EstimatedRobotPose e = o.get();

        publisher.set(e.estimatedPose);

        // Display on map
        RobotContainer.getRobot().swerve.field.getObject(cam.getName()).setPose(e.estimatedPose.toPose2d());

        // Calculate our current equation, just for dashbard display to compare
        double area = 0;
        double ambiguity = 0;
        double score = 0;
        for (PhotonTrackedTarget x : e.targetsUsed) {
          double amb = x.getPoseAmbiguity();

          area += x.getArea();
          ambiguity += x.getPoseAmbiguity();

          // Don't use overly ambiguous poses or invalid (-1) poses
          if (amb < 0) {
            // Tags estimated as a MultiTag are invalid, but are extremely accurate
            // Hell, double it!
            score += x.getArea() * 2;
          }
          // Docs suggest anything above 0.2 is useless
          // Scale down score down to nothing at or beyond 0.2
          else if (amb < 0.2) {
            score += x.getArea() * (x.getPoseAmbiguity() * -5 + 1);

            // Linear from 1 to 0 for x from 0 to 0.2:
            // -5x+1
          }
        }

        // About 2 seems to be the highest score I can achieve with 3 tags inview
        // 1 mid range tag is almost 0
        // 2 far tags is 0.5
        // 2 close is 1
        // 1 close tag is .45

        // Final confidence values:
        // 10 slides slowly
        // 1 moves pretty quick
        // 0 is instant
        // Negative values are treated as positive

        // Would like to scale from 5 to 0.5 as score increases from 0 to 1.5?
        // x * (-4.5 / 1.5)+5
        double calculatedConf = Math.max(score * (-9.5 / 1.5) + 10, 0.5);
        // double calculatedConf = Math.max(Math.pow(score,2) * (-9.5 / 1.5) + 15, 0.5);

        // Matt's equation from summer 2024
        // double calculatedConf = Math.pow(area,2) * 8.0/3.0 - 0.83333333;

        SmartDashboard.putNumber(cam.getName() + "/total area", area);
        SmartDashboard.putNumber(cam.getName() + "/ambiguity", ambiguity);
        SmartDashboard.putNumber(cam.getName() + "/score", score);

        SmartDashboard.putNumber(cam.getName() + "/calculated conf", calculatedConf);

        double xy;
        //SmartDashboard.getNumber("Photon XY Confidence", 0);
        //double r;
        //SmartDashboard.getNumber("Photon R Confidence", 0);

        //if (SmartDashboard.getBoolean("Photon use calculated", true)) {
          xy = calculatedConf;
          // r = calculatedConf;
        //  r = SmartDashboard.getNumber("Photon R Confidence", 10000);
        //} else {
        //  xy = SmartDashboard.getNumber("Photon XY Confidence", 1000);
        //  r = SmartDashboard.getNumber("Photon R Confidence", 1000);
       // }

        // Fill existing matrix instead of making a new one to save on memory
        // allocations
        confidenceMatrix.set(0, 0, xy);
        confidenceMatrix.set(1, 0, xy);
        // confidenceMatrix.set(2, 0, r);
        if (inField(e)) {
          RobotContainer.getRobot().swerve.swerveOdometry.addVisionMeasurement(
              e.estimatedPose.toPose2d(),
              e.timestampSeconds,
              confidenceMatrix);
        }
      }
    }
  }

  // 8.2106y m 16.5418x m
  public boolean inField(EstimatedRobotPose e) {
    {
      if (e.estimatedPose.getY() > 8.1) {
        return false;
      } else if (e.estimatedPose.getY() < -0.1) {
        return false;
      }
      if (e.estimatedPose.getX() > 17.6) {
        return false;
      } else if (e.estimatedPose.getX() < -0.1) {
        return false;
      }
      if (e.estimatedPose.getZ() > 0.5) {
        return false;
      } else if (e.estimatedPose.getZ() < -0.5) {
        return false;
      }
      return true;
    }
  }
}

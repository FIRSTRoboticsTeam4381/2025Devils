// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Photon extends SubsystemBase {
  /** Creates a new Camera. */
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();
  private PhotonCamera camA;
  private PhotonCamera camB;
  private PhotonCamera camC;
  private PhotonCamera camD;
  private Transform3d robotToCamA;
  private Transform3d robotToCamB;
  private Transform3d robotToCamC;
  private Transform3d robotToCamD;
  private PhotonPoseEstimator poseEstimateA;
  private PhotonPoseEstimator poseEstimateB;
  private PhotonPoseEstimator poseEstimateC;
  private PhotonPoseEstimator poseEstimateD;
  private Pose3d pose;
  private RobotContainer robotContainer;

  private Matrix<N3,N1> camAMatrix = new Matrix<N3,N1>(SimpleMatrix.filled(3,1,100));
  private Matrix<N3,N1> camBMatrix = new Matrix<N3,N1>(SimpleMatrix.filled(3,1,100));
  private Matrix<N3,N1> camCMatrix = new Matrix<N3,N1>(SimpleMatrix.filled(3,1,100));
  private Matrix<N3,N1> camDMatrix = new Matrix<N3,N1>(SimpleMatrix.filled(3,1,100));

  /*
   * Testing Notes:
   * Successfully got it to work with different numbers. 0 trusted it perfectly,
   * 1 trusted it a bit slower, 10000 didn't move and 1000 hardly moved, 100 was v slow
   */

  StructPublisher<Pose3d> publisherA = NetworkTableInstance.getDefault()
    .getStructTopic("Camera_A", Pose3d.struct).publish();
  StructPublisher<Pose3d> publisherB = NetworkTableInstance.getDefault()
    .getStructTopic("Camera_B", Pose3d.struct).publish(); 
  StructPublisher<Pose3d> publisherC = NetworkTableInstance.getDefault()
    .getStructTopic("Camera_C", Pose3d.struct).publish();
  StructPublisher<Pose3d> publisherD = NetworkTableInstance.getDefault()
    .getStructTopic("Camera_D", Pose3d.struct).publish();
    

  public Photon() {
    camA = new PhotonCamera("Camera_A");
    camB = new PhotonCamera("Camera_B");
    camC = new PhotonCamera("Camera_C");
    camD = new PhotonCamera("Camera_D");
    robotToCamA = new Transform3d(new Translation3d(-0.27, 0.26, 0.21), new Rotation3d(0,-45/180.0*Math.PI,225.0/180.0*Math.PI));
    robotToCamB = new Transform3d(new Translation3d(-0.27, -0.26, 0.21), new Rotation3d(0,-45/180.0*Math.PI,135.0/180.0*Math.PI));
    robotToCamC = new Transform3d(new Translation3d(-0.27, 0.26, 0.21), new Rotation3d(0,-45/180.0*Math.PI,225.0/180.0*Math.PI));
    robotToCamD = new Transform3d(new Translation3d(-0.27, -0.26, 0.21), new Rotation3d(0,-45/180.0*Math.PI,135.0/180.0*Math.PI));
    poseEstimateA = new PhotonPoseEstimator(aprilTagFieldLayout, null, robotToCamA);
    poseEstimateB = new PhotonPoseEstimator(aprilTagFieldLayout, null, robotToCamB);
    poseEstimateC = new PhotonPoseEstimator(aprilTagFieldLayout, null, robotToCamC);
    poseEstimateD = new PhotonPoseEstimator(aprilTagFieldLayout, null, robotToCamD);
  }


  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> a = poseEstimateC.update(null);
    Optional<EstimatedRobotPose> b = poseEstimateD.update(null);
    Optional<EstimatedRobotPose> c = poseEstimateC.update(null);
    Optional<EstimatedRobotPose> d = poseEstimateD.update(null);

    List<PhotonTrackedTarget> camATargets = camA.getLatestResult().getTargets();
    double area = 0;
    for(PhotonTrackedTarget t : camATargets){
      area+=t.getArea();
    }
    SmartDashboard.putNumber("Cam A Target Area", area);
    SmartDashboard.putNumber("Cam A Matrix Fill", 2.66667/(area*area) -0.833333);

    SmartDashboard.putBoolean("CamA Target", a.isPresent());
    if(a.isPresent()) {
      camAMatrix.fill(2.66667/(area*area) -0.833333);
      //camCMatrix.fill(0);
      EstimatedRobotPose pose = a.get();
      publisherA.set(pose.estimatedPose);
      robotContainer.swerve.swerveOdometry.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, camAMatrix);
    
    }else{
      pose = new Pose3d();
    }

    List<PhotonTrackedTarget> camBTargets = camB.getLatestResult().getTargets();
    area = 0;
    for(PhotonTrackedTarget t : camBTargets){
      area+=t.getArea();
    }
    SmartDashboard.putNumber("Cam B Target Area", area);
    SmartDashboard.putNumber("Cam B Matrix Fill", 2.66667/(area*area) -0.833333);

    SmartDashboard.putBoolean("CamB Target", b.isPresent());
    if(b.isPresent()) {
      camCMatrix.fill(2.66667/(area*area) -0.833333);
      //camCMatrix.fill(0);
      EstimatedRobotPose pose = b.get();
      publisherB.set(pose.estimatedPose);
      robotContainer.swerve.swerveOdometry.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, camBMatrix);
    
    }else{
      pose = new Pose3d();
    }

    List<PhotonTrackedTarget> camCTargets = camC.getLatestResult().getTargets();
    area = 0;
    for(PhotonTrackedTarget t : camCTargets){
      area+=t.getArea();
    }
    SmartDashboard.putNumber("Cam C Target Area", area);
    SmartDashboard.putNumber("Cam C Matrix Fill", 2.66667/(area*area) -0.833333);

    SmartDashboard.putBoolean("CamC Target", c.isPresent());
    if(c.isPresent()) {
      camCMatrix.fill(2.66667/(area*area) -0.833333);
      //camCMatrix.fill(0);
      EstimatedRobotPose pose = c.get();
      publisherC.set(pose.estimatedPose);
      robotContainer.swerve.swerveOdometry.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, camCMatrix);
    
    }else{
      pose = new Pose3d();
    }

    List<PhotonTrackedTarget> camDTargets = camD.getLatestResult().getTargets();
    area=0;
    for(PhotonTrackedTarget t : camDTargets){
      area+=t.getArea();
    }
    SmartDashboard.putNumber("Cam D Target Area", area);
    SmartDashboard.putNumber("Cam D Matrix Fill", 2.66667/(area*area) -0.833333);

    SmartDashboard.putBoolean("CamD Target", d.isPresent());
    if(d.isPresent()) {
      camDMatrix.fill(2.66667/(area*area) -0.833333);
      //camDMatrix.fill(0);
      EstimatedRobotPose pose = d.get();
      publisherD.set(pose.estimatedPose);
      robotContainer.swerve.swerveOdometry.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, camDMatrix);
    }else{
      pose = new Pose3d();
    }

    /*
     * Testing Notes:
     * Got it to work with a hard cutoff, it's super cool. I think the area scale
     * is between 0-1.5 with this year's field, you don't really get more than 1.5.
     * I think next we're going to linearly adjust trust based on area.
     */

    //camCMatrix.

    
  }

  public boolean GetTargets() {
    var result = camC.getLatestResult();
    boolean hasTargets = result.hasTargets();
    return hasTargets;
  }


  public double GetTargetYaw() {
    var result = camC.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    double yaw = target.getYaw();
    return yaw;
  }

  public double GetTargetPitch() {
    var result = camC.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    double pitch = target.getPitch();
    return pitch;
  }

  public double GetTargetArea() {
    var result = camC.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    double area = target.getArea();
    return area;
  }

  public int GetAprilTagID() {
    var result = camC.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    int targetID = target.getFiducialId();
    return targetID;
  }

  public double GetAprilTagPoseAmbiguity() {
    var result = camC.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    double poseAmbiguity = target.getPoseAmbiguity();
    return poseAmbiguity;
  }

  public void GetAprilTagPose() {
    var result = camC.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    // Oddly doesn't work
    //Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()), cameraToRobot);
  }

  
}
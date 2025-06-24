// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.objectDetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionIO;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;

/** Real Object Detection */
public class ObjectDetectionVisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  protected Supplier<Pose2d> poseSupplier;
  protected Pose3d robotPose;
  protected Pose3d cameraPose;

  public ObjectDetectionVisionIOPhotonVision(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.poseSupplier = poseSupplier;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    for (var result : camera.getAllUnreadResults()) {
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }

      // Get object pose
      robotPose = new Pose3d(poseSupplier.get());
      cameraPose = robotPose.transformBy(robotToCamera);
      if (result.hasTargets()) {
        inputs.objectPose = cameraPose.transformBy(result.getBestTarget().bestCameraToTarget);
      } else {
        inputs.objectPose = null;
      }
    }
  }
}

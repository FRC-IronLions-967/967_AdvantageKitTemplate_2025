// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetection extends SubsystemBase {
  /** Creates a new ObjectDetection. */
  private PhotonCamera objectDetectionCamera;

  private List<PhotonPipelineResult> result;
  private PhotonTrackedTarget bestObject;

  public ObjectDetection() {

    objectDetectionCamera = new PhotonCamera(VisionConstants.objectDetectionCameraName);
  }

  public Rotation2d getObjectAngle() {
    return new Rotation2d(bestObject.getYaw());
  }

  @Override
  public void periodic() {
    result = objectDetectionCamera.getAllUnreadResults();
    bestObject = result.get(0).getBestTarget();
  }
}

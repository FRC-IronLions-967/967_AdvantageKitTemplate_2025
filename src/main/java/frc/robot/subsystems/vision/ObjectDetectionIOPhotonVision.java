// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;

/** Add your docs here. */
public class ObjectDetectionIOPhotonVision implements ObjectDetectionIO {

  protected PhotonCamera camera;
  protected Transform3d robotToCamera;

  public ObjectDetectionIOPhotonVision(String cameraName, Transform3d robotToCamera) {
    camera = new PhotonCamera(cameraName);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    inputs.isConnected = camera.isConnected();
    if (inputs.isConnected) {
      for (var result : camera.getAllUnreadResults()) {
        inputs.hasTarget = result.hasTargets();
        if (inputs.hasTarget) {
          var bestTarget = result.getBestTarget();
          inputs.targetRot =
              new Rotation3d(
                  bestTarget.getYaw(),
                  bestTarget.getPitch(),
                  bestTarget.getSkew()); // skew only used if nonsymetric game piece
          inputs.distanceToTarget =
              bestTarget
                  .getBestCameraToTarget()
                  .plus(robotToCamera)
                  .getTranslation()
                  .toTranslation2d()
                  .getNorm();
        } else {
          inputs.targetRot = new Rotation3d();
          inputs.distanceToTarget = 0.0;
        }
      }
    } else {
      inputs.hasTarget = false;
      inputs.targetRot = new Rotation3d();
      inputs.distanceToTarget = 0.0;
    }
  }
}

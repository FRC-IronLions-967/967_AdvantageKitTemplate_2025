// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.objectDetection;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ObjectDetectionVisionIO {

  @AutoLog
  public static class ObjectDetectionVisionIOInputs {
    boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());
    Pose3d objectPose = new Pose3d();
  }

  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  public default void updateInputs(ObjectDetectionVisionIOInputs inputs) {}
}

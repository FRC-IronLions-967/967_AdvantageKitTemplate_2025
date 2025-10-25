// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface AprilTagIO {
  @AutoLog
  public static class AprilTagIOInputs {
    public boolean isConnected = false;
    public boolean hasTarget = false;
    public TargetInfo[] targetInfo;
    public PoseObservation[] poseObservations = new PoseObservation[0];
  }

  public static record TargetInfo(int tagID, Rotation3d targetRot, double distanceToTarget) {}

  public static record PoseObservation(double ambiguity, Pose3d pose) {}

  public default void updateInputs(AprilTagIOInputs inputs) {}
}

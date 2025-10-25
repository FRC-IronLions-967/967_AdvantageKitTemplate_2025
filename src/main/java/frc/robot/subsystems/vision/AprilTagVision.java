// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.AprilTagIO.PoseObservation;

public class AprilTagVision extends SubsystemBase {

  private final AprilTagIO[] io;
  private final AprilTagIOInputsAutoLogged[] inputs;

  /** Creates a new AprilTagVision. */
  public AprilTagVision(AprilTagIO... io) {
    this.io = io;

    this.inputs = new AprilTagIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new AprilTagIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    // Update Inputs
    for (int i = 0; i < inputs.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs(
          "Vision/AprilTag/Camera" + Integer.toString(i), inputs[i]);
    }
    List<PoseObservation> robotObservatons = new LinkedList<>();
    List<PoseObservation> robotObservatonsAccepted = new LinkedList<>();
    List<PoseObservation> robotObservatonsRejected = new LinkedList<>();
    // Update Pose
    for (int cameraIndex = 0; cameraIndex < inputs.length; cameraIndex++) {
      for (var obs : inputs[cameraIndex].poseObservations) {
        boolean rejectPose = 
                obs.ambiguity() > VisionConstants.maxAmbiguity 
                || obs.pose().getZ() > VisionConstants.maxZError 
                || !obs.hasTags()
                || obs.pose().getX() < 0
                || obs.pose().getY() < 0 
                || obs.pose().getX() > VisionConstants.kTagLayout.getFieldLength()
                || obs.pose().getY() > VisionConstants.kTagLayout.getFieldWidth();

        robotObservatons.add(obs);
        if (rejectPose) {
          robotObservatonsRejected.add(obs);
        } else {
          robotObservatonsAccepted.add(obs);
        }       
      }
    }
  }
  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
      Pose2d pose,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs
    );
  }
}

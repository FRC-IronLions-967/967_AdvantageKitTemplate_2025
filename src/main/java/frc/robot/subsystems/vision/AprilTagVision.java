// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.AprilTagIO.PoseObservation;
import frc.robot.subsystems.vision.AprilTagIO.TargetInfo;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {

  private final AprilTagIO[] io;
  private final AprilTagIOInputsAutoLogged[] inputs;
  private Pose2d acceptedPose;
  private boolean acceptedPoseGood;

  /** Creates a new AprilTagVision. */
  public AprilTagVision(AprilTagIO... io) {
    this.io = io;

    this.inputs = new AprilTagIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new AprilTagIOInputsAutoLogged();
    }
  }

  /**
   * @param cameraIndex what camera to look
   * @param aprilTagID what april tag to look at
   * @return the rot and norm of the specified april tag
   */
  public TargetInfo getTargetInfo(int cameraIndex, int aprilTagID) {
    for (int i = 0; i < inputs[cameraIndex].targetInfo.length; i++) {
      if (inputs[cameraIndex].targetInfo[i].tagID() == aprilTagID) {
        return inputs[cameraIndex].targetInfo[i];
      }
    }
    return null;
  }

  /**
   * @return if the vision pose is good
   */
  public boolean isVisionPoseGood() {
    return acceptedPoseGood;
  }

  /**
   * !!!Use with isVisionPoseGood() !!!
   *
   * @return the pose calculated between all cameras
   */
  public Pose2d getVisionPose() {
    return acceptedPose;
  }

  @Override
  public void periodic() {
    // Update Inputs
    for (int i = 0; i < inputs.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/AprilTag/Camera" + Integer.toString(i), inputs[i]);
    }
    List<PoseObservation> robotObservatons = new LinkedList<>();
    List<PoseObservation> robotObservatonsAccepted = new LinkedList<>();
    List<PoseObservation> robotObservatonsRejected = new LinkedList<>();
    double totalambiguity = 0;
    // Update Pose
    for (int cameraIndex = 0; cameraIndex < inputs.length; cameraIndex++) {
      for (var obs : inputs[cameraIndex].poseObservations) {
        // filtering
        boolean rejectPose =
            obs.ambiguity() > VisionConstants.maxAmbiguity
                || obs.pose().getZ() > VisionConstants.maxZError
                || !obs.hasTags()
                || obs.pose().getX() < 0
                || obs.pose().getY() < 0
                || obs.pose().getX() > VisionConstants.kTagLayout.getFieldLength()
                || obs.pose().getY() > VisionConstants.kTagLayout.getFieldWidth();

        totalambiguity += obs.ambiguity();
        robotObservatons.add(obs);
        if (rejectPose) {
          robotObservatonsRejected.add(obs);
        } else {
          robotObservatonsAccepted.add(obs);
        }
      }
    }

    /*Returns an accepted pose
     * if there is only one good pose then take that one.
     * if there is more than one, find how ambigous one est is compared to another and average them with a the amb factor
     */
    if (robotObservatonsAccepted.size() >= 1) {
      if (robotObservatonsAccepted.size() == 1) {
        acceptedPose = robotObservatonsAccepted.get(0).pose().toPose2d();
        acceptedPoseGood = true;
      } else {
        int x = 0;
        int y = 0;
        for (int i = 0; i < robotObservatonsAccepted.size(); i++) {
          double ambiguityFactor = robotObservatonsAccepted.get(i).ambiguity() / totalambiguity;
          x += robotObservatonsAccepted.get(i).pose().getX() * ambiguityFactor;
          y += robotObservatonsAccepted.get(i).pose().getY() * ambiguityFactor;
        }
        acceptedPose = new Pose2d(x, y, null);
        acceptedPoseGood = true;
      }
    } else {
      acceptedPoseGood = false;
    }

    Logger.recordOutput("Vision/AprilTag/AcceptedPose", acceptedPose);
    Logger.recordOutput("Vision/AprilTag/AcceptedPoseGood", acceptedPoseGood);
    Logger.recordOutput(
        "Vision/AprilTag/AcceptedVisionMesurmentsCount", robotObservatonsAccepted.size());
  }
}

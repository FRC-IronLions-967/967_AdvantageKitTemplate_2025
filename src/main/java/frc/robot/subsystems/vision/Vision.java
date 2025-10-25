// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  private final ObjectDetectionIO[] objectDetectionIOs;
  private final ObjectDetectionIOInputsAutoLogged[] objectDetectionInputs;
  private final AprilTagIO[] aprilTagIOs;
  private final AprilTagIOInputsAutoLogged[] aprilTagInputs;

  /** Creates a new Vision. */
  public Vision(ObjectDetectionIO[] objectDetectionIOs, AprilTagIO... aprilTagIOs) {
    this.objectDetectionIOs = objectDetectionIOs;
    this.aprilTagIOs = aprilTagIOs;

    this.objectDetectionInputs = new ObjectDetectionIOInputsAutoLogged[objectDetectionIOs.length];
    for (int i = 0; i < objectDetectionInputs.length; i++) {
      objectDetectionInputs[i] = new ObjectDetectionIOInputsAutoLogged();
    }

    this.aprilTagInputs = new AprilTagIOInputsAutoLogged[aprilTagIOs.length];
    for (int i = 0; i < aprilTagInputs.length; i++) {
      aprilTagInputs[i] = new AprilTagIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    // Update Inputs
    for (int i = 0; i < objectDetectionInputs.length; i++) {
      objectDetectionIOs[i].updateInputs(objectDetectionInputs[i]);
      Logger.processInputs(
          "Vision/ObjectDetection/Camera" + Integer.toString(i), objectDetectionInputs[i]);
    }
    for (int i = 0; i < aprilTagInputs.length; i++) {
      aprilTagIOs[i].updateInputs(aprilTagInputs[i]);
      Logger.processInputs("Vision/AprilTag/Camera" + Integer.toString(i), aprilTagInputs[i]);
    }

    //Find vision pose est !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!work here
  }
}

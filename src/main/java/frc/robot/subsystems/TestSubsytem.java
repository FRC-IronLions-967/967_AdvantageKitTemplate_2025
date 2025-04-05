// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubsytem extends SubsystemBase {
  /** Creates a new TestSubsytem. */
  private SparkFlex roller;
  private SparkFlexConfig rollerConfig;
  private SparkClosedLoopController rollerController;

  public TestSubsytem() {

    roller = new SparkFlex(9, MotorType.kBrushless);

    rollerConfig = new SparkFlexConfig();
    rollerConfig.closedLoop.p(1.0).i(0).d(0);

    roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rollerController = roller.getClosedLoopController();
  }

  public void runRoller(double speed) {
    rollerController.setReference(speed, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

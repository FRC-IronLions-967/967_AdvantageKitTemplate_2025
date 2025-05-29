// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimitSwitchManager;

public class Arm extends SubsystemBase {
  /** Creates a new arm. */

  private SparkFlex elevatorVortex;
  private SparkClosedLoopController elevatorVortexController;
  private SparkFlexConfig elevatorVortexConfig;

  private SparkFlex armVortex;
  private SparkClosedLoopController armVortexController;
  private SparkFlexConfig armVortexConfig;

  private SparkFlex coralManipulatorVortex;
  private SparkClosedLoopController coralManipulatorVortexController;
  private SparkFlexConfig coralManipulatorVortexConfig;

  private SparkFlex algaeManipulatorVortex;
  // private SparkClosedLoopController algaeManipulatorVortexController;
  private SparkFlexConfig algaeManipulatorVortexConfig;

  private BooleanSupplier coralInnerLimitSwitch;
  private BooleanSupplier coralOuterLimitSwitch;
  private BooleanSupplier algaeLimitSwitch;

  private double elevatorHeightEndGoal;
  private double elevatorHeightCurrentTarget;
  private double rotaryArmEndGoal;
  private double rotaryArmCurrentTarget;

  private ArmStates state;
  private boolean elevatorHomed;

  public Arm() {
    //Arm Init
    elevatorHeightEndGoal = 5.0;
    rotaryArmEndGoal = Math.PI;
    state = ArmStates.STARTUP;

    //Map Motors
    elevatorVortex = new SparkFlex(9, MotorType.kBrushless);
    elevatorVortexController = elevatorVortex.getClosedLoopController();
    elevatorVortexConfig = new SparkFlexConfig();

    armVortex = new SparkFlex(10, MotorType.kBrushless);
    armVortexController = armVortex.getClosedLoopController();
    armVortexConfig = new SparkFlexConfig();

    coralManipulatorVortex = new SparkFlex(11, MotorType.kBrushless);
    coralManipulatorVortexController = coralManipulatorVortex.getClosedLoopController();
    coralManipulatorVortexConfig = new SparkFlexConfig();

    algaeManipulatorVortex = new SparkFlex(12, MotorType.kBrushless);
    // algaeManipulatorVortexController = algaeManipulatorVortex.getClosedLoopController();
    algaeManipulatorVortexConfig = new SparkFlexConfig();

    elevatorVortexConfig
      .inverted(true)
      .smartCurrentLimit(60)
      .idleMode(IdleMode.kBrake);
    elevatorVortexConfig.encoder
      .positionConversionFactor((2.0 * ArmConstants.elevatorSprocketRadius * Math.PI) / ArmConstants.elevatorGearRatio) // to meters
      .velocityConversionFactor((2.0 * ArmConstants.elevatorSprocketRadius * Math.PI) / (60.0 * ArmConstants.elevatorGearRatio)); //to meters/sec
    elevatorVortexConfig.analogSensor
      .positionConversionFactor(Units.metersToInches(2.0) / 5.0) // native 0-5V, 2 meter travel
      .velocityConversionFactor(Units.metersToInches(2.0) / 5.0); // 
    elevatorVortexConfig.closedLoop
      .outputRange(-0.4, 1)
      .feedbackSensor(FeedbackSensor.kAnalogSensor)
      .pid(0.25, 0, 3);//FF: 0.000139

    elevatorVortex.configure(elevatorVortexConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    armVortexConfig
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake);
    armVortexConfig.absoluteEncoder
      .velocityConversionFactor(2.0 * Math.PI / 60.0)
      .positionConversionFactor(Math.PI * 2)
      .zeroOffset(ArmConstants.kArmZeroOffset / (Math.PI * 2));
    armVortexConfig.closedLoop
      .outputRange(-0.5, 0.5)
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .pid(1.0, 0, 0.1)
      .positionWrappingInputRange(ArmConstants.armWiringMinConstraint, ArmConstants.armWiringMaxConstraint)
      .positionWrappingEnabled(false);

    armVortex.configure(armVortexConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


    coralManipulatorVortexConfig
      .smartCurrentLimit(60)
      .idleMode(IdleMode.kCoast);
    coralManipulatorVortexConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1e-4, 0, 0);

    coralManipulatorVortex.configure(coralManipulatorVortexConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


    algaeManipulatorVortexConfig
      .smartCurrentLimit(60)
      .idleMode(IdleMode.kCoast);
    algaeManipulatorVortexConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1e-4, 0, 0);

    algaeManipulatorVortex.configure(algaeManipulatorVortexConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


    coralInnerLimitSwitch = LimitSwitchManager.getSwitch(2);
    coralOuterLimitSwitch = LimitSwitchManager.getSwitch(1);
    algaeLimitSwitch = LimitSwitchManager.getSwitch(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run  

    // transitions are less important than just knowing the current state
    setArmState();
    switch (state) {
      case HOMING:
        elevatorVortex.set(-1);
        if (elevatorVortex.getOutputCurrent() > 40) {
          elevatorVortex.set(0);
          elevatorVortex.getEncoder().setPosition(0);
          elevatorHomed = true;
        }
      case STARTUP:
        elevatorHeightEndGoal = ArmConstants.armFullRotationElevatorHeight + ArmConstants.elevatorTolerance;
      case EMPTY:

      //Makes Rotation Safe
        if (getElevatorPosition() <= ArmConstants.armFullRotationElevatorHeight + ArmConstants.armTolerance) { // Fudge factor for imperfect positioning
          rotaryArmCurrentTarget = (rotaryArmEndGoal >= ArmConstants.emptyArmConstraintForAlgaeManipulatorAtE0) ? ArmConstants.emptyArmConstraintForAlgaeManipulatorAtE0 : rotaryArmEndGoal;
        } else {
          rotaryArmCurrentTarget = rotaryArmEndGoal;
        }

      //Makes Elevator Safe
        if (elevatorHeightEndGoal < getElevatorPosition() && elevatorHeightEndGoal < ArmConstants.armFullRotationElevatorHeight && getArmAngle() >= ArmConstants.emptyArmConstraintForAlgaeManipulatorAtE0) {
          elevatorHeightCurrentTarget = ArmConstants.armFullRotationElevatorHeight;
        } else {
          elevatorHeightCurrentTarget = elevatorHeightEndGoal;
        }

        break;

      case ALGAE_IN:

      //Makes Rotation Safe
        if (getElevatorPosition() <= ArmConstants.armWithAlgaeFullRotationElevatorHeight) {
          if (rotaryArmEndGoal <= ArmConstants.armWithAlgaeMinConstraint) {
            rotaryArmCurrentTarget = ArmConstants.armWithAlgaeMinConstraint;
          } else if (rotaryArmEndGoal >= ArmConstants.armWithAlgaeMaxConstraint) {
            rotaryArmCurrentTarget = ArmConstants.armWithAlgaeMaxConstraint;
          } else {
            rotaryArmCurrentTarget = rotaryArmEndGoal;
          }
        } else {
          rotaryArmCurrentTarget = (rotaryArmEndGoal >= ArmConstants.armWithAlgaeMaxConstraint) ? ArmConstants.armWithAlgaeMaxConstraint : rotaryArmEndGoal;
        }

      //Makes Elevation Safe
        if (elevatorHeightEndGoal < getElevatorPosition() && elevatorHeightCurrentTarget < ArmConstants.armWithAlgaeFullRotationElevatorHeight 
        && (ArmConstants.armWithAlgaeMinConstraint <= getArmAngle() && getArmAngle() <= ArmConstants.armWithAlgaeMaxConstraint)) {
          elevatorHeightCurrentTarget = ArmConstants.armWithAlgaeFullRotationElevatorHeight;
        } else {
          elevatorHeightCurrentTarget = elevatorHeightEndGoal;
        }

        break;

      default:
        
        break;
    }


    if (state != ArmStates.STARTUP && state != ArmStates.HOMING) {
      armVortexController.setReference(rotaryArmCurrentTarget, ControlType.kPosition);
    }

    elevatorVortexController.setReference(elevatorHeightCurrentTarget, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.09, ArbFFUnits.kPercentOut);
    
  }

  public void changeStateToStartup() {
    state = ArmStates.STARTUP;
  }

  /**
   * Moves the elevator end goal.
   * @param position the value that the elevator goes to.
   */
  public void moveElevator(double position) {
    elevatorHeightEndGoal = position;
  }

  /**
   * 
   * @return The elevator position.
   */
  public double getElevatorPosition() {
    return elevatorVortex.getEncoder().getPosition();
    //return elevatorVortex.getAnalog().getPosition() - ArmConstants.kElevatorAnalogZeroOffset;
  }

  /**
   * Moves the rotary arm to a safe end goal.
   * @param angle the value that the rotary arm goes to.
   */
  public void moveArm(double angle) {
    if (ArmConstants.armWiringMinConstraint >= angle) {
      rotaryArmEndGoal = ArmConstants.armWiringMinConstraint;
    } else if (angle >= ArmConstants.armWiringMaxConstraint) {
      rotaryArmEndGoal = ArmConstants.armWiringMaxConstraint;
    } else {
      rotaryArmEndGoal = angle;
    }
  }

  /**
   * 
   * @return The rotary arm position.
   */
  public double getArmAngle() {
    return armVortex.getAbsoluteEncoder().getPosition();
  }

  /**
   * Sets the coral manipulator to a speed.
   * @param speed The speed at which the wheel runs.
   */
  public void runCoralManipulator(double speed) {
    coralManipulatorVortexController.setReference(speed, ControlType.kVelocity);
    // coralManipulatorVortex.set(speed);
  }

  /**
   * Checks the inner limit switch
   * @return If the coral manipulator has Coral in it.
   */
  public boolean hasCoral() {
    return coralInnerLimitSwitch.getAsBoolean();
  }

  /**
   * Checks the outer limit switch.
   * @return If the robot has coral in the oral manipulator
   */
  public boolean doesntHaveCoral() {
    return !coralOuterLimitSwitch.getAsBoolean();
  }

  /**
   * Sets the Algae manipulator to a speed.
   * @param speed The speed at which the wheels run.
   */
  public void runAlgaeManipulator(double speed) {
    algaeManipulatorVortex.set(speed);
  }

  /**
   * 
   * @return If the algae manipulator has Algae in it.
   */
  public boolean hasAlgae() {
    return algaeLimitSwitch.getAsBoolean();
  }

  /**
   * Checks if the robot arm is in the correct position
   * @return if the arm is in position
   */
  public boolean isArmInPosition() {
    return getArmAngle() - ArmConstants.armTolerance < rotaryArmEndGoal && rotaryArmEndGoal < getArmAngle() + ArmConstants.armTolerance;
  }

  /**
   * Checks if the robot elevator is in the correct position
   * @return If the arm is in position
   */
  public boolean isElevatorInPosition() {
    return getElevatorPosition() - ArmConstants.elevatorTolerance < elevatorHeightEndGoal &&
    elevatorHeightEndGoal < getElevatorPosition() + ArmConstants.elevatorTolerance;
  }

  /**
   * Checks if the whole arm subsystem is in position
   * @return If both the arm and elevator are in position.
   */
  public boolean isInPosition() {
    return isArmInPosition() && isElevatorInPosition();
  }

  /**
   * Sets the Arm State based on what game pieces are in the manipuators.
   */
  private void setArmState() {
    if (state == ArmStates.HOMING && elevatorHomed) {
      state = ArmStates.STARTUP;
    }
    if (elevatorHomed && (state != ArmStates.STARTUP || getElevatorPosition() >= ArmConstants.armFullRotationElevatorHeight)) {
      if (hasAlgae()) {
        state = ArmStates.ALGAE_IN;
      } else {
        state = ArmStates.EMPTY;
      }
    }
  }
}

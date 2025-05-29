package frc.robot.subsystems.arm;

public class ArmConstants {

    public static final double kElevatorAnalogZeroOffset = 4.23;
    public static final double kArmZeroOffset = 1.961;

    public static final double elevatorGearRatio =  14.087; // taken from robot 2/1
    public static final double elevatorSprocketRadius = 0.8755; // inches
    public static final double armGearRatio = 103.448; //taken from CAD 1/28
    public static final double coralWheelRadius = 1.0; 
    public static final double algaeWheelRadius = 1.0; 

    public static final double coralIntakeSpeed = -6500; 
    public static final double coralScoringSpeed = 6500; 
    public static final double algaeIntakeSpeed = -1; 
    public static final double algaeScoringSpeed = 1; 

    public static final double armWiringMinConstraint = 0.0;
    public static final double armWiringMaxConstraint = 4.732;
    public static final double armFullRotationElevatorHeight = 5.0;
    public static final double armWithAlgaeFullRotationElevatorHeight = 23;
    public static final double emptyArmConstraintForAlgaeManipulatorAtE0 = 2.734; // Shouldn't go past this without raising the elevator.
    public static final double armWithAlgaeMinConstraint = 1.551;
    public static final double armWithAlgaeMaxConstraint = 2.103;
    // State coral has no additional constraints.


    // Arm positions
    public static final double L2L3ArmAngle = 4.02;
    public static final double L4ArmAngle = 4.187;
    public static final double coralArmAngle = 0.681;
    public static final double reefAlgaeAngle = 1.710;
    public static final double bargeAlgaeAngle = 0.29;
    public static final double processorAlgaeAngle = 1.89;
    public static final double climbArmAngle = Math.PI / 2;
    public static final double defaultArmAngle = Math.PI;
  
    // Elevator Positions
    public static final double L2ElevatorPosition = 13.27;
    public static final double L3ElevatorPosition = 27.04;
    public static final double L4ElevatorPosition = 54.4;
    public static final double L2AlgaeElevatorPosition = 13;
    public static final double L3AlgaeElevatorPosition = 30;
    public static final double coralElevatorPosition = 26.11;
    public static final double bargeElevatorPosition = 54.0;
    public static final double climbElevatorPosition = 0.0;   
    public static final double processorElevatorPosition = 1.0;

    //Tolerances
    public static final double elevatorTolerance = 1.0;
    public static final double armTolerance = 0.1;
}

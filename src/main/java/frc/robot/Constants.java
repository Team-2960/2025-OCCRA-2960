package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class Constants {

    //Elevator Constants
    public static final AngularVelocity maxElevatorSpeed = AngularVelocity.ofBaseUnits(2, RotationsPerSecond);

    // Drivetrain Constants
    public static final Distance wheelDiameter = Inches.of(3);
    public static final double driveRatio = 1.0/7.31;

    // CAN IDs
    public static final int lfDriveMotorID = 1;
    public static final int lrDriveMotorID = 3;
    public static final int rfDriveMotorID = 2;
    public static final int rrDriveMotorID = 4;

    public static final double elevatorScale = 1;

    public static final Angle elevatorRampDownDist = Angle.ofBaseUnits(10, Rotations);

    
}

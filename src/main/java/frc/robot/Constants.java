package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

public class Constants {
    // Drivetrain Constants
    public static final Distance wheelDiameter = Inches.of(3);
    public static final double driveRatio = 1.0/8.45;

    // CAN IDs
    public static final int lfDriveMotorID = 1;
    public static final int lrDriveMotorID = 2;
    public static final int rfDriveMotorID = 3;
    public static final int rrDriveMotorID = 4;
}

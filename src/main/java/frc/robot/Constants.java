package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

public class Constants {
    // Drivetrain Constants
    public static final Distance wheelDiameter = Inches.of(3);
    public static final double driveRatio = 1.0/7.31;

    // CAN IDs
    public static final int lfDriveMotorID = 9;
    public static final int lbDriveMotorID = 8;
    public static final int rfDriveMotorID = 1;
    public static final int rbDriveMotorID = 2;

    public static final int lIntakeMotorID = 7;
    public static final int rIntakeMotorID = 5;
    public static final int elevatorMotorID = 10;
    public static final int topIndexerMotorID = 3;
    public static final int botIndexMotorID = 4;


}

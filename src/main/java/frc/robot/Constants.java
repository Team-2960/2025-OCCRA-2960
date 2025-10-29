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
    public static final int lfDriveMotorID = 9;
    public static final int lbDriveMotorID = 8;
    public static final int rfDriveMotorID = 1;
    public static final int rbDriveMotorID = 2;

    public static final int lIntakeMotorID = 5;
    public static final int rIntakeMotorID = 7;
    public static final int elevatorMotorID = 10;
    public static final int topIndexerMotorID = 3;
    public static final int botIndexMotorID = 4;
    public static final int endEffectorMotorID = 6;



    public static final double elevatorScale = 1;
    public static final Angle elevatorRampDownDist = Angle.ofBaseUnits(10, Rotations);
    public static final Angle elevTopLim = Rotations.of(12.4);
    public static final Angle elevBotLim = Rotations.of(0.2);

    
}

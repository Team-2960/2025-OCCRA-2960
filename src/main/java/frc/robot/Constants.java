package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {

    //Elevator Constants
    public static final AngularVelocity maxElevatorSpeed = AngularVelocity.ofBaseUnits(12, RotationsPerSecond);
    public static final double elevatorScale = 1;
    public static final Angle elevatorRampDownDist = Angle.ofBaseUnits(0.5, Rotations);
    public static final Angle elevTopLim = Rotations.of(16.52);
    public static final Angle elevBotLim = Rotations.of(0.2);

    // Drivetrain Constants
    public static final Distance wheelDiameter = Inches.of(3);
    public static final double driveRatio = 1.0/7.31;
    public static final LinearVelocity maxDrivetrainSpeed = MetersPerSecond.of(3);
    public static final Distance driveRampDownDist = Meters.of(0.2);
    public static final Distance trackWidth = Inches.of(21.9375);

    // CAN IDs
    public static final int lfDriveMotorID = 1;
    public static final int lbDriveMotorID = 2;
    public static final int rfDriveMotorID = 4;
    public static final int rbDriveMotorID = 3;

    public static final int lIntakeMotq2orID = 5;
    // public static final int rIntakeMotorID = 6;
    public static final int elevatorMotorID = 7;
    // public static final int topIndexerMotorID = 3;
    // public static final int botIndexMotorID = 4;
    public static final int lEndEffectorMotorID = 5;
    public static final int rEndEffectorMotorID = 6;

    // Motor Voltages
    public static final double driveVolt = 8;
    public static final double intakeVolt = 3;
    public static final double topIntakeVolt = 6;
    public static final double botIntakeVolt = 5;
    public static final double indexVolt = 6;
    public static final double rEndEffectorVolt = 6;
    public static final double lEndEffectorVolt = 6;

    

    
}

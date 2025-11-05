package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BNO055;
import frc.robot.util.BNO055.opmode_t;
import frc.robot.util.BNO055.vector_type_t;

/**
 * Manages the robot drivetrain Drivetrain
 */
public class Drivetrain extends SubsystemBase {

    public class DriveLDistanceCmd extends Command{

        private Distance startDistance;
        private Distance distance;
        private Voltage volts;

        public DriveLDistanceCmd(Voltage volts, Distance distance){
            this.startDistance = getLeftDistance();
            this.distance = distance;
            this.volts = volts;
        }

        @Override
        public void initialize(){
            driveLMotor(volts);
        }

        @Override
        public boolean isFinished(){
            return (distance.in(Meters) == (getLeftDistance().in(Meters) - startDistance.in(Meters)));
        }
        
        @Override 
        public void end(boolean interrupted){
            driveLMotor(Volts.zero());
        }

    }


    private final SparkMax lfMotor; // Left Front Drive Motor
    private final SparkMax lrMotor; // Left Rear Drive Motor
    private final SparkMax rfMotor; // Right Front Drive Motor
    private final SparkMax rrMotor; // Right Rear Drive Motor

    private final RelativeEncoder lEncoder; // Left Drive Encoder
    private final RelativeEncoder rEncoder; // Right Drive Encoder
    private final Distance wheelRadius;
    private final double driveRatio;

    private final BNO055 imu;

    /**
     * Constructor
     * 
     * @param lfMotorID   Motor ID for the left front motor. This is the motor that
     *                    will be used for the left encoder.
     * @param lrMotorID   Motor ID for the left rear motor. This motor will be set
     *                    to follow the left front motor.
     * @param lfMotorID   Motor ID for the right front motor. This is the
     *  motor that
     *                    will be used for the right encoder.
     * @param lrMotorID   Motor ID for the right rear motor. This motor will be set
     *                    to follow the right front motor.
     * @param driveRatio  Drive motor gear ratio. All motors are assumed to have the
     *                    same gear ratio. This should be the number of turns of the
     *                    wheel per turn of the motor.
     * @param wheelRadius Drive wheel radius.
     */
    public Drivetrain(int lfMotorID, int lbMotorID, int rfMotorID, int rbMotorID, double driveRatio,
            Distance wheelRadius) {

        // Create Motors
        lfMotor = new SparkMax(lfMotorID, MotorType.kBrushless);
        lrMotor = new SparkMax(lbMotorID, MotorType.kBrushless);
        rfMotor = new SparkMax(rfMotorID, MotorType.kBrushless);
        rrMotor = new SparkMax(rbMotorID, MotorType.kBrushless);

        this.wheelRadius = wheelRadius;

        this.driveRatio = driveRatio;

        imu = BNO055.getInstance(opmode_t.OPERATION_MODE_IMUPLUS, vector_type_t.VECTOR_EULER);
        

        // Calculate the position conversion factor
        double distPerRev = wheelRadius.in(Meters) * Math.PI * driveRatio; // Distance traveled for one revolution of
                                                                           // the motor

        // Configure Left Front Motor
        SparkMaxConfig lfConfig = new SparkMaxConfig();

        // lfConfig.encoder.positionConversionFactor(distPerRev); // Set the left encoder position conversion factor

        lfMotor.configure(lfConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // Configure Left Rear Motor
        SparkMaxConfig lrConfig = new SparkMaxConfig();

        lrConfig.follow(lfMotor); // Set left rear motor to follow left front motor

        lrMotor.configure(lrConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // Configure Right Front Motor
        SparkMaxConfig rfConfig = new SparkMaxConfig();

        // rfConfig.encoder.positionConversionFactor(distPerRev); // Set the right encoder position conversion factor

        rfMotor.configure(rfConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // Configure Left Rear Motor
        SparkMaxConfig rrConfig = new SparkMaxConfig();

        rrConfig.follow(rfMotor); // Set right rear motor to follow right front motor

        rrMotor.configure(rrConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // Get drive encoders
        lEncoder = lfMotor.getEncoder();
        rEncoder = rfMotor.getEncoder();

        lEncoder.setPosition(0);
        rEncoder.setPosition(0);

    }

    /**
     * Sets the drive motor voltage
     * 
     * @param left  voltage to apply to the left drive motors
     * @param right voltage to apply to the right drive motors
     */
    public void setDrive(Voltage left, Voltage right) {
        lfMotor.setVoltage(left);
        rfMotor.setVoltage(right);
    }

    public void driveRMotor(Voltage voltage){
        rfMotor.setVoltage(voltage);
    }
    public void driveLMotor(Voltage voltage){
        lfMotor.setVoltage(voltage);
    }
    public void driveBMotor(Voltage voltage){
        lfMotor.setVoltage(voltage);
        rfMotor.setVoltage(voltage);
    }

    /**
     * Gets the average distance traveled by both sides of the drivetrain
     * 
     * @return average distance traveled by both sides of the drivetrain
     */
    // public Distance getDistance() {
    //     double lDist = lEncoder.getPosition();
    //     double rDist = rEncoder.getPosition();
    //     double avgDist = (lDist + rDist) / 2;

    //     return Inches.of(avgDist);
    // }

    public Angle getRightRotations(){
        return Rotations.of(-rEncoder.getPosition());
    }

    public Angle getLeftRotations(){
        return Rotations.of(-lEncoder.getPosition());
    }

    public Distance getRightDistance(){
        return Meters.of(getRightRotations().in(Rotations) * driveRatio * 2 * wheelRadius.in(Meters) * Math.PI);
    }

    public Distance getLeftDistance(){
        return Meters.of(getLeftRotations().in(Rotations) * driveRatio * 2 * wheelRadius.in(Meters) * Math.PI);
    }


    public Rotation2d getRotation2d(){
        int times = (int) imu.getHeading()/180;
        int multiplier = (times%2 == 1) ? 1 : 0;
        double newAngle = imu.getHeading() - (180 * times) - (180 * multiplier);

        return Rotation2d.fromDegrees(newAngle);
    }

    


    /**
     * Resets the drive distance to zero
     */
    public void resetDistance() {
        lEncoder.setPosition(0);
        rEncoder.setPosition(0);
    }


    /**
     * Drive command factory
     * @param left  left drive voltage supplier
     * @param right right drive voltage supplier
     * @return new drive command
     */
    public Command getDriveCmd(Supplier<Voltage> left, Supplier<Voltage> right) {
        return this.runEnd(
                () -> setDrive(left.get(), right.get()),
                () -> setDrive(Volts.zero(), Volts.zero()));
    }

    public Command getDriveRMotorCmd(Supplier<Voltage> voltage){
        return this.runEnd( 
            () -> driveRMotor(voltage.get()),
            () -> driveRMotor(Volts.zero())
        );
    }

    public Command getDriveLMotorCmd(Supplier<Voltage> voltage){
        return this.runEnd( 
            () -> driveLMotor(voltage.get()),
            () -> driveLMotor(Volts.zero())
        );
    }


    public Command getDriveBMotor(Supplier<Voltage> voltage){
        return this.runEnd(
            () -> driveBMotor(voltage.get()),
            () -> driveBMotor(Volts.zero())
            );
    }
    /**
     * Generates a command to drive forward for a certain distance
     * 
     * @param dist    distance to travel
     * @param voltage voltage to set to the motors
     * @return new command to drive the robot until it reaches a given distance
     */
    // public Command getDriveDistCmd(Distance dist, Voltage voltage) {
    //     return Commands.deadline(Commands.waitUntil(() -> getDistance().gte(dist)), // Waits until the robot has reached
    //                                                                                 // a certain distance
    //             this.startEnd( // Sets the motors to a voltage at the start and turns them off at the end
    //                     () -> setDrive(voltage, voltage), () -> setDrive(Volts.zero(), Volts.zero())));
    // }

    /**
     * Turns the robot for a given amount of time with a set voltage
     * 
     * @param time    time to turn
     * @param voltage voltage to set to the motors. Positive voltage will cause
     *                counter clockwise turning.
     * @return new command to turn the robot for a given amount of time
     */
    public Command getTurnTimeCmd(Time time, Voltage voltage) {
        return Commands.deadline(Commands.waitTime(time), // Waits for a certain amount of time
                this.startEnd( // Sets the drive motors to equal but opposite voltages at the starts and turns
                               // them off a the end
                        () -> setDrive(voltage, voltage.unaryMinus()), () -> setDrive(Volts.zero(), Volts.zero())));
    }

    public Command getDriveLDistanceCmd(Voltage volts, Distance distance){
        
        Distance startDistance = getLeftDistance();

        return this.runEnd(
                () -> driveLMotor(volts), 
                () -> driveLMotor(Volts.of(0)))
            .until(() -> distance.in(Meters) == (getLeftDistance().in(Meters) - startDistance.in(Meters)));
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Rotation", getRotation2d().getDegrees());
        SmartDashboard.putNumber("Raw Rotation", imu.getHeading());
        SmartDashboard.putNumber("Left Encoder", lEncoder.getPosition());
        SmartDashboard.putNumber("Left Distance", getLeftDistance().in(Meters));
        SmartDashboard.putNumber("Right Distance", getRightDistance().in(Meters));
    }
}

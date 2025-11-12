package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.LimitDirection;
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
        public void execute(){
        }

        @Override
        public boolean isFinished(){
            return (distance.in(Meters) <= (getLeftDistance().in(Meters) - startDistance.in(Meters)));
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

    private final DifferentialDrive diffDrive;

    private final MutVoltage appliedVoltage;
    private final MutCurrent appliedCurrent;
    private final MutAngle angle;
    private final MutLinearVelocity linearVelocity;

    private final SysIdRoutine sysIdRoutine;

    public final Command sysIdCommandUpQuasi;
    public final Command sysIdCommandDownQuasi;
    public final Command sysIdCommandUpDyn;
    public final Command sysIdCommandDownDyn;
    public final Command sysIdCommandGroup;

    //Feedback Controller
    private final PIDController drivePID;
    private final SimpleMotorFeedforward driveFF;
    private final PIDController anglePID;

    private Distance startDistanceL = Meters.of(0);
    private Distance startDistanceR = Meters.of(0);

    private final BNO055 imu;

    // private AnalogGyro gyro;

    private final DifferentialDriveKinematics kinematics;
    private final DifferentialDrivePoseEstimator poseEstimator;
    private RobotConfig config;

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

        imu = BNO055.getInstance(opmode_t.OPERATION_MODE_GYRONLY, vector_type_t.VECTOR_EULER);
        // byte address = 0x29;
        // imu = BNO055.getInstance(opmode_t.OPERATION_MODE_IMUPLUS, vector_type_t.VECTOR_EULER, Port.kOnboard, address);       
        
        // gyro = new AnalogGyro(0);

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

        //Set up differential drive class
        diffDrive = new DifferentialDrive(lfMotor, rfMotor);

        //Feedback Controllers
        drivePID = new PIDController(0.20798, 0, 0);
        driveFF = new SimpleMotorFeedforward(0.17074, 1.9301, 0.55485);
        anglePID = new PIDController(0, 0, 0);

        anglePID.enableContinuousInput(-180, 180);

        anglePID.setTolerance(1);

        //System Identification
        appliedVoltage = Volts.mutable(0);
        appliedCurrent = Amps.mutable(0);
        angle = Rotations.mutable(0);
        linearVelocity =  MetersPerSecond.mutable(0);

        sysIdRoutine = new SysIdRoutine(
            new Config(
                Volts.per(Second).of(.5),
                Volts.of(2),
                Seconds.of(4)
            ), 
            new Mechanism(
                this::driveBMotor,
                this::sysIDLogging, this)
        );


        sysIdCommandUpQuasi = sysIdRoutine.quasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward);
        sysIdCommandDownQuasi = sysIdRoutine.quasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse);
        sysIdCommandUpDyn = sysIdRoutine.dynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward);
        sysIdCommandDownDyn = sysIdRoutine.dynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse);

        sysIdCommandGroup =  new SequentialCommandGroup(
            sysIdCommandUpQuasi,
            sysIdCommandDownQuasi,
            sysIdCommandUpDyn,
            sysIdCommandDownDyn
        );

        kinematics = new DifferentialDriveKinematics(Constants.trackWidth.in(Meters));
        poseEstimator = new DifferentialDrivePoseEstimator(kinematics, getRotation2d(), 0, 0, new Pose2d());

        try{
        config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
        }

    // Configure AutoBuilder last
        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
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

    public void setLRate(LinearVelocity rate){
        double pidVolt = drivePID.calculate(getLRate().in(MetersPerSecond));
        double ffVolt = driveFF.calculate(rate.in(MetersPerSecond));
        driveLMotor(Volts.of(pidVolt + ffVolt));
    }

    public void setRRate(LinearVelocity rate){
        double pidVolt = drivePID.calculate(getRRate().in(MetersPerSecond));
        double ffVolt = driveFF.calculate(rate.in(MetersPerSecond));

        driveRMotor(Volts.of(pidVolt + ffVolt));
    }

    public void setRate(LinearVelocity left, LinearVelocity right){
        setLRate(left);
        setRRate(right);
    }
    
    public void setTankDrive(double leftStick, double rightStick){
        diffDrive.tankDrive(leftStick, rightStick);
    }

    private void setArcadeDrive(double leftStick, double rightStick){
        diffDrive.arcadeDrive(leftStick, rightStick);
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
        double left = kinematics.toWheelSpeeds(chassisSpeeds).leftMetersPerSecond;
        double right = kinematics.toWheelSpeeds(chassisSpeeds).rightMetersPerSecond;

        setLRate(MetersPerSecond.of(left));
        setRRate(MetersPerSecond.of(right));
    }

    public void setLPos(Distance targetPos, Distance startPos) {
        // Calculate trapezoidal profile
        double maxPosRate = Constants.maxDrivetrainSpeed.in(MetersPerSecond);
        Distance posError = targetPos.minus(getLeftDistance().minus(startPos));

        double targetSpeed = maxPosRate * ((posError.in(Meters) > 0 ? 1 : -1));
        //double rampDownSpeed = posError / Constants.elevatorRampDownDist * maxPosRate;
        double rampDownSpeed = posError.div(Constants.driveRampDownDist.times(maxPosRate)).magnitude();

        if (Math.abs(rampDownSpeed) < Math.abs(targetSpeed))
            targetSpeed = rampDownSpeed;
        
        setLRate(MetersPerSecond.of(targetSpeed));
    }

    public void setRPos(Distance targetPos, Distance startPos) {
        // Calculate trapezoidal profile
        double maxPosRate = Constants.maxDrivetrainSpeed.in(MetersPerSecond);
        Distance posError = targetPos.minus(getRightDistance().minus(startPos));

        double targetSpeed = maxPosRate * ((posError.in(Meters) > 0 ? 1 : -1));
        //double rampDownSpeed = posError / Constants.elevatorRampDownDist * maxPosRate;
        double rampDownSpeed = posError.div(Constants.driveRampDownDist.times(maxPosRate)).magnitude();

        if (Math.abs(rampDownSpeed) < Math.abs(targetSpeed))
            targetSpeed = rampDownSpeed;
        
        setRRate(MetersPerSecond.of(targetSpeed));
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
        return Rotations.of(rEncoder.getPosition());
    }

    public Angle getLeftRotations(){
        return Rotations.of(lEncoder.getPosition());
    }

    public Distance getRightDistance(){
        return Meters.of(getRightRotations().in(Rotations) * driveRatio * 2 * wheelRadius.in(Meters) * Math.PI);
    }

    /**
     * Returns distance traveled calculated by the rotations, gear ratio, and the circumference(2*pi*r) of the wheel
     * @return
     */
    public Distance getLeftDistance(){
        return Meters.of(getLeftRotations().in(Rotations) * driveRatio * 2 * wheelRadius.in(Meters) * Math.PI);
    }

    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

    public DifferentialDriveWheelPositions getDriveWheelPositions(){
        return new DifferentialDriveWheelPositions(getLeftDistance(), getRightDistance());
    }

    public DifferentialDriveWheelSpeeds getDriveWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(getLRate(), getRRate());
    }

    public ChassisSpeeds getChassisSpeeds(){
        return kinematics.toChassisSpeeds(getDriveWheelSpeeds());
    }

    public void resetPose(Pose2d pose){
        poseEstimator.resetPose(pose);
    }

    private void setStartDistanceL(Distance startDistance){
        this.startDistanceL = startDistance;
    }

    private void setStartDistanceR(Distance startDistance){
        this.startDistanceR = startDistance;
    }

    public Rotation2d getRotation2d(){
        int times = (int) imu.getHeading()/180;
        int multiplier = (times%2 == 1) ? 1 : 0;
        double newAngle = imu.getHeading() - (180 * times) - (180 * multiplier);

        return Rotation2d.fromDegrees(newAngle);
    }

    public Rotation2d toRotation2d(double angle){
        angle %= 360;

        if (angle >= 180.0)
            angle -= 360.0;
        else if (angle < -180.0)
            angle += 360.0;

        return Rotation2d.fromDegrees(angle);
    }

    
    public void setAngle(Angle tarAngle){
        double rate = anglePID.calculate(getRotation2d().getDegrees(), tarAngle.in(Degrees));

        setRate(MetersPerSecond.of(rate), MetersPerSecond.of(-rate));
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

    public Command getTankDriveCmd(Supplier<Double> leftStick, Supplier<Double> rightStick){
        return this.runEnd(
            () -> setTankDrive(leftStick.get(), rightStick.get()), 
            () -> setDrive(Volts.zero(), Volts.zero())
        );
    }

    public Command getArcadeDriveCmd(Supplier<Double> leftStick, Supplier<Double> rightStick){
        return this.runEnd(
            () -> setArcadeDrive(leftStick.get(), rightStick.get()), 
            () -> setDrive(Volts.zero(), Volts.zero())
        );
    }

    public Voltage getLVoltage(){
        return Voltage.ofBaseUnits(lfMotor.getAppliedOutput() * lfMotor.getBusVoltage(), Volts);
    }

    public Current getLCurrent(){
        return Amps.of(lfMotor.getOutputCurrent());
    }

    public LinearVelocity getLRate(){
        return MetersPerSecond.of(lEncoder.getVelocity()/60 * driveRatio * 2 * wheelRadius.in(Meters) * Math.PI);
    }

    public LinearVelocity getRRate(){
        return MetersPerSecond.of(rEncoder.getVelocity()/60 * driveRatio * 2 * wheelRadius.in(Meters) * Math.PI);
    }


    private void sysIDLogging(SysIdRoutineLog log){
        log.motor("Drivetrain")
            .voltage(getLVoltage())
            .current(getLCurrent())
            .linearVelocity(getLRate())
            .linearPosition(getLeftDistance());
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
    
        return this.runOnce(() -> setStartDistanceL(getLeftDistance()))
            .andThen(
                this.runEnd(
                    () -> driveLMotor(volts), 
                    () -> driveLMotor(Volts.of(0))
                )
            )
            .until(() -> distance.in(Meters) <= (getLeftDistance().in(Meters) - this.startDistanceL.in(Meters)));
    }

    public Command getDriveRDistanceCmd(Voltage volts, Distance distance){
    
        return this.runOnce(() -> setStartDistanceR(getRightDistance()))
            .andThen(
                this.runEnd(
                    () -> driveRMotor(volts), 
                    () -> driveRMotor(Volts.of(0))
                )
            )
            .until(() -> distance.in(Meters) <= (getRightDistance().in(Meters) - this.startDistanceL.in(Meters)));
    }

    public Command getDriveDistanceCmd(Voltage volts, Distance left, Distance right){
        return this.runOnce(() -> {
                setStartDistanceR(getRightDistance());
                setStartDistanceL(getLeftDistance());
            })
            .andThen(
                this.runEnd(
                    () -> {
                        setDrive(volts, volts);
                    }, 
                    () -> driveRMotor(Volts.of(0))
                )
                .until(
                    () -> right.in(Meters) <= (getRightDistance().in(Meters) - this.startDistanceL.in(Meters))
                        && left.in(Meters) <= (getLeftDistance().in(Meters) - this.startDistanceL.in(Meters)))
            );
    }

    public Command getDriveRateCmd(LinearVelocity left, LinearVelocity right){
        return this.runEnd(
        () -> setRate(left, right),
        () -> setDrive(Volts.zero(), Volts.zero()));
    }

    public Command getDrivePosCmd(Distance left, Distance right){
        return this.runOnce(() -> setStartDistanceR(getRightDistance()))
            .andThen(this.runOnce(() -> setStartDistanceL(getLeftDistance())))
            .andThen(
                this.run(() -> {
                    setLPos(left, this.startDistanceL);
                    setRPos(right, this.startDistanceR);
                })
                .until(
                    () -> left.isNear(getLeftDistance().minus(this.startDistanceL), Meters.of(0.01)) 
                        && right.isNear(getLeftDistance().minus(this.startDistanceR), Meters.of(0.01)))
            );
            
    }

    public Command getDriveToAngleCmd(Angle angle){
        return this.runEnd(
            () -> setAngle(angle), 
            () -> setDrive(Volts.zero(), Volts.zero())
        );
    }

    public Command getSysIdCommandGroup(){
        return sysIdCommandGroup;
    }

    public String getStringCommand(){
        String commandName = "";
        if (getCurrentCommand() == null){
            commandName = "null";
        } else{
            commandName = getCurrentCommand().getName();
        }
        
        return commandName;
    }


    @Override
    public void periodic(){
        poseEstimator.update(getRotation2d(), getDriveWheelPositions());

       // SmartDashboard.putNumber("Rotation", toRotation2d(gyro.getAngle()).getDegrees());
        //SmartDashboard.putNumber("Raw Rotation", gyro.getAngle());
        SmartDashboard.putBoolean("Gyro Initialized", imu.isInitialized());
        SmartDashboard.putBoolean("Gyro Present", imu.isSensorPresent());
        SmartDashboard.putNumber("Left Encoder", lEncoder.getPosition());
        SmartDashboard.putNumber("Left Distance", getLeftDistance().in(Meters));
        SmartDashboard.putNumber("Right Distance", getRightDistance().in(Meters));
        SmartDashboard.putString("Drivetrain Command", getStringCommand());
        SmartDashboard.putNumber("Drivetrain Rate", getRRate().in(MetersPerSecond));
    }
}

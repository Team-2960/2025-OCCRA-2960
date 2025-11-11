package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {

    //Set Variables
    private final SparkMax lEndEffectorMotor;
    private final SparkMax rEndEffectorMotor;

    private final SparkMaxConfig lConfig;
    private final SparkMaxConfig rConfig;

    private final RelativeEncoder lEncoder;
    private final RelativeEncoder rEncoder;

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

    public final PIDController pidController;
    public final SimpleMotorFeedforward ffController;

    private Angle startPosL = Rotations.zero();
    private Angle startPosR = Rotations.zero();


    //Sets EndEffector motor types
    public EndEffector(int lEndEffectorMotorID, int rEndEffectorMotorID) {
    
        lEndEffectorMotor = new SparkMax(lEndEffectorMotorID, MotorType.kBrushless);
        rEndEffectorMotor = new SparkMax(rEndEffectorMotorID, MotorType.kBrushless);

        lConfig = new SparkMaxConfig();
        rConfig = new SparkMaxConfig();

        lConfig.idleMode(IdleMode.kBrake);
        rConfig.idleMode(IdleMode.kBrake);

        lConfig.inverted(true);

        lEndEffectorMotor.configure(lConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rEndEffectorMotor.configure(rConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        lEncoder = lEndEffectorMotor.getEncoder();
        rEncoder = rEndEffectorMotor.getEncoder();

        pidController = new PIDController(0.1, 0, 0);
        ffController = new SimpleMotorFeedforward(0.1, 0.08, 0.01);

        setDefaultCommand(getHoldEndEffectorCmd());

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
                this::setVoltage,
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
    }

    //Sets EndEffector motors to run inverse of eachother at set voltage
    public void setVoltage(Voltage voltage) {
        rEndEffectorMotor.setVoltage(voltage);
        lEndEffectorMotor.setVoltage(voltage.times(-1));
    }

    //Sets EndEffector motors to run at individual voltages
    public void setVoltage(Voltage leftVolt, Voltage rightVolt){
        rEndEffectorMotor.setVoltage(rightVolt);
        lEndEffectorMotor.setVoltage(leftVolt);
    }

    public void setLVoltage(Voltage volts){
        lEndEffectorMotor.setVoltage(volts);
    }

    public void setRVoltage(Voltage volts){
        rEndEffectorMotor.setVoltage(volts);
    }

    public Voltage getVoltage(){
        return Volts.of(lEndEffectorMotor.getBusVoltage() * lEndEffectorMotor.getAppliedOutput());
    }

    public Current getLCurrent(){
        return Amps.of(lEndEffectorMotor.getOutputCurrent());
    }

    public AngularVelocity getLRate(){
        return RotationsPerSecond.of(lEncoder.getVelocity()/60);
    }

    public AngularVelocity getRRate(){
        return RotationsPerSecond.of(rEncoder.getVelocity()/60);
    }

    public Angle getLPos(){
        return Rotations.of(lEncoder.getPosition());
    }

    public Angle getRPos(){
        return Rotations.of(rEncoder.getPosition());
    }

    public void setStartPosL(Angle startPos){
        this.startPosL = startPos;
    }
    
    public void setStartPosR(Angle startPos){
        this.startPosR = startPos;
    }

    public void setRRate(AngularVelocity rate){
        double pid = pidController.calculate(getRRate().in(RotationsPerSecond), rate.in(RotationsPerSecond));
        double ff = ffController.calculate(rate.in(RotationsPerSecond));
        setRVoltage(appliedVoltage);
    }

    public void setLRate(AngularVelocity rate){
        double pid = pidController.calculate(getLRate().in(RotationsPerSecond), rate.in(RotationsPerSecond));
        double ff = ffController.calculate(rate.in(RotationsPerSecond));
        setLVoltage(appliedVoltage);
    }

    public void setLPos(Angle targetPos) {

        // Calculate trapezoidal profile
        Angle currentPos = getLPos();
        AngularVelocity maxPosRate = RotationsPerSecond.of(8);
        Angle posError = targetPos.minus(currentPos);

        AngularVelocity targetSpeed = maxPosRate.times((posError.in(Rotations) > 0 ? 1 : -1));
        //double rampDownSpeed = posError / Constants.elevatorRampDownDist * maxPosRate;
        AngularVelocity rampDownSpeed = AngularVelocity.ofBaseUnits(posError.div(Rotations.of(0.2).times(maxPosRate)).magnitude(), RotationsPerSecond);

        if (Math.abs(rampDownSpeed.in(RotationsPerSecond)) < Math.abs(targetSpeed.in(RotationsPerSecond)))
            targetSpeed = rampDownSpeed;
        
        setLRate(targetSpeed);
    }

    public void setRPos(Angle targetPos) {

        // Calculate trapezoidal profile
        Angle currentPos = getRPos();
        AngularVelocity maxPosRate = RotationsPerSecond.of(8);
        Angle posError = targetPos.minus(currentPos);

        AngularVelocity targetSpeed = maxPosRate.times((posError.in(Rotations) > 0 ? 1 : -1));
        //double rampDownSpeed = posError / Constants.elevatorRampDownDist * maxPosRate;
        AngularVelocity rampDownSpeed = AngularVelocity.ofBaseUnits(posError.div(Rotations.of(0.2).times(maxPosRate)).magnitude(), RotationsPerSecond);

        if (Math.abs(rampDownSpeed.in(RotationsPerSecond)) < Math.abs(targetSpeed.in(RotationsPerSecond)))
            targetSpeed = rampDownSpeed;
        
        setRRate(targetSpeed);
    }

    private void sysIDLogging(SysIdRoutineLog log){
        log.motor("Drivetrain")
            .voltage(getVoltage())
            .current(getLCurrent())
            .angularVelocity(getLRate())
            .angularPosition(getLPos());
    }

    /**
     * @return Command to run both EndEffector motors at same voltage
     */
    // public Command getEndEffectorCmd(Supplier<Voltage> voltage) {
    //     return this.runEnd(
    //             () -> setVoltage(voltage.get()),
    //             () -> setRate(RotationsPerSecond.zero()));
    // }

    /**
     * @return Command to run both EndEffector motors at individual voltages
     */
    public Command getIndEndEffectorCmd(Supplier<Voltage> lVoltage, Supplier<Voltage> rVoltage) {
        return this.runOnce(() -> {
                setStartPosR(getRPos()); 
                setStartPosL(getLPos());
            }).andThen(this.runEnd(
                () -> setVoltage(lVoltage.get(), rVoltage.get()),
                () -> {
                    setRPos(this.startPosR);
                    setLPos(this.startPosL);
                }
                )
            );
    } 

    public Command getHoldEndEffectorCmd(){
        return this.run(() -> {setRRate(RotationsPerSecond.zero()); setLRate(RotationsPerSecond.zero());});
    }
}

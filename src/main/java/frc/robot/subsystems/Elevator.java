package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;

public class Elevator extends SubsystemBase{

    private final SparkMax elevMotor;
    private final SparkFlexConfig elevConfig;
    private final PIDController elevPID;
    private final RelativeEncoder relEncoder;

    //System Identification
    private final MutVoltage appliedVoltage;
    private final MutCurrent appliedCurrent;
    private final MutAngle angle;
    private final MutAngularVelocity angularVelocity;

    private final SysIdRoutine sysIdRoutine;

    public final Command sysIdCommandUpQuasi;
    public final Command sysIdCommandDownQuasi;
    public final Command sysIdCommandUpDyn;
    public final Command sysIdCommandDownDyn;
    public final Command sysIdCommandGroup;

    public Elevator(int elevMotorID){
        elevMotor = new SparkMax(elevMotorID, MotorType.kBrushless);
        elevPID = new PIDController(0.001, 0, 0);
        relEncoder = elevMotor.getEncoder();

        elevConfig = new SparkFlexConfig();
        elevConfig.encoder
            .positionConversionFactor(Constants.elevatorScale)
            .velocityConversionFactor(Constants.elevatorScale / 60.0);

        appliedVoltage = Volts.mutable(0);
        appliedCurrent = Amps.mutable(0);
        angle = Rotations.mutable(0);
        angularVelocity =  RotationsPerSecond.mutable(0);

        sysIdRoutine = new SysIdRoutine(
            new Config(
                Volts.per(Second).of(.5),
                Volts.of(7),
                Seconds.of(10)
            ), 
            new Mechanism(
                this::setVoltage,
                this::sysIDLogging, this)
        );

    }
    public void setVoltage(Voltage voltage){
        elevMotor.setVoltage(voltage);
    }

    public void setRate(AngularVelocity rate){
        double voltage = elevPID.calculate(getRate().in(RadiansPerSecond), rate.in(RadiansPerSecond));
        setVoltage(Volts.of(0));
    }

    public void setElevatorPos(Angle targetPos) {
        
        // Calculate trapezoidal profile
        Angle currentPos = getElevatorPos();
        AngularVelocity maxPosRate = Constants.maxElevatorSpeed;
        Angle posError = targetPos.minus(currentPos);

        AngularVelocity targetSpeed = maxPosRate.times((posError.in(Rotations) > 0 ? 1 : -1));
        //double rampDownSpeed = posError / Constants.elevatorRampDownDist * maxPosRate;
        AngularVelocity rampDownSpeed = AngularVelocity.ofBaseUnits(posError.div(Constants.elevatorRampDownDist.times(maxPosRate)).magnitude(), RotationsPerSecond);

        if (Math.abs(rampDownSpeed.in(RotationsPerSecond)) < Math.abs(targetSpeed.in(RotationsPerSecond)))
            targetSpeed = rampDownSpeed;
        
        setRate(targetSpeed);
    }

    public AngularVelocity getRate(){
        return AngularVelocity.ofBaseUnits(relEncoder.getVelocity()/60, RotationsPerSecond);
    }

    public Angle getElevatorPos(){
        return Angle.ofBaseUnits(relEncoder.getPosition(), Rotations);
    }

    public Command getElevVoltCmd(Supplier<Voltage> voltage){
        return this.runEnd(
            () -> setVoltage(voltage.get()),
            () -> setVoltage(Volts.zero()));
    }


    public Command getElevRateCmd(Supplier<MutAngularVelocity> rate){
        return this.runEnd(
            () -> setRate(rate.get()),
            () -> setRate(AngularVelocity.ofBaseUnits(0, RadiansPerSecond)));
    }

    public Voltage getElevVoltage(){
        return Voltage.ofBaseUnits(elevMotor.getAppliedOutput() * elevMotor.getBusVoltage(), Volts);
    }

    public Current getElevCurrent(){
        return Amps.of(elevMotor.getOutputCurrent());
    }



    private void sysIDLogging(SysIdRoutineLog log){
        log.motor("Elevator")
            .voltage(getElevVoltage())
            .current(getElevCurrent())
            .angularVelocity(getRate())
            .angularPosition(getElevatorPos());
    }
}

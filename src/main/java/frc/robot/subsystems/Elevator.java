package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.*;
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

public class Elevator extends SubsystemBase{

    public enum LimitDirection{
        UP,
        DOWN
    }

    private final SparkMax elevMotor;
    private final SparkFlexConfig elevConfig;
    private final PIDController elevPID;
    private final SimpleMotorFeedforward elevFF;
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

    public Angle lastElevatorPos;
    
    public Elevator(int elevMotorID){
        
        elevMotor = new SparkMax(elevMotorID, MotorType.kBrushless);
        elevPID = new PIDController(0.014517, 0, 0);
        elevFF = new SimpleMotorFeedforward(0.15215, 0.66776, 0.031198);
        relEncoder = elevMotor.getEncoder();

        elevConfig = new SparkFlexConfig();
        elevConfig.idleMode(IdleMode.kCoast);
        elevConfig.inverted(true);

        elevMotor.configure(elevConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // elevConfig.encoder
        //     .positionConversionFactor(Constants.elevatorScale)
        //     .velocityConversionFactor(Constants.elevatorScale / 60.0);

        appliedVoltage = Volts.mutable(0);
        appliedCurrent = Amps.mutable(0);
        angle = Rotations.mutable(0);
        angularVelocity =  RotationsPerSecond.mutable(0);

        setDefaultCommand(getHoldPosCmd());

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

        sysIdCommandUpQuasi = sysIdRoutine.quasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward);
        sysIdCommandDownQuasi = sysIdRoutine.quasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse);
        sysIdCommandUpDyn = sysIdRoutine.dynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward);
        sysIdCommandDownDyn = sysIdRoutine.dynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse);

        sysIdCommandGroup =  new SequentialCommandGroup(

        
            new ParallelRaceGroup(
                sysIdCommandUpQuasi,
                getTopLimCommand(LimitDirection.UP)
            ),
            new ParallelRaceGroup(
                sysIdCommandDownQuasi,
                getBotLimitCommand(LimitDirection.DOWN)
            ),
            new ParallelRaceGroup(
                sysIdCommandUpDyn,
                getTopLimCommand(LimitDirection.UP)
            ),
            new ParallelRaceGroup(
                sysIdCommandDownDyn,
                getBotLimitCommand(LimitDirection.DOWN)
            )
        );

        lastElevatorPos = Rotations.of(0);
    }
    public void setVoltage(Voltage voltage){
        elevMotor.setVoltage(voltage);
    }

    public void setRate(AngularVelocity rate){
        double pidVolt = elevPID.calculate(getRate().in(RotationsPerSecond), rate.in(RotationsPerSecond));
        double ffVolt = elevFF.calculate(rate.in(RadiansPerSecond));

        setVoltage(Volts.of(pidVolt + ffVolt));
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

    public boolean getTopLimit(LimitDirection direction){
        return getElevatorPos().gte(Constants.elevTopLim) && direction.equals(LimitDirection.UP);
    }

    public boolean getBotLimit(LimitDirection direction){
        return (Constants.elevBotLim.gte(getElevatorPos()) && direction.equals(LimitDirection.DOWN));
    }

    public LimitDirection getDirection(double value){
        if (value >= 0){
            return LimitDirection.UP;
        }else{
            return LimitDirection.DOWN;
        }
    }

    public Command getTopLimCommand(LimitDirection direction){
        return Commands.waitUntil(() -> getTopLimit(direction));
    }

    public Command getBotLimitCommand(LimitDirection direction){
        return Commands.waitUntil(() -> getBotLimit(direction));
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

    public Command getElevVoltCmd(Supplier<Voltage> voltage){
        return this.runEnd(
            () -> setVoltage(voltage.get()),
            () -> setVoltage(Volts.zero()));
    }


    public Command getElevRateCmd(Supplier<AngularVelocity> rate){
        LimitDirection direction = getDirection(rate.get().in(RotationsPerSecond));

        return this.runEnd(
            () -> setRate(rate.get()),
            () -> setVoltage(Volts.zero())).unless(() -> getBotLimit(direction)).unless(() -> getTopLimit(direction)).finallyDo(() -> this.lastElevatorPos = getElevatorPos());
    }

    public Command getElevatorPosCmd(Supplier<Angle> position){
        LimitDirection direction = getDirection(position.get().in(Rotations));

        return this.runEnd(
            () -> setElevatorPos(position.get()),
            () -> setVoltage(Volts.zero())).unless(() -> getBotLimit(direction)).unless(() -> getTopLimit(direction)).finallyDo(() -> this.lastElevatorPos = getElevatorPos());
    }


    public Command getHoldPosCmd(){
        return this.run(() -> setElevatorPos(this.lastElevatorPos));
    }

    public Command getSysIdCommandGroup(){
        return sysIdCommandGroup;
    }

    public String getCommandString(){
        if (!(this.getCurrentCommand() == null)){
            return this.getCurrentCommand().getName();
        }else{
            return "null";
        }
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("In Voltage", elevMotor.getAppliedOutput());
        SmartDashboard.putString("Elev Command", getCommandString());
        SmartDashboard.putNumber("Elevator Rotations", getElevatorPos().in(Rotations));
        SmartDashboard.putBoolean("Elev Top Lim Reached", getTopLimit(LimitDirection.UP));
        SmartDashboard.putBoolean("Elev Bot Lim Reached", getBotLimit(LimitDirection.DOWN));
        SmartDashboard.putNumber("Elevator Rate", getRate().in(RotationsPerSecond));
    }

}

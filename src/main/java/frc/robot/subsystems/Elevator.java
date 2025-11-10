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

    //System Identification
    public enum LimitDirection{
        UP,
        DOWN
    }

    private final SparkMax elevMotor;
    private final SparkFlexConfig elevConfig;
    private final PIDController elevPID;
    private final SimpleMotorFeedforward elevFF;
    private final RelativeEncoder relEncoder;

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
        
        //Assign motor type to elevator motor 
        elevMotor = new SparkMax(elevMotorID, MotorType.kBrushless);

        //Sets PID Values
        elevPID = new PIDController(0.014517, 0, 0);

        //Sets FeedForward values
        elevFF = new SimpleMotorFeedforward(0.15215, 0.66776, 0.031198);

        //Indentifies Motor Encoder
        relEncoder = elevMotor.getEncoder();

        //Sets REV Motor Configuations
        elevConfig = new SparkFlexConfig();
        elevConfig.idleMode(IdleMode.kCoast);
        elevConfig.inverted(true);
        elevMotor.configure(elevConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // elevConfig.encoder
        //     .positionConversionFactor(Constants.elevatorScale)
        //     .velocityConversionFactor(Constants.elevatorScale / 60.0);

        //Sets up System Identification Mutable values
        appliedVoltage = Volts.mutable(0);
        appliedCurrent = Amps.mutable(0);
        angle = Rotations.mutable(0);
        angularVelocity =  RotationsPerSecond.mutable(0);

        //Sets Elevator Default Command to Hold positon
        setDefaultCommand(getHoldPosCmd());

        //System ID Routine to configure FF and PID values, configuration for routine.
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

        //Commands to run System ID config routine
        sysIdCommandUpQuasi = sysIdRoutine.quasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward);
        sysIdCommandDownQuasi = sysIdRoutine.quasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse);
        sysIdCommandUpDyn = sysIdRoutine.dynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward);
        sysIdCommandDownDyn = sysIdRoutine.dynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse);

        //Command Group to run SysID Commands in Parallel
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

        //Sets up last elevator position for the Hold Position Command
        lastElevatorPos = Rotations.of(0);
    }

    //Method to set Elevator Motor Voltage
    public void setVoltage(Voltage voltage){
        elevMotor.setVoltage(voltage);
    }

    //Method to set PID & FF rate
    public void setRate(AngularVelocity rate){
        double pidVolt = elevPID.calculate(getRate().in(RotationsPerSecond), rate.in(RotationsPerSecond));
        double ffVolt = elevFF.calculate(rate.in(RadiansPerSecond));

        setVoltage(Volts.of(pidVolt + ffVolt));
    }

    //Method for setting Elevator Position using trapezoidal function
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

    //Returns Elevator Rate
    public AngularVelocity getRate(){
        return AngularVelocity.ofBaseUnits(relEncoder.getVelocity()/60, RotationsPerSecond);
    }

    //Returns Elevator Angle Position
    public Angle getElevatorPos(){
        return Angle.ofBaseUnits(relEncoder.getPosition(), Rotations);
    }

    //Returns Top Elevator Limit Value as either true or false
    public boolean getTopLimit(LimitDirection direction){
        return getElevatorPos().gte(Constants.elevTopLim) && direction.equals(LimitDirection.UP);
    }

    //Returns Bottom Elevator Limit Value as either true or false
    public boolean getBotLimit(LimitDirection direction){
        return (Constants.elevBotLim.gte(getElevatorPos()) && direction.equals(LimitDirection.DOWN));
    }

    //Uses Elevator Rotation value to determine whether to use top or bottom Limit
    public LimitDirection getDirection(double value){
        if (value >= 0){
            return LimitDirection.UP;
        }else{
            return LimitDirection.DOWN;
        }
    }

    //Command to return Top Limit Value
    public Command getTopLimCommand(LimitDirection direction){
        return Commands.waitUntil(() -> getTopLimit(direction));
    }

    //Command to return Bottom Limit Value
    public Command getBotLimitCommand(LimitDirection direction){
        return Commands.waitUntil(() -> getBotLimit(direction));
    }

    //Returns Elevator Voltage Value in Volts
    public Voltage getElevVoltage(){
        return Voltage.ofBaseUnits(elevMotor.getAppliedOutput() * elevMotor.getBusVoltage(), Volts);
    }

    //Returns Elevator Motor Amps in Amps
    public Current getElevCurrent(){
        return Amps.of(elevMotor.getOutputCurrent());
    }

    //Logs System ID
    private void sysIDLogging(SysIdRoutineLog log){
        log.motor("Elevator")
            .voltage(getElevVoltage())
            .current(getElevCurrent())
            .angularVelocity(getRate())
            .angularPosition(getElevatorPos());
    }

    //Command to set Elevator Voltage
    public Command getElevVoltCmd(Supplier<Voltage> voltage){
        return this.runEnd(
            () -> setVoltage(voltage.get()),
            () -> setVoltage(Volts.zero()));
    }

    /**
     * @param rate
     * @return Command for setting Elevator rate unless top/bottom limits
     */
    public Command getElevRateCmd(Supplier<AngularVelocity> rate){
        LimitDirection direction = getDirection(rate.get().in(RotationsPerSecond));

        return this.runEnd(
            () -> setRate(rate.get()),
            () -> setVoltage(Volts.zero())).unless(() -> getBotLimit(direction)).unless(() -> getTopLimit(direction)).finallyDo(() -> this.lastElevatorPos = getElevatorPos());
    }

    //Command for setting Elevator Position in rotations unless top/bottom limits
    public Command getElevatorPosCmd(Supplier<Angle> position){
        LimitDirection direction = getDirection(position.get().in(Rotations));

        return this.runEnd(
            () -> setElevatorPos(position.get()),
            () -> setVoltage(Volts.zero())).unless(() -> getBotLimit(direction)).unless(() -> getTopLimit(direction)).finallyDo(() -> this.lastElevatorPos = getElevatorPos());
    }

    //Command to hold Elevator Position
    public Command getHoldPosCmd(){
        return this.run(() -> setElevatorPos(this.lastElevatorPos));
    }

    //Command to return System ID
    public Command getSysIdCommandGroup(){
        return sysIdCommandGroup;
    }

    //Replaces value of Elastic Widget if null to string "null"
    public String getCommandString(){
        if (!(this.getCurrentCommand() == null)){
            return this.getCurrentCommand().getName();
        }else{
            return "null";
        }
    }

    //Sets Elastic Widgets
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

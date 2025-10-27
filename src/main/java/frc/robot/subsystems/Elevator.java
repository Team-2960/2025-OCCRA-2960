package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{

    private final SparkMax elevMotor;
    private final PIDController elevPID;
    private final RelativeEncoder relEncoder;

    public Elevator(int elevMotorID){
        elevMotor = new SparkMax(elevMotorID, MotorType.kBrushless);
        elevPID = new PIDController(0.5, 0, 0);
        relEncoder = elevMotor.getEncoder();
    }
    public void setVoltage(Voltage voltage){
        elevMotor.setVoltage(voltage);
    }

    public void setRate(AngularVelocity rate){
        double voltage = elevPID.calculate(getRate().in(RadiansPerSecond), rate.in(RadiansPerSecond));
        setVoltage(Volts.of(voltage));
    }

    public AngularVelocity getRate(){
        return AngularVelocity.ofBaseUnits(relEncoder.getVelocity()/60, RotationsPerSecond);
    }

    public Command getElevVoltCmd(Supplier<Voltage> voltage){
        return this.runEnd(
            () -> setVoltage(voltage.get()),
            () -> setVoltage(Volts.zero()));
    }


    public Command getElevRateCmd(Supplier<AngularVelocity> rate){
        return this.runEnd(
            () -> setRate(rate.get()),
            () -> setVoltage(Volts.zero()));
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
    }
}

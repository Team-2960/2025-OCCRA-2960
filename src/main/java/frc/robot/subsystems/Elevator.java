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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{

    private final SparkMax elevMotor;
    private final PIDController elevPID;
    private final RelativeEncoder relEncoder;

    public Elevator(int elevMotorID){
        elevMotor = new SparkMax(elevMotorID, MotorType.kBrushless);
        elevPID = new PIDController(0.001, 0, 0);
        relEncoder = elevMotor.getEncoder();
    }
    public void setVoltage(Voltage voltage){
        elevMotor.setVoltage(voltage);
    }

    public void setRate(AngularVelocity rate){
        double voltage = elevPID.calculate(getRate().in(RadiansPerSecond), rate.in(RadiansPerSecond));
        setVoltage(Volts.of(0));
    }

    public AngularVelocity getRate(){
        return AngularVelocity.ofBaseUnits(relEncoder.getVelocity()/60, RotationsPerSecond);
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
}

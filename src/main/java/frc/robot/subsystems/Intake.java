package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final SparkMax intakeMotor;

    public Intake(int intakeMotorID) {
        intakeMotor = new SparkMax(intakeMotorID, MotorType.kBrushless);
    }

    public void setVoltage(Voltage voltage) {
        intakeMotor.setVoltage(voltage);
    }

    public Command getIntakeCmd(Supplier<Voltage> voltage) {
        return this.runEnd(
                () -> setVoltage(voltage.get()),
                () -> setVoltage(Volts.zero()));
    }
}

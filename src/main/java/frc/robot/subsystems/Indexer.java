package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

    private final SparkMax indexMotor;

    public Indexer(int indexMotorID) {
        indexMotor = new SparkMax(indexMotorID, MotorType.kBrushless);
    }

    public void setVoltage(Voltage voltage) {
        indexMotor.setVoltage(voltage);
    }

    public Command getDriveCmd(Supplier<Voltage> voltage) {
        return this.runEnd(
                () -> setVoltage(voltage.get()),
                () -> setVoltage(Volts.zero()));
    }
}

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

    private final SparkMax topIndexMotor;
    private final SparkMax botIndexMotor;

    public Indexer(int topIndexMotorID, int botIndexMotorID) {
        topIndexMotor = new SparkMax(topIndexMotorID, MotorType.kBrushless);
        botIndexMotor = new SparkMax(botIndexMotorID, MotorType.kBrushless);        
    }

    public void setVoltage(Voltage voltage) {
        topIndexMotor.setVoltage(voltage);
        botIndexMotor.setVoltage(voltage.times(-1));
    }

    public void setVoltage(Voltage topVolt, Voltage botVolt){
        topIndexMotor.setVoltage(topVolt);
        botIndexMotor.setVoltage(botVolt);
    }

    public Command getIndexCmd(Supplier<Voltage> voltage) {
        return this.runEnd(
                () -> setVoltage(voltage.get()),
                () -> setVoltage(Volts.zero()));
    }

    public Command getIndIndexCmd(Supplier<Voltage> topVolt, Supplier<Voltage> botVolt) {
        return this.runEnd(
                () -> setVoltage(topVolt.get(), botVolt.get()),
                () -> setVoltage(Volts.zero()));
    }
}
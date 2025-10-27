package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final SparkMax lIntakeMotor;
    private final SparkMax rIntakeMotor;


    public Intake(int lIntakeMotorID, int rIntakeMotorID) {
        
        lIntakeMotor = new SparkMax(lIntakeMotorID, MotorType.kBrushless);
        rIntakeMotor = new SparkMax(rIntakeMotorID, MotorType.kBrushless);


    }

    public void setVoltage(Voltage voltage) {
        rIntakeMotor.setVoltage(voltage);
        lIntakeMotor.setVoltage(voltage.times(-1));
    }

    public void setVoltage(Voltage leftVolt, Voltage rightVolt){
        rIntakeMotor.setVoltage(rightVolt);
        lIntakeMotor.setVoltage(leftVolt);
    }

    public Command getIntakeCmd(Supplier<Voltage> voltage) {
        return this.runEnd(
                () -> setVoltage(voltage.get()),
                () -> setVoltage(Volts.zero()));
    }

    public Command getIndIntakeCmd(Supplier<Voltage> lVoltage, Supplier<Voltage> rVoltage) {
        return this.runEnd(
                () -> setVoltage(lVoltage.get(), rVoltage.get()),
                () -> setVoltage(Volts.zero()));
    }
}

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {

    private final SparkMax lEndEffectorMotor;
    private final SparkMax rEndEffectorMotor;


    public EndEffector(int lEndEffectorMotorID, int rEndEffectorMotorID) {
        
        lEndEffectorMotor = new SparkMax(lEndEffectorMotorID, MotorType.kBrushless);
        rEndEffectorMotor = new SparkMax(rEndEffectorMotorID, MotorType.kBrushless);
    }

    public void setVoltage(Voltage voltage) {
        rEndEffectorMotor.setVoltage(voltage);
        lEndEffectorMotor.setVoltage(voltage.times(-1));
    }

    public void setVoltage(Voltage leftVolt, Voltage rightVolt){
        rEndEffectorMotor.setVoltage(rightVolt);
        lEndEffectorMotor.setVoltage(leftVolt);
    }

    public Command getEndEffectorCmd(Supplier<Voltage> voltage) {
        return this.runEnd(
                () -> setVoltage(voltage.get()),
                () -> setVoltage(Volts.zero()));
    }

    public Command getIndEndEffectorCmd(Supplier<Voltage> lVoltage, Supplier<Voltage> rVoltage) {
        return this.runEnd(
                () -> setVoltage(lVoltage.get(), rVoltage.get()),
                () -> setVoltage(Volts.zero()));
    }
}

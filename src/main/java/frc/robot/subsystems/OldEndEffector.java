package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OldEndEffector extends SubsystemBase{

    private final SparkMax endEffectorMotor;

    private final SparkMaxConfig endEffectorConfig;

    public OldEndEffector(int endEffectorMotorID) {
        endEffectorMotor = new SparkMax(endEffectorMotorID, MotorType.kBrushless);

        endEffectorConfig = new SparkMaxConfig();
        //endEffectorConfig.inverted(true);

        endEffectorMotor.configure(endEffectorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setVoltage(Voltage voltage) {
        endEffectorMotor.setVoltage(voltage);
    }


    public Command getEndEffectorCmd(Supplier<Voltage> voltage) {
        return this.runEnd(
                () -> setVoltage(voltage.get()),
                () -> setVoltage(Volts.zero())
        );
    }

}

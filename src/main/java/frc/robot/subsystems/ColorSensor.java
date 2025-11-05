package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase{
    
    static final I2C.Port port = I2C.Port.kOnboard;

    final ColorSensorV3 colorSensor;

    final ColorMatch matcher;


    public ColorSensor(){
        colorSensor = new ColorSensorV3(port);
        matcher = new ColorMatch();

        matcher.setConfidenceThreshold(0.2);

        matcher.addColorMatch(Color.WHITE.color);
        matcher.addColorMatch(Color.YELLOW.color);
        matcher.addColorMatch(Color.BLACK.color);

    }

    public Color matchColor(){

        ColorMatchResult result = matcher.matchClosestColor(colorSensor.getColor());

        if (result.color == Color.WHITE.color) {
            return Color.WHITE;
        } else if (result.color == Color.YELLOW.color) {
            return Color.YELLOW;
        } else if (result.color == Color.BLACK.color) {
            return Color.BLACK;
        } else {
            return null;
        }
    }

    @Override
    public void periodic(){

        int red = (int) (colorSensor.getColor().red * 255);
        int green = (int) (colorSensor.getColor().green * 255);
        int blue = (int) (colorSensor.getColor().blue * 255);

        Color color = matchColor();

        SmartDashboard.putString("RGB Values", red + "," + green + "," + blue);
        SmartDashboard.putString("Color sensor values", color.toString());
    }

    enum Color {
        WHITE(new edu.wpi.first.wpilibj.util.Color(255, 255, 255)),
        YELLOW(new edu.wpi.first.wpilibj.util.Color(255, 255, 0)),
        BLACK(new edu.wpi.first.wpilibj.util.Color(0, 0, 0));

        edu.wpi.first.wpilibj.util.Color color;

        Color(edu.wpi.first.wpilibj.util.Color color) {
            this.color = color;
        }

        @Override
        public String toString() {
            if (this == WHITE) {
                return "white";
            } else if (this == YELLOW) {
                return "yellow";
            } else if (this == BLACK) {
                return "black";
            } else {
                return "";
            }
        }
    }
}

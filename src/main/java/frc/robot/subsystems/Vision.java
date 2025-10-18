package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    
    private final PhotonCamera camera;

    private double area;
    private Colors color;

    enum Colors{
        YELLOW,
        WHITE;
    }

    public Vision(String name){
        camera = new PhotonCamera(name);
        area = 0;
    }

    public void updateVision(){
        var results = camera.getAllUnreadResults();
        double lastArea = 0;
        ArrayList<Double> areaList = new ArrayList<Double>();
        Colors[] colors = Colors.values();

        for (int i = 0; i < Colors.values().length; i++){

            camera.setPipelineIndex(i);

            for (var change : results){
                var targets = change.getTargets();

                if (!targets.isEmpty()){
                    
                    for (var target : targets){
                        area = target.getArea();
                    }

                }else{
                    area = 0;
                }
            }

            areaList.add(area);
        }

        double maxValue = Collections.max(areaList);
        int idxArea = areaList.indexOf(maxValue);
        this.color = colors[idxArea];
        SmartDashboard.putNumber("max area", maxValue);

    }

    @Override
    public void periodic(){
        updateVision();
        SmartDashboard.putBoolean("Is Connected", camera.isConnected());
        SmartDashboard.putNumber("Target Area", area);
        SmartDashboard.putString("Current Color", color.toString());
    }

    

}

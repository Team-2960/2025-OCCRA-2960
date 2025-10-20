package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.TreeMap;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    
    private final PhotonCamera camera;

    private double area;
    private TreeMap<Integer, String> colorMap;
    private String curColor;

    public Vision(String name){
        camera = new PhotonCamera(name);
        area = 0;
        colorMap.put(0, "yellow");
        colorMap.put(1, "white");
    }

    public void updateVision(){
        var results = camera.getAllUnreadResults();
        double colorArea;
        ArrayList<Double> areaList = new ArrayList<Double>();

        for (int i = 0; i < colorMap.size(); i++){

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
        this.curColor = colorMap.get(idxArea);
        SmartDashboard.putNumber("max area", maxValue);
        SmartDashboard.putString("Value Array", areaList.toString());
    }

    @Override
    public void periodic(){
        updateVision();
        SmartDashboard.putBoolean("Is Connected", camera.isConnected());
        SmartDashboard.putNumber("Target Area", area);
        SmartDashboard.putString("Current Color", curColor.toString());
    }

    

}

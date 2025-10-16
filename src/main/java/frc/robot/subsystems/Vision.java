package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    
    private final PhotonCamera camera;

    private double area;

    public Vision(String name){
        camera = new PhotonCamera(name);
        camera.setPipelineIndex(0);
        area = 0;
    }

    public void updateVision(){
        var results = camera.getAllUnreadResults();

        for (var change : results){
            var target = change.getBestTarget();

            area = target.getArea();
        }

    }

    @Override
    public void periodic(){
        updateVision();
        SmartDashboard.putNumber("Target Area", area);
    }

    

}

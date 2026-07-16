package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Pattern;

public class LLSubsystem extends SubsystemBase {

    public Limelight3A limelight;

    public LLResult result;

    public final Alliance alliance;

    private int lastPipeline = 0;
    private int currentPipeline = 0;

    public LLSubsystem(HardwareMap hMap, Alliance alliance){
        limelight = hMap.get(Limelight3A.class, "limelight");
        this.alliance = alliance;

        if(limelight != null) {
            limelight.setPollRateHz(100);
            limelight.start();
        }

    }

    @Override
    public void periodic() {
        if (limelight == null) return;

        if (lastPipeline != currentPipeline) {
            limelight.pipelineSwitch(currentPipeline);
        }
        lastPipeline = currentPipeline;

        result = limelight.getLatestResult();


    }





    public Pattern getObelisk() {
        setObeliskPipeline();

        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            int id = result.getFiducialResults().get(0).getFiducialId();

            switch(id) {
                case 21:
                    return Pattern.GPP;
                case 22:
                    return Pattern.PGP;
                case 23:
                    return Pattern.PPG;
            }
        }

        return Pattern.GPP;
    }

    public void setObeliskPipeline() {
        currentPipeline = 1;
    }

    public  void setMegaTagPipeline(){
        currentPipeline = 0;
    }



}

package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Pattern;

public class LLSubsystem extends SubsystemBase {

    public Limelight3A limelight;

    public LLResult result;

    public final Alliance alliance;



    private int currentPipeline = 0;

    public LLSubsystem(HardwareMap hMap, Alliance alliance){
        limelight = hMap.get(Limelight3A.class, "limelight");
        this.alliance = alliance;

        limelight.setPollRateHz(100);
        limelight.start();
    }

    @Override
    public void periodic(){
        limelight.pipelineSwitch(currentPipeline);

        result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            if(currentPipeline == 0) {
                FtcDashboard.getInstance().getTelemetry().addData("Limelight Alliance tA", getAllianceTA());
                FtcDashboard.getInstance().getTelemetry().addData("Limelight Alliance tX", getAllianceTX());
            } else if(currentPipeline == 1) {
                FtcDashboard.getInstance().getTelemetry().addData("Obelisk Pattern", getObelisk());
            }
        } else {
            FtcDashboard.getInstance().getTelemetry().addData("Limelight", "No Targets");
        }

        FtcDashboard.getInstance().getTelemetry().addData("pipe",currentPipeline);
    }

    public Double getAllianceTA() {
        setAimingPipeline();

        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            int id = result.getFiducialResults().get(0).getFiducialId();

            if (alliance == Alliance.ANY ||
                    (alliance == Alliance.RED && id == 24) ||
                    (alliance == Alliance.BLUE && id == 20)) {
                return result.getTa();
            }
        }

        return null;
    }


    public Double getAllianceTX() {
        setAimingPipeline();

        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            int id = result.getFiducialResults().get(0).getFiducialId();

            if (alliance == Alliance.ANY ||
                    (alliance == Alliance.RED && id == 24) ||
                    (alliance == Alliance.BLUE && id == 20)) {
                return result.getTx();
            }
        }

        return null;
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

        return Pattern.UNKNOWN;
    }

    public void setObeliskPipeline() {
        currentPipeline = 1;
    }

    public void setAimingPipeline() {
        currentPipeline = 0;
    }

}

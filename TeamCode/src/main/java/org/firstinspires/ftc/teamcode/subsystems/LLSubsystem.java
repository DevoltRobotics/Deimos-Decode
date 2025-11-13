package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Alliance;

public class LLSubsystem extends SubsystemBase {

    public Limelight3A limelight;

    public LLResult result;

    public final Alliance alliance;

    public LLSubsystem(HardwareMap hMap, Alliance alliance){
        limelight = hMap.get(Limelight3A.class, "limelight");
        this.alliance = alliance;

        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0); // apriltags
    }

    @Override
    public void periodic(){
        result = limelight.getLatestResult();


        if (result != null && result.isValid()) {
            FtcDashboard.getInstance().getTelemetry().addData("LL AprilTag tA", result.getTa());
            FtcDashboard.getInstance().getTelemetry().addData("LL AprilTag tX", result.getTy());
        } else {
            FtcDashboard.getInstance().getTelemetry().addData("Limelight", "No Targets");
        }
    }

    public Double getAllianceTA() {
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


}

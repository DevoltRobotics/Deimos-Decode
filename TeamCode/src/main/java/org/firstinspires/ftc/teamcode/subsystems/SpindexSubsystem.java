package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;


@Config
public class SpindexSubsystem extends SubsystemBase {

    public com.qualcomm.robotcore.hardware.CRServo spindex;

    public NormalizedColorSensor colorSensorL;
    public NormalizedColorSensor colorSensorR;
    private DetectedColor currentColorL = DetectedColor.UNKNOWN;


    double offset = 153.27;

    double targetPos = offset;

    public double FPos = offset;

    public double SPos = FPos + 120;

   public double TPos = (SPos + 120)-360;

    public enum DetectedColor{
        PURPLE,
        GREEN,
        UNKNOWN
    }


    public AnalogInput SpindexPos;

    public PIDFController SpindexPID;
    public static PIDFCoefficients SPcoeffs = new PIDFCoefficients(0.0055,0,0,0);

    float normRed,normGreen,normBlue;
    NormalizedRGBA colorsL;

    public double Spindexp;

    public SpindexSubsystem (HardwareMap hMap){

        spindex = hMap.get(CRServo.class,"spindex");

        SpindexPos = hMap.get(AnalogInput.class,"Spencoder");

        colorSensorL = hMap.get(NormalizedColorSensor.class,"colorL");
        colorSensorR = hMap.get(NormalizedColorSensor.class,"colorR");

        colorSensorL.setGain(8);

        SpindexPID = new PIDFController(SPcoeffs);

    }

    @Override
    public void periodic(){
        SpindexPID.setCoefficients(SPcoeffs);
        Spindexp = SpindexPos.getVoltage() / 3.3 * 360;

        SpindexPID.setSetPoint(targetPos);
        double power = SpindexPID.calculate(Spindexp);

        if (SpindexPID.getPositionError() > 180){
        spindex.setPower(power);
        } else if (SpindexPID.getPositionError() < 180) {
            spindex.setPower(-power);
        }else {
            spindex.setPower(power);
        }

        colorsL = colorSensorL.getNormalizedColors();
         normRed = colorsL.red/ colorsL.alpha;
        normGreen = colorsL.green/ colorsL.alpha;
        normBlue = colorsL.blue/ colorsL.alpha;

        FtcDashboard.getInstance().getTelemetry().addData("Red",normRed);
        FtcDashboard.getInstance().getTelemetry().addData("Green",normGreen);
        FtcDashboard.getInstance().getTelemetry().addData("Blue",normBlue);
        FtcDashboard.getInstance().getTelemetry().addData("target",targetPos);
        FtcDashboard.getInstance().getTelemetry().addData("Spindex_pos",Spindexp);


        if (normRed > 0.35 && normBlue > 0.35 && normGreen < 0.35){
            currentColorL = DetectedColor.PURPLE;
        } else if (normGreen > 0.35 && normBlue < 0.35 && normRed < 0.35) {
            currentColorL = DetectedColor.GREEN;
        }else {
            currentColorL = DetectedColor.UNKNOWN;
        }


    }

    public void setTargetPos(double targetPos){
        this.targetPos = targetPos;
    }




}

package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class TurretSubsystem extends SubsystemBase {

    CRServo Torreta;
    CRServo Torreta2;

    public static PIDFCoefficients encPidfCoeffs = new PIDFCoefficients(0.025, 0, 0, 0.05);
    public static PIDFCoefficients llPidfCoeffs = new PIDFCoefficients(0.025,0,0.001,0.05);

    public static double Minimum = 0.043;

    AnalogInput encoder;
    Double lastPos = null;

    double currentRelativePos;

    public TurretSubsystem(HardwareMap hMap) {
        Torreta = hMap.get(CRServo.class, "torreta");
        Torreta2 = hMap.get(CRServo.class,"torreta2");
        encoder = hMap.get(AnalogInput.class, "torretaenc");
    }

    public void setTurretPower(double power) {
        Torreta.setPower(power);
        Torreta2.setPower(power);
    }

    @Override
    public void periodic() {
        double absolutePos = (encoder.getVoltage() / 3.3) * 360;
        if(lastPos == null) {
            lastPos = absolutePos;
        }

        double deltaPos = AngleUnit.normalizeDegrees(absolutePos - lastPos);

        currentRelativePos += deltaPos;
        lastPos = absolutePos;

        FtcDashboard.getInstance().getTelemetry().addData("turret power", Torreta.getPower());
        FtcDashboard.getInstance().getTelemetry().addData("turret relative pos", currentRelativePos);
    }

    public double getCurrentPosition() {
        return currentRelativePos;
    }

}

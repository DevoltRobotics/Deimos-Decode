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

    public static double limit = 1300;
    public static PIDFCoefficients encPidfCoeffs = new PIDFCoefficients(0.002, 0, 0.0001, 0.0008);
    public static PIDFCoefficients llPidfCoeffs = new PIDFCoefficients(0.022,0,0.00055,0);

    public static double Minimum = 0.037;
    public static double MinimumEnc = 0.04;

    AnalogInput encoder;
    static Double lastPos = null;

    static double currentRelativePos;

    public TurretSubsystem(HardwareMap hMap) {
        Torreta = hMap.get(CRServo.class, "torreta");
        Torreta2 = hMap.get(CRServo.class,"torreta2");
        encoder = hMap.get(AnalogInput.class, "torretaenc");
    }

    public void setTurretPower(double power) {
        if((currentRelativePos < 20 && power > 0) || (currentRelativePos > limit && power < 0)) {
            Torreta.setPower(0);
            Torreta2.setPower(0);
        } else {
            Torreta.setPower(power);
            Torreta2.setPower(power);
        }
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

    public void resetRelative() {
        currentRelativePos = 0;
    }

}

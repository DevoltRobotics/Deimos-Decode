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

@Config
public class TurretSubsystem extends SubsystemBase {

    CRServo Torreta;
    CRServo Torreta2;

    public static PIDFCoefficients llPidfCoeffs = new PIDFCoefficients(0.035, 0, 0.0015, 0);
    public static double Minimum = 0.043;


    public TurretSubsystem(HardwareMap hMap) {
        Torreta = hMap.get(CRServo.class, "torreta");
        Torreta2 = hMap.get(CRServo.class,"torreta2");
    }

    public void setTurretPower(double power) {
        Torreta.setPower(power);
        Torreta2.setPower(power);
    }

    @Override
    public void periodic() {
        FtcDashboard.getInstance().getTelemetry().addData("turret power", Torreta.getPower());
    }

}

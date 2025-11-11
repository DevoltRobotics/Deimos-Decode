package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

@Config
@Configurable
public class ShooterSubsystem extends SubsystemBase {


    public DcMotorEx shooter;
    public DcMotorEx shooter2;

    public PIDFController shooterPIDF;

    public static PIDFCoefficients SCoeffs = new PIDFCoefficients(0.005,0,0,0);
    public static double kV = 0.000505;

    private double TargetVelocity = 0;

    public ShooterSubsystem(HardwareMap hMap){
        shooter = hMap.get(DcMotorEx.class,"shooter");
        shooter2 = hMap.get(DcMotorEx.class,"shooter2");

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterPIDF = new PIDFController(SCoeffs);
    }

    @Override
    public void periodic(){
        shooterPIDF.setCoefficients(SCoeffs);

        double CurrentVelocity = -(shooter2.getVelocity());

        shooterPIDF.setSetPoint(TargetVelocity);
        double power = (kV *TargetVelocity)+shooterPIDF.calculate(CurrentVelocity);

        shooter.setPower(power);
        shooter2.setPower(power);

        FtcDashboard.getInstance().getTelemetry().addData("shooter target",TargetVelocity);
        FtcDashboard.getInstance().getTelemetry().addData("shooter current",CurrentVelocity);
    }

    public void setTargetVelocity(double target){
        TargetVelocity = target;
    }

    public double getTargetVelocity(){
        return TargetVelocity;
    }
}

package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.config.LoggedSubsystem;

@Config
@Configurable
public class ShooterSubsystem extends LoggedSubsystem {


    public DcMotorEx shooter;
    public DcMotorEx shooter2;

    public PIDFController shooterPIDF;

    public static PIDFCoefficients SCoeffs = new PIDFCoefficients(0.0047,0,0,0);
    public static double kV = 0.00066;

    private double TargetVelocity = 0;

    public Servo hood;

    public ShooterSubsystem(HardwareMap hMap){
        shooter = hMap.get(DcMotorEx.class,"shooter");
        shooter2 = hMap.get(DcMotorEx.class,"shooter2");

        hood = hMap.get(Servo.class,"hood");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        shooterPIDF = new PIDFController(SCoeffs);
    }

    @Override
    public void periodic(){
        shooterPIDF.setCoefficients(SCoeffs);

        double CurrentVelocity = getCurrentVelocity();

        shooterPIDF.setSetPoint(TargetVelocity);
        double power = (kV *TargetVelocity)+shooterPIDF.calculate(CurrentVelocity);

        shooter.setPower(power);
        shooter2.setPower(power);

      /*  FtcDashboard.getInstance().getTelemetry().addData("shooter target",TargetVelocity);
        FtcDashboard.getInstance().getTelemetry().addData("shooter current",CurrentVelocity);*/
    }

    public double getCurrentVelocity() {
        return (-shooter2.getVelocity());
    }

    public void setTargetVelocity(double target){
        TargetVelocity = target;
    }

    public double getTargetVelocity(){
        return TargetVelocity;
    }

    public void setHood(double ServoPos){
        hood.setPosition(ServoPos);
    }

    @Override
    public void log(TelemetryPacket packet) {
        packet.put("Shooter/Target Vel",TargetVelocity);
        packet.put("Shooter/Current Vel",getCurrentVelocity());
    }
}

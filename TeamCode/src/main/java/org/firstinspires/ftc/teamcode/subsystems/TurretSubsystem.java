package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Alliance;

@Config
public class TurretSubsystem extends SubsystemBase {

    CRServo Torreta;
    CRServo Torreta2;

    public static double lowerLimit = -120;
    public static double upperLimit = 180;
    public static PIDFCoefficients encPidfCoeffs = new PIDFCoefficients(0.005, 0, 0.0002, 0.000525);
    public static PIDFCoefficients llPidfCoeffs = new PIDFCoefficients(0.022, 0, 0.00055, 0);

    public static double Minimum = 0.037;
    public static double MinimumEnc = 0.06;

    double goalX, goalY;

    DcMotor encoder;

    public static double currentRelativePos;

    PedroSubsystem pedroSubsystem;

    double robotToGoalAngle;
    double turretToGoalAngle;

    Alliance alliance;

    public void setGoalPos(double x, double y) {
        goalX = x;
        goalY = y;
    }

    public TurretSubsystem(HardwareMap hMap, PedroSubsystem pedroSubsystem, Alliance alliance) {
        Torreta = hMap.get(CRServo.class, "torreta");
        Torreta2 = hMap.get(CRServo.class, "torreta2");
        encoder = hMap.get(DcMotor.class, "intake");
        this.pedroSubsystem = pedroSubsystem;
        this.alliance = alliance;
        if (alliance == Alliance.RED) {
            setGoalPos(140.69, 139.15);
        } else if (alliance == Alliance.BLUE) {
            setGoalPos(3.52, 139.15);
        } else {
            setGoalPos(140.69, 139.15);
        }
    }

    public void setTurretPower(double power) {
        if ((currentRelativePos < lowerLimit && power > 0) || (currentRelativePos > upperLimit && power < 0)) {
            Torreta.setPower(0);
            Torreta2.setPower(0);
        } else {
            Torreta.setPower(power);
            Torreta2.setPower(power);
        }
    }

    @Override
    public void periodic() {
        Pose robotPos = pedroSubsystem.follower.getPose();

        double dx = goalX - robotPos.getX();
        double dy = goalY - robotPos.getY();

        robotToGoalAngle = Math.toDegrees(Math.atan2(dy, dx));
        turretToGoalAngle = AngleUnit.normalizeDegrees(Math.toDegrees(robotPos.getHeading()) - robotToGoalAngle);

        double encoderDegrees = (encoder.getCurrentPosition() * 360) / 8192.0; //8192 ticks per rev
        // zero is with the turret pointing to the front of the bot, left is negative front right is positive
        currentRelativePos = AngleUnit.normalizeDegrees(encoderDegrees / (109 / 24.0)); // gear ratio

        FtcDashboard.getInstance().getTelemetry().addData("turret power", Torreta.getPower());
        FtcDashboard.getInstance().getTelemetry().addData("turret angle", currentRelativePos);
        FtcDashboard.getInstance().getTelemetry().addData("turret to goal angle", getTurretToGoalAngle());
    }

    public double getRobotToGoalAngle() {
        return robotToGoalAngle;
    }

    public double getTurretToGoalAngle() {
        return turretToGoalAngle;
    }


    public double getCurrentPosition() {
        return currentRelativePos;
    }

    public void resetEncoder() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}

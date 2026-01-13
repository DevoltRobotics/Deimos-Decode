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
import org.firstinspires.ftc.teamcode.config.AngleKalman1D;

@Config
public class TurretSubsystem extends SubsystemBase {

    CRServo Torreta;
    CRServo Torreta2;

    public static double lowerLimit = -115;
    public static double upperLimit = 215;

    public static double LL_OFFSET = 3;

    public static PIDFCoefficients encPidfCoeffs = new PIDFCoefficients(0.005, 0, 0.0002, 0.000525);
    public static PIDFCoefficients llPidfCoeffs = new PIDFCoefficients(0.022, 0, 0.00055, 0);

    public static double Minimum = 0.037;
    public static double MinimumEnc = 0.06;

    double goalX, goalY;

    DcMotor encoder;

    public static double currentRelativePos;

    private AngleKalman1D fuse;
    private double fusedTurretGoalDeg = 0;

    public static double kQ = 0.8;   // tune
    public static double kRGood = 4; // tune
    public static double kRBad  = 25;// tune

    PedroSubsystem pedroSubsystem;

    double robotToGoalAngle;
    double turretToGoalAngle;

     public double distanceToGoal = 0;

     public double heading;

     public double offsetY;
     public double offsetX;

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
            setGoalPos(155, 143);
        } else if (alliance == Alliance.BLUE) {
            setGoalPos(0, 143);
        } else {
            setGoalPos(155, 143);
        }
        fuse = new AngleKalman1D(
                0.0,     // x0: estimación inicial
                9.0,     // P0: incertidumbre inicial
                kQ,      // Q
                kRGood   // R inicial
        );
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


        heading = robotPos.getHeading();

        offsetX = (0 * Math.cos(heading)) + (-1.22*Math.sin(heading));
        offsetY = (-0*Math.sin(heading)) + (-1.22 * Math.cos(heading));

        double dx = goalX - (robotPos.getX()+ offsetX);
        double dy = goalY - (robotPos.getY()+ offsetY);

        robotToGoalAngle = Math.toDegrees(Math.atan2(dy, dx));//direction
        distanceToGoal = Math.hypot(dx,dy);//magnitude

        turretToGoalAngle =AngleUnit.normalizeDegrees(Math.toDegrees(robotPos.getHeading())) - robotToGoalAngle;

        fuse.setQ(kQ);
        fuse.predictFromOdo(turretToGoalAngle);

        fusedTurretGoalDeg = fuse.get();

        double encoderDegrees = (encoder.getCurrentPosition() * 360) / 8192.0; //8192 ticks per rev
        // zero is with the turret pointing to the front of the bot, left is negative front right is positive
        currentRelativePos =encoderDegrees / (109 / 24.0); // gear ratio

        FtcDashboard.getInstance().getTelemetry().addData("turret power", Torreta.getPower());
        FtcDashboard.getInstance().getTelemetry().addData("turret angle", currentRelativePos);
        FtcDashboard.getInstance().getTelemetry().addData("turret to goal angle", getTurretToGoalAngle());
        FtcDashboard.getInstance().getTelemetry().addData("distance to goal", getDistanceToGoal() );
        FtcDashboard.getInstance().getTelemetry().addData("fused angle", getFusedTurretGoalAngle());
    }

    /** Call this from your LL command only when LL is valid. */
    public void pushLimelightMeasurement(double txOffsetDeg) {

        // Adaptive R: when target is big/close -> trust LL more
        fuse.setR(kRBad);

        // Paralaje por cámara a la derecha:
        double parallaxDeg = Math.toDegrees(Math.atan2(LL_OFFSET, distanceToGoal));

        // Convert tx error to absolute turret setpoint measurement
        double llDesired = AngleKalman1D.normalizeDeg(currentRelativePos - txOffsetDeg - parallaxDeg); // flip sign if needed
        FtcDashboard.getInstance().getTelemetry().addData("ll desired", llDesired);
        FtcDashboard.getInstance().getTelemetry().addData("ll parallax", parallaxDeg);

        fuse.updateWithLL(llDesired);
    }

    public double getFusedTurretGoalAngle() {
        return fusedTurretGoalDeg;
    }

    public double getRobotToGoalAngle() {
        return robotToGoalAngle;
    }

    public double getTurretToGoalAngle() {
        return turretToGoalAngle;
    }

    public double getDistanceToGoal(){
        return distanceToGoal;
    }


    public double getCurrentPosition() {
        return currentRelativePos;
    }

    public void resetEncoder() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}

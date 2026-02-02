package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.config.AngleKalman1D;

import java.awt.font.NumericShaper;

@Config
public class TurretSubsystem extends SubsystemBase {

    Servo Torreta;


    public static double lowerLimit = -110;
    public static double upperLimit = 207.0642;



    double goalX, goalY;


    public static double currentRelativePos;


    PedroSubsystem pedroSubsystem;

    double robotToGoalAngle;
    double turretToGoalAngle;

     public double distanceToGoal = 0;

     public double heading;

     public double offsetY;
     public double offsetX;

     public static double kF = 0.0244;

     public static double LeftkF = 0.019;

    Alliance alliance;

    public void setGoalPos(double x, double y) {
        goalX = x;
        goalY = y;
    }

    public TurretSubsystem(HardwareMap hMap, PedroSubsystem pedroSubsystem, Alliance alliance) {
        Torreta = hMap.get(Servo.class, "torreta");
        Torreta.setDirection(Servo.Direction.REVERSE);

        this.pedroSubsystem = pedroSubsystem;
        this.alliance = alliance;
        if (alliance == Alliance.RED) {
            setGoalPos(130, 130);
        } else if (alliance == Alliance.BLUE) {
            setGoalPos(13, 130);
        } else {
            setGoalPos(130, 130);
        }

    }



    @Override
    public void periodic() {
        Pose robotPos = pedroSubsystem.follower.getPose();


        heading = robotPos.getHeading();

        offsetX = (0 * Math.cos(heading)) + (-1.22*Math.sin(heading));
        offsetY = (-0*Math.sin(heading)) + (-1.22 * Math.cos(heading));

        double dx = goalX - (robotPos.getX() + offsetX); //offsetx
        double dy = goalY - (robotPos.getY() + offsetY); //offsety

        robotToGoalAngle = Math.toDegrees(Math.atan2(dy, dx));//direction
        distanceToGoal = Math.hypot(dx,dy);//magnitude

        turretToGoalAngle =AngleUnit.normalizeDegrees(Math.toDegrees(robotPos.getHeading()) - robotToGoalAngle );

        FtcDashboard.getInstance().getTelemetry().addData("turret to goal angle", getTurretToGoalAngle());
        FtcDashboard.getInstance().getTelemetry().addData("distance to goal", getDistanceToGoal() );
        FtcDashboard.getInstance().getTelemetry().addData("Servo pos", Torreta.getPosition() );

    }





    public void TurretSetPos(double PosDeg){
        double servoPos = (PosDeg - lowerLimit) / (upperLimit - lowerLimit);
        if (PosDeg > 0) {
            servoPos += kF;
        }else if (PosDeg < 0) {
            servoPos -= LeftkF;
        }
        Torreta.setPosition(Range.clip(servoPos,0,1));

    }

    public double getTurretToGoalAngle() {
        return turretToGoalAngle;
    }

    public double getDistanceToGoal(){
        return distanceToGoal;
    }





}

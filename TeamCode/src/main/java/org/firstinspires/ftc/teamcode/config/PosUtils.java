package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PosUtils {

    public static Pose PedrotoFTC(Pose PedroPos){
        double x = -PedroPos.getY() +72; //inverted for FTC coords
        double y = PedroPos.getX() -72; //inverted for FTC coords
        double heading = AngleUnit.normalizeRadians(PedroPos.getHeading() - Math.toRadians(-90));

        return new Pose(x,y,heading);
    }
}

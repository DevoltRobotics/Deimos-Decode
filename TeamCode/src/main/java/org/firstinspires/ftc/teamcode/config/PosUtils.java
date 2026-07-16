package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class PosUtils {

    public static Pose PedrotoFTC(Pose PedroPos){
        double x = -PedroPos.getY() +72; //inverted for FTC coords
        double y = PedroPos.getX() -72; //inverted for FTC coords
        double heading = AngleUnit.normalizeRadians(PedroPos.getHeading() - Math.toRadians(-90));

        return new Pose(x,y,heading);
    }

    public static Pose from3DToPedro(Pose3D pose3d, double headingRad) {
        double xFtc = pose3d.getPosition().x * 39.37;
        double yFtc = pose3d.getPosition().y * 39.37;
        // 1) shift origin: center → corner
        double xShifted = 72 - xFtc;
        double yShifted = yFtc + 72.0;

        // 2) axis swap
        double xPedro = yShifted;
        double yPedro = xShifted;

        // 3) heading conversion
        double headingPedro = -headingRad;

        return new Pose(
                xPedro,
                yPedro,
                headingPedro
        );
    }
}

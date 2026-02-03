package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Pattern;

@Config
public class SpindexSubsystem extends SubsystemBase {

    public static Pattern obeliskPattern = Pattern.PGP;

    NormalizedColorSensor colorSensor;
    NormalizedRGBA colors;

    float normRed, normBlue, normGreen;

    DigitalChannel laser;

    public enum DetectedColor {
        Purple,
        Green,
        Unknown
    }


    public DetectedColor detectedColor = DetectedColor.Unknown;
    public double ticks_per_rev = 8192;
    public boolean isShooting = false;
    public static double IntakePos = 60;

    public static double ShootPos = 0;

    public double GrenBallPos = 0;


    public static double tolerance = 1;


    DcMotor Spindex;

    public PIDFController SpinPID;
    public static double kS = 0.032;

    public static PIDFCoefficients Spincoeffs = new PIDFCoefficients(0.012, 0, 0.0008, 0);

    public static double TRIGGER_COOLDOWN_MS = 120;             // anti rebote tiempo
    private double targetPos;
    private boolean Shootmode = false;     // TRUE = modo manual/tiro; FALSE = modo indexar auto
    public static int nBalls = 0; // CONTAODR INTERNO

    public boolean FirstInitIn = true;

    public boolean FirstInitSho = true;

    double power = 0;

    double distCm = 10;

    static double spindexEncoderOffset = 0;
    double spindexPosDeg = 0;

    public static double minO = 0.087;

    double ff;

    double Normtarget = 0;

    double DeltaAngleRad = 0;

    public double DeltaAngleDeg = 0;

    double Normpos = 0;

    public SpindexSubsystem(HardwareMap hMap) {

        laser = hMap.get(DigitalChannel.class, "laser");
        laser.setMode(DigitalChannel.Mode.INPUT);

        colorSensor = hMap.get(NormalizedColorSensor.class, "color");
        colorSensor.setGain(8);

        Spindex = hMap.get(DcMotor.class, "spindex");
        Spindex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Spindex.setDirection(DcMotorSimple.Direction.REVERSE);


        setTargetPos(IntakePos);
        SpinPID = new PIDFController(Spincoeffs);
        SpinPID.setTolerance(2);
        SpinPID.setMinimumOutput(minO);
    }

    @Override
    public void periodic() {
        ff = Math.signum(SpinPID.getPositionError()) * kS;


        SpinPID.setMinimumOutput(minO);
        SpinPID.setCoefficients(Spincoeffs);


        if (nBalls == 3 && !isShooting) {
            Shootmode = true;
        } else if (nBalls == 0 && !isShooting) {
            Shootmode = false;
        }

        nBalls = Range.clip(nBalls, 0, 3);

        Normtarget = AngleUnit.normalizeDegrees(targetPos);

        spindexPosDeg = ((Spindex.getCurrentPosition() - spindexEncoderOffset) / ticks_per_rev) * 360;

        Normpos = AngleUnit.normalizeDegrees(spindexPosDeg);

        DeltaAngleRad = MathFunctions.getTurnDirection(Math.toRadians(Normtarget), Math.toRadians(Normpos)) * MathFunctions.getSmallestAngleDifference(Math.toRadians(Normtarget),Math.toRadians(Normpos));

        DeltaAngleDeg = Math.toDegrees(DeltaAngleRad);

        SpinPID.setSetPoint(0);

        power = Math.min(SpinPID.calculate(DeltaAngleDeg), 1);


        if (Math.abs(SpinPID.getPositionError()) < tolerance) {
            Spindex.setPower(0);
        } else {
            Spindex.setPower(power + ff);
        }


        detectedColor = getDetectedColor();
        distCm = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);



        FtcDashboard.getInstance().getTelemetry().addData("shootMode", Shootmode);
        FtcDashboard.getInstance().getTelemetry().addData("spindex target pos", targetPos);
        FtcDashboard.getInstance().getTelemetry().addData("spindexposDeg", spindexPosDeg);
        FtcDashboard.getInstance().getTelemetry().addData("distanciaB", distCm);
        FtcDashboard.getInstance().getTelemetry().addData("nBalls", nBalls);
        FtcDashboard.getInstance().getTelemetry().addData("spinError", SpinPID.getPositionError());
        FtcDashboard.getInstance().getTelemetry().addData("spindexPower", Spindex.getPower());
        FtcDashboard.getInstance().getTelemetry().addData("detected color", detectedColor);
        FtcDashboard.getInstance().getTelemetry().addData("Patron", obeliskPattern);
        FtcDashboard.getInstance().getTelemetry().addData("GreenBPos", GrenBallPos);
        FtcDashboard.getInstance().getTelemetry().addData("red", normRed);
        FtcDashboard.getInstance().getTelemetry().addData("blue", normBlue);
        FtcDashboard.getInstance().getTelemetry().addData("green", normGreen );
        FtcDashboard.getInstance().getTelemetry().addData("Extra ball", laser.getState());






    }

    public DetectedColor getDetectedColor() {
        colors = colorSensor.getNormalizedColors();

        normRed = colors.red / colors.alpha;
        normBlue = colors.blue / colors.alpha;
        normGreen = colors.green / colors.alpha;


        if (normRed < 0.075 && normBlue > 0.06 && normGreen > 0.095) {
            return DetectedColor.Green;
        }else {
            return DetectedColor.Unknown;
        }

    }

    public DetectedColor sampleColorBestAlpha(int samples) {
        double bestAlpha = 0;
        double r = 0, g = 0, b = 0;

        for (int i = 0; i < samples; i++) {
            NormalizedRGBA c = colorSensor.getNormalizedColors();

            if (c.alpha > bestAlpha) {
                bestAlpha = c.alpha;
                r = c.red;
                g = c.green;
                b = c.blue;
            }
        }

        double nr = r / bestAlpha;
        double ng = g / bestAlpha;
        double nb = b / bestAlpha;

        FtcDashboard.getInstance().getTelemetry().addData("nr", nr);
        FtcDashboard.getInstance().getTelemetry().addData("ng",ng);
        FtcDashboard.getInstance().getTelemetry().addData("nb", nb);




        if (nr < 0.13 && nb > 0.12 && ng > 0.2) {
            return DetectedColor.Green;
        }else {
            return DetectedColor.Unknown;
        }


    }


    public boolean getExtraB() {
        return laser.getState();
    }


    public void setnBalls(int nBalls) {
        SpindexSubsystem.nBalls = nBalls;
    }

    public boolean getFirstInitIn() {
        return FirstInitIn;
    }

    public void setFirstInitIn(boolean FirstInitIn) {
        this.FirstInitIn = FirstInitIn;
    }

    public boolean getFirstInitSho() {
        return FirstInitSho;
    }

    public void setFirstInitsho(boolean FirstInitSho) {
        this.FirstInitSho = FirstInitSho;
    }

    public void setShootmode(boolean Shootmode) {
        this.Shootmode = Shootmode;
    }


    public boolean getShootmode() {
        return Shootmode;
    }

    public void advanceOneIndex() {
        targetPos -= 120;
    }

    public void returnOneIndex() {
        targetPos += 120;
    }

    public int getnBalls() {
        return nBalls;
    }

    public void addnBalls() {
        nBalls++;
    }

    public void lessBalls() {
        nBalls--;
    }

    public boolean getBPresence() {
        distCm = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        return (distCm < 5);
    }

    public void setTargetPos(double targetPos) {
        this.targetPos = targetPos;
    }

    public double getTargetPos() {
        return targetPos;
    }

    public void setShooting(boolean isShooting) {
        this.isShooting = isShooting;
    }

    public double getPatternOffset() {
        if (obeliskPattern == Pattern.PGP) {
            return GrenBallPos + 120;
        } else if (obeliskPattern == Pattern.PPG) {
            return GrenBallPos - 120;
        } else if (obeliskPattern == Pattern.GPP) {
            return GrenBallPos;
        } else {
            if (nBalls == 3) {
                return targetPos;
            } else {
                return (targetPos + 60);
            }
        }
    }

    public double getPatternOffsetShootcMode() {
        if (obeliskPattern == Pattern.PGP) {
            return GrenBallPos + 120;
        } else if (obeliskPattern == Pattern.PPG) {
            return GrenBallPos - 120;
        } else if (obeliskPattern == Pattern.GPP) {
            return GrenBallPos;
        } else {
            return (targetPos + 60);
        }
    }

    public void SARSP() {
        spindexEncoderOffset = Spindex.getCurrentPosition();
    }
}
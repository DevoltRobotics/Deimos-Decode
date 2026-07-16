package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Pattern;
import org.firstinspires.ftc.teamcode.config.BallColorSensorPipeline;
import org.firstinspires.ftc.teamcode.config.LoggedSubsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
public class SpindexSubsystem extends LoggedSubsystem {

    public static Pattern obeliskPattern = Pattern.PGP;

    RevColorSensorV3 colorSensor;

    OpenCvWebcam colorSensorWebcam;
    BallColorSensorPipeline colorSensorPipeline = new BallColorSensorPipeline();

    DigitalChannel laser;

    public double error = 0;

    public enum DetectedColor {
        Purple,
        Green,
        Unknown
    }

    public DetectedColor detectedColor = DetectedColor.Unknown;
    public double h, s, v = 0;

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

    public static double TRIGGER_COOLDOWN_MS = 200;             // anti rebote tiempo
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


    public SpindexSubsystem(HardwareMap hMap) {
        laser = hMap.get(DigitalChannel.class, "laser");
        laser.setMode(DigitalChannel.Mode.INPUT);

        colorSensor = hMap.get(RevColorSensorV3.class, "color");

        Spindex = hMap.get(DcMotor.class, "spindex");
        Spindex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Spindex.setDirection(DcMotorSimple.Direction.REVERSE);

        colorSensorWebcam = OpenCvCameraFactory.getInstance().createWebcam(
                hMap.get(WebcamName.class, "Webcam 1")
        );

        colorSensorWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                colorSensorWebcam.setPipeline(colorSensorPipeline);
                colorSensorWebcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG);

                FtcDashboard.getInstance().startCameraStream(colorSensorWebcam, 30);
            }

            @Override
            public void onError(int errorCode) {}
        });

        setTargetPos(IntakePos);
        SpinPID = new PIDFController(Spincoeffs);
        SpinPID.setTolerance(2);
        SpinPID.setMinOutput(minO);
    }

    @Override
    public void periodic() {
        h = colorSensorPipeline.getExportedData(0);
        s = colorSensorPipeline.getExportedData(1);
        v = colorSensorPipeline.getExportedData(2);

        ff = Math.signum(SpinPID.getPositionError()) * kS;

        SpinPID.setMinOutput(minO);
        SpinPID.setCoefficients(Spincoeffs);



        if (nBalls == 3 && !isShooting) {
            Shootmode = true;
        } else if (nBalls == 0 && !isShooting) {
            Shootmode = false;
        }


        nBalls = Range.clip(nBalls, 0, 3);



        spindexPosDeg = ((Spindex.getCurrentPosition() - spindexEncoderOffset) / ticks_per_rev) * 360;


        error = targetPos - spindexPosDeg;

        SpinPID.setSetPoint(0);

        power = Range.clip(SpinPID.calculate(error) + Math.signum(error) * kS, -1, 1);

        if (Math.abs(error) < tolerance) {
            Spindex.setPower(0);
        } else {
            Spindex.setPower(-power);
        }

      /*  FtcDashboard.getInstance().getTelemetry().addData("shootMode", Shootmode);
        FtcDashboard.getInstance().getTelemetry().addData("spindex target pos", targetPos);
        FtcDashboard.getInstance().getTelemetry().addData("spindexposDeg", spindexPosDeg);
        FtcDashboard.getInstance().getTelemetry().addData("distanciaB", distCm);
        FtcDashboard.getInstance().getTelemetry().addData("nBalls", nBalls);
        FtcDashboard.getInstance().getTelemetry().addData("spinError", SpinPID.getPositionError());
        FtcDashboard.getInstance().getTelemetry().addData("spindexPower", Spindex.getPower());
        FtcDashboard.getInstance().getTelemetry().addData("detected color", detectedColor);
        FtcDashboard.getInstance().getTelemetry().addData("Patron", obeliskPattern);
        FtcDashboard.getInstance().getTelemetry().addData("GreenBPos", GrenBallPos);
        FtcDashboard.getInstance().getTelemetry().addData("Extra ball", laser.getState());
        FtcDashboard.getInstance().getTelemetry().addData("Alpha", colorSensor.getNormalizedColors().alpha);*/
    }

    @Override
    public void log(TelemetryPacket packet) {
        packet.put("SpindexSubsystem/shootMode", Shootmode);
        packet.put("SpindexSubsystem/spindex target pos", targetPos);
        packet.put("SpindexSubsystem/spindexposDeg", spindexPosDeg);
        packet.put("SpindexSubsystem/Bpresence", getBPresence());
        packet.put("SpindexSubsystem/nBalls", nBalls);
        packet.put("SpindexSubsystem/spinError", SpinPID.getPositionError());
        packet.put("SpindexSubsystem/spindexPower", Spindex.getPower());
        packet.put("SpindexSubsystem/DetectedColor", detectedColor);
        packet.put("SpindexSubsystem/Patron", obeliskPattern);
        packet.put("SpindexSubsystem/GreenBPos", GrenBallPos);

        packet.put("SpindexSubsystem/ColorSensor/H", h);
        packet.put("SpindexSubsystem/ColorSensor/S", s);
        packet.put("SpindexSubsystem/ColorSensor/V", v);
       // packet.put("SpindexSubsystem/ColorSensor/FPS", colorSensorWebcam.getFps());
    }

    public DetectedColor getDetectedColor() {
        /*
        colors = colorSensor.getNormalizedColors();

        double r = colors.red;
        double b = colors.blue;
        double g = colors.green;

        double sum = r+g+b;
        double gRatio = g/sum;



        //FtcDashboard.getInstance().getTelemetry().addData("gRatio", gRatio);


        if (gRatio > 0.44 && g>r && g>b) {
            return DetectedColor.Green;
        }else {
            return DetectedColor.Unknown;
        }
        */
        return DetectedColor.Unknown;
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

    public void advanceOneSorting() {
        targetPos -= 120;
    }

    public void advanceOneshooting() {
        targetPos += 120;
    }

    public void ShootAll() {
        targetPos += 360;
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
        return laser.getState();
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
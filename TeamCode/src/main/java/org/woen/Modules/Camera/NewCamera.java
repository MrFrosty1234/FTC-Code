

package org.woen.Modules.Camera;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.woen.Utility.Team.TEAM;


import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class NewCamera implements VisionProcessor {

    //TODO static pos
    private static Position cameraPosition = new Position(DistanceUnit.METER,
            0, 0, 0, 0);

    private static YawPitchRollAngles cameraOrient = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;


    private int id = 0;

    public int id22 = 22;

    public int id21 = 21;

    public int id23 = 23;

    public static int height = 480;

    public static int width = 640;


    public static TEAM TEAM = null;

    Position position = null;

    YawPitchRollAngles orient = null;

    AprilTagPoseFtc pos = new AprilTagPoseFtc(0,0,0,0,0,0,0,0,0);



    HardwareMap hardwareMap;

    public NewCamera(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

    }


    public void closePortal() {
        visionPortal.close();
    }


    public int getId() {
        return id;
    }


    public AprilTagPoseFtc getDistance() {
        return pos;
    }

    ExposureControl exposureControl = null;

    DetectColor pipeline = new DetectColor();

    /// left
    public static double leftL = -0.8;
    public static double topL = 0.1;
    public static double rightL = -0.6;
    public static double bottomL = -0.1;
    /// center
    public static double   leftC = -0.1;
    public static double    topC = 0.1;
    public static double  rightC = 0.1;
    public static double bottomC = -0.1;
    /// right
    public static double   leftR = 0.6;
    public static double    topR = 0.1;
    public static double  rightR = 0.8;
    public static double bottomR = -0.1;
    PredominantColorProcessor leftDetection = new PredominantColorProcessor.Builder()
            .setRoi(ImageRegion.asUnityCenterCoordinates(leftL, topL, rightL, bottomL))
            .setSwatches(
                    PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                    PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                    PredominantColorProcessor.Swatch.RED,
                    PredominantColorProcessor.Swatch.BLUE,
                    PredominantColorProcessor.Swatch.YELLOW,
                    PredominantColorProcessor.Swatch.BLACK,
                    PredominantColorProcessor.Swatch.WHITE)
            .build();
    PredominantColorProcessor rightDetection = new PredominantColorProcessor.Builder()
            .setRoi(ImageRegion.asUnityCenterCoordinates(leftR, topR, rightR, bottomR))
            .setSwatches(
                    PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                    PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                    PredominantColorProcessor.Swatch.RED,
                    PredominantColorProcessor.Swatch.BLUE,
                    PredominantColorProcessor.Swatch.YELLOW,
                    PredominantColorProcessor.Swatch.BLACK,
                    PredominantColorProcessor.Swatch.WHITE)
            .build();
    PredominantColorProcessor centerDetection = new PredominantColorProcessor.Builder()
            .setRoi(ImageRegion.asUnityCenterCoordinates(leftC, topC, rightC, bottomC))
            .setSwatches(
                    PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                    PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                    PredominantColorProcessor.Swatch.RED,
                    PredominantColorProcessor.Swatch.BLUE,
                    PredominantColorProcessor.Swatch.YELLOW,
                    PredominantColorProcessor.Swatch.BLACK,
                    PredominantColorProcessor.Swatch.WHITE)
            .build();

    public void init() {
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrient)
                .build();


        this.TEAM = TEAM;

        this.position = position;

        this.orient = orient;



        VisionPortal.Builder builder = new VisionPortal.Builder();




        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        builder.addProcessor(aprilTag)
                .addProcessor(leftDetection)
                .addProcessor(rightDetection)
                .addProcessor(centerDetection);

        //builder.addProcessor(pipeline);


        builder.setCameraResolution(new Size(width, height));

        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        visionPortal = builder.build();

    }

    public void getColors(){
        PredominantColorProcessor.Result resultL = leftDetection.getAnalysis();
        PredominantColorProcessor.Result resultR = rightDetection.getAnalysis();
        PredominantColorProcessor.Result resultC = centerDetection.getAnalysis();

        FtcDashboard.getInstance().getTelemetry().addData("left",resultL.closestSwatch);
        FtcDashboard.getInstance().getTelemetry().addData("center",resultC.closestSwatch);
        FtcDashboard.getInstance().getTelemetry().addData("right",resultR.closestSwatch);

        FtcDashboard.getInstance().getTelemetry().update();
    }


    public void update() {
        List<AprilTagDetection> currentDetectionList = aprilTag.getDetections();



       // exposureControl.setExposure((long) (1/visionPortal.getFps()), TimeUnit.SECONDS);

        if (!currentDetectionList.isEmpty()) {
            for (AprilTagDetection tag : currentDetectionList) {
                if (tag.id == id21 || tag.id == id22 || tag.id == id23) {
                    id = tag.id;
                    break;
                }
                if (tag.id == 24 && TEAM == org.woen.Utility.Team.TEAM.RED) {
                    pos = tag.ftcPose;
                    break;
                }
                if (tag.id == 20 && TEAM == org.woen.Utility.Team.TEAM.BLUE) {
                    pos = tag.ftcPose;
                    break;
                }
            }
        }
        getColors();
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}

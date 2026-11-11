package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class Sistema_visao {

    public enum DetectedColor { RED, BLUE, YELLOW, NONE }

    public enum Mode { COLOR, APRILTAG }

    // ===== Ajustáveis pelo Dashboard (Câmera AprilTag) =====
    public static boolean USE_CUSTOM_TAG_LIBRARY = true;
    public static int TAG_ID_MIN = 30;
    public static int TAG_ID_MAX = 45;
    public static double TAG_SIZE_INCHES = 6.5;

    public static int TAG_CAM_WIDTH = 640;
    public static int TAG_CAM_HEIGHT = 480;
    public static int TAG_DECIMATION = 2;

    public static int EXPOSURE_MS = 6;
    public static int GAIN = 250;

    private ColorDetectorProcessor colorProcessor;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private Mode mode;

    public int getRedPixels() { return colorProcessor != null ? colorProcessor.redPixels : 0; }
    public int getBluePixels() { return colorProcessor != null ? colorProcessor.bluePixels : 0; }
    public int getYellowPixels() { return colorProcessor != null ? colorProcessor.yellowPixels : 0; }

    public void init(HardwareMap hardwareMap, String cameraName, Mode mode, int viewId) {
        this.mode = mode;

        Size resolution = (mode == Mode.COLOR)
                ? new Size(640, 480)
                : new Size(TAG_CAM_WIDTH, TAG_CAM_HEIGHT);

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, cameraName))
                .setCameraResolution(resolution)
                .setLiveViewContainerId(viewId);

        if (mode == Mode.COLOR) {
            colorProcessor = new ColorDetectorProcessor();
            builder.addProcessor(colorProcessor);
        } else {
            AprilTagProcessor.Builder tagBuilder = new AprilTagProcessor.Builder()
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .setDrawTagID(true)
                    .setDrawTagOutline(true)
                    .setDrawAxes(true);

            if (USE_CUSTOM_TAG_LIBRARY) {
                AprilTagLibrary.Builder lib = new AprilTagLibrary.Builder();
                for (int id = TAG_ID_MIN; id <= TAG_ID_MAX; id++) {
                    lib.addTag(id, "Tag " + id, TAG_SIZE_INCHES, DistanceUnit.INCH);
                }
                tagBuilder.setTagLibrary(lib.build());
            }

            aprilTagProcessor = tagBuilder.build();
            aprilTagProcessor.setDecimation(TAG_DECIMATION);
            builder.addProcessor(aprilTagProcessor);
        }

        visionPortal = builder.build();
    }

    public Mode getMode() { return mode; }

    public void setManualExposure() {
        if (visionPortal == null) return;
        try {
            long start = System.currentTimeMillis();
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING
                    && System.currentTimeMillis() - start < 3000) {
                Thread.sleep(20);
            }
            if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) return;

            ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
            if (exposure.getMode() != ExposureControl.Mode.Manual) {
                exposure.setMode(ExposureControl.Mode.Manual);
                Thread.sleep(50);
            }
            exposure.setExposure((long) EXPOSURE_MS, TimeUnit.MILLISECONDS);
            Thread.sleep(20);

            GainControl gain = visionPortal.getCameraControl(GainControl.class);
            gain.setGain(Math.max(gain.getMinGain(), Math.min(gain.getMaxGain(), GAIN)));
            Thread.sleep(20);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public DetectedColor getDetectedColor() {
        if (colorProcessor != null) {
            return colorProcessor.getDetectedColor();
        }
        return DetectedColor.NONE;
    }

    public List<AprilTagDetection> getDetections() {
        if (aprilTagProcessor == null) return new ArrayList<>();
        return aprilTagProcessor.getDetections();
    }

    public AprilTagSingleDetection getDetectionById(int id) {
        for (AprilTagDetection d : getDetections()) {
            if (d instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) d;
                if (single.id == id) return single;
            }
        }
        return null;
    }

    public List<Integer> getDetectedIds() {
        List<Integer> ids = new ArrayList<>();
        for (AprilTagDetection d : getDetections()) {
            if (d instanceof AprilTagSingleDetection) {
                ids.add(((AprilTagSingleDetection) d).id);
            }
        }
        return ids;
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    public void close() {
        if (visionPortal != null && visionPortal.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_CLOSED) {
            visionPortal.close();
        }
    }

    // ================= Processador de Cor =================
    static class ColorDetectorProcessor implements VisionProcessor {

        private DetectedColor detectedColor = DetectedColor.NONE;
        private Rect roi = new Rect(280, 200, 80, 80);

        public static int minThreshold = 1000;

        public int redPixels = 0;
        public int bluePixels = 0;
        public int yellowPixels = 0;

        // Declaracao dos limites HSV
        private Scalar lowerYellow = new Scalar(15, 100, 100);
        private Scalar upperYellow = new Scalar(30, 255, 255);

        private Scalar lowerBlue = new Scalar(100, 100, 100);
        private Scalar upperBlue = new Scalar(130, 255, 255);

        private Scalar lowerRed1 = new Scalar(0, 100, 100);
        private Scalar upperRed1 = new Scalar(10, 255, 255);
        private Scalar lowerRed2 = new Scalar(160, 100, 100);
        private Scalar upperRed2 = new Scalar(180, 255, 255);

        // Declaracao das matrizes de imagem
        private Mat hsvMat = new Mat();
        private Mat maskYellow = new Mat();
        private Mat maskBlue = new Mat();
        private Mat maskRed1 = new Mat();
        private Mat maskRed2 = new Mat();
        private Mat maskRed = new Mat();

        @Override
        public void init(int width, int height, CameraCalibration calibration) {}

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

            // Declaracao do submat (corte da regiao de interesse)
            Mat submat = hsvMat.submat(roi);

            // Mascaras de cor
            Core.inRange(submat, lowerYellow, upperYellow, maskYellow);
            Core.inRange(submat, lowerBlue, upperBlue, maskBlue);
            Core.inRange(submat, lowerRed1, upperRed1, maskRed1);
            Core.inRange(submat, lowerRed2, upperRed2, maskRed2);
            Core.add(maskRed1, maskRed2, maskRed);

            yellowPixels = Core.countNonZero(maskYellow);
            bluePixels = Core.countNonZero(maskBlue);
            redPixels = Core.countNonZero(maskRed);

            if (redPixels > minThreshold && redPixels > bluePixels && redPixels > yellowPixels) {
                detectedColor = DetectedColor.RED;
            } else if (bluePixels > minThreshold && bluePixels > redPixels && bluePixels > yellowPixels) {
                detectedColor = DetectedColor.BLUE;
            } else if (yellowPixels > minThreshold && yellowPixels > redPixels && yellowPixels > bluePixels) {
                detectedColor = DetectedColor.YELLOW;
            } else {
                detectedColor = DetectedColor.NONE;
            }

            submat.release();
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBilinear, float scaleCanvas, Object userContext) {}

        public DetectedColor getDetectedColor() {
            return detectedColor;
        }
    }
}
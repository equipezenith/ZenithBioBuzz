package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import com.acmerobotics.dashboard.config.Config; // Import do FTC Dashboard
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import android.util.Size;

@Config // Permite alterar variáveis pelo Dashboard
public class Sistema_visao {

    public enum DetectedColor { RED, BLUE, YELLOW, NONE }

    private ColorDetectorProcessor colorProcessor;
    private VisionPortal visionPortal;

    // Métodos para ver a contagem de pixels na telemetria
    public int getRedPixels() { return colorProcessor != null ? colorProcessor.redPixels : 0; }
    public int getBluePixels() { return colorProcessor != null ? colorProcessor.bluePixels : 0; }

    public void init(HardwareMap hardwareMap) {
        colorProcessor = new ColorDetectorProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                colorProcessor
        );
    }

    public DetectedColor getDetectedColor() {
        if (colorProcessor != null) {
            return colorProcessor.getDetectedColor();
        }
        return DetectedColor.NONE;
    }

    // Permite que o FtcDashboard acesse o fluxo de vídeo da câmera
    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    public void close() {
        if (visionPortal != null && visionPortal.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_CLOSED) {
            visionPortal.close();
        }
    }

    static class ColorDetectorProcessor implements VisionProcessor {

        private DetectedColor detectedColor = DetectedColor.NONE;
        private Rect roi = new Rect(280, 200, 80, 80);

        // Limite mínimo de pixels para considerar uma cor válida
        public static int minThreshold = 1000;

        // Pixels contados (para debugging)
        public int redPixels = 0;
        public int bluePixels = 0;
        public int yellowPixels = 0;

        // Limites HSV
        private Scalar lowerBlue = new Scalar(100, 100, 100);
        private Scalar upperBlue = new Scalar(130, 255, 255);
        private Scalar lowerRed1 = new Scalar(0, 100, 100);
        private Scalar upperRed1 = new Scalar(10, 255, 255);
        private Scalar lowerRed2 = new Scalar(160, 100, 100);
        private Scalar upperRed2 = new Scalar(180, 255, 255);

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
            Mat submat = hsvMat.submat(roi);

            Core.inRange(submat, lowerBlue, upperBlue, maskBlue);
            Core.inRange(submat, lowerRed1, upperRed1, maskRed1);
            Core.inRange(submat, lowerRed2, upperRed2, maskRed2);
            Core.add(maskRed1, maskRed2, maskRed);

            bluePixels = Core.countNonZero(maskBlue);
            redPixels = Core.countNonZero(maskRed);

            if (redPixels > minThreshold && redPixels > bluePixels) {
                detectedColor = DetectedColor.RED;
            } else if (bluePixels > minThreshold && bluePixels > redPixels) {
                detectedColor = DetectedColor.BLUE;
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
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

/**
 * Cada instância controla UMA câmera, com UM VisionPortal.
 *   - Mode.COLOR    -> câmera do indexer (detecção de cor)
 *   - Mode.APRILTAG -> câmera que olha a tag
 * Use duas instâncias no TeleOp, uma para cada câmera.
 */
@Config // Permite alterar variáveis pelo Dashboard
public class Sistema_visao {

    public enum DetectedColor { RED, BLUE, YELLOW, NONE }

    public enum Mode { COLOR, APRILTAG }

    // ===== Ajustáveis pelo Dashboard (câmera da AprilTag) =====
    // Biblioteca própria com as tags do jogo, para TODAS terem pose (range/bearing/yaw)
    public static boolean USE_CUSTOM_TAG_LIBRARY = true;
    public static int TAG_ID_MIN = 30;
    public static int TAG_ID_MAX = 45;
    public static double TAG_SIZE_INCHES = 6.5;   // MEÇA: lado do quadrado PRETO da tag, em polegadas

    // Resolução da câmera da tag: mais pixels = enxerga mais tags e de mais longe (custa CPU/USB)
    public static int TAG_CAM_WIDTH = 640;
    public static int TAG_CAM_HEIGHT = 480;
    public static int TAG_DECIMATION = 2;         // 1 = mais alcance/precisão, 3 = mais FPS

    // Exposição baixa + ganho alto = loop mais rápido e menos borrão (ajuste para a sua sala/webcam)
    public static int EXPOSURE_MS = 6;
    public static int GAIN = 250;

    private ColorDetectorProcessor colorProcessor;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private Mode mode;

    // Métodos para ver a contagem de pixels na telemetria
    public int getRedPixels() { return colorProcessor != null ? colorProcessor.redPixels : 0; }
    public int getBluePixels() { return colorProcessor != null ? colorProcessor.bluePixels : 0; }

    /**
     * @param cameraName nome da webcam na configuração do robô (ex.: "Webcam 1")
     * @param mode       COLOR ou APRILTAG
     * @param viewId     id da área de live view (veja makeMultiPortalView no TeleOp)
     */
    public void init(HardwareMap hardwareMap, String cameraName, Mode mode, int viewId) {
        this.mode = mode;

        // O ROI da cor foi feito para 640x480; a câmera da tag usa a resolução configurável
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

    /**
     * Exposição manual baixa + ganho alto (dica do vídeo). Chame no INIT do OpMode,
     * depois do init(). Espera a câmera começar a transmitir (até ~3 s).
     */
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

    /** Lista de tags detectadas (só funciona em Mode.APRILTAG). */
    public List<AprilTagDetection> getDetections() {
        if (aprilTagProcessor == null) return new ArrayList<>();
        return aprilTagProcessor.getDetections();
    }

    /**
     * Detecção de uma tag específica (ex.: 36) ou null se ela não está sendo vista agora.
     * No SDK 12+, 'id' e 'metadata' ficam em AprilTagSingleDetection, não na classe base.
     * A pose (ftcPose.range/bearing/yaw) continua disponível normalmente.
     */
    public AprilTagSingleDetection getDetectionById(int id) {
        for (AprilTagDetection d : getDetections()) {
            if (d instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) d;
                if (single.id == id) return single;
            }
        }
        return null;
    }

    /** IDs de todas as tags (individuais) vistas neste momento (útil para telemetria). */
    public List<Integer> getDetectedIds() {
        List<Integer> ids = new ArrayList<>();
        for (AprilTagDetection d : getDetections()) {
            if (d instanceof AprilTagSingleDetection) {
                ids.add(((AprilTagSingleDetection) d).id);
            }
        }
        return ids;
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

    // ================= Seu processador de cor (sem alterações) =================
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
package org.firstinspires.ftc.toolkit.PhoneCamera.Vuforia;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.HINT;
import com.vuforia.Vuforia;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.toolkit.Utilities.Constants;
/**
 * A collection of functions for locating images in
 * 3D space, using the robot controller phone's camera
 * <p>
 * NOTE: For the 2017-18 First Tech Challenge, Relic Recovery, the only "target" that
 * this class will be working with is the crypto key VuMark, so its functions will not
 * have parameters for which target to track, since there is only one. This will be changed
 * whenever there are more available targets
 */
public class ImageTracker {
    private VuforiaLocalizer.Parameters vuforiaSettings;
    private VuforiaTrackables targets;
    private CameraSide cameraSide;
    private PhoneOrientation orientation;
    private VuforiaTrackableDefaultListener listener;
    public ImageTracker(CameraSide cameraSide, PhoneOrientation orientation, int maxSimultaneousImageTargets, boolean showCameraFeedbackOnPhone, int cameraMonitorViewId) {
        this.cameraSide = cameraSide;
        this.orientation = orientation;
        if(showCameraFeedbackOnPhone) {
            //TODO: Make A way for the module to have access to R.id.CameraMonitorView
            vuforiaSettings = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        } else {
            vuforiaSettings = new VuforiaLocalizer.Parameters();
        }

        switch(cameraSide) {
            case SCREEN:
                vuforiaSettings.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
                break;
            case BACK:
                vuforiaSettings.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
                break;
        }

        vuforiaSettings.vuforiaLicenseKey = Constants.LICENSE;
        vuforiaSettings.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, maxSimultaneousImageTargets);
    }



    public void init() {
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(vuforiaSettings);

        targets = vuforia.loadTrackablesFromAsset("RelicVuMark");

        targets.get(0).setName("Crypto Key");

        targets.activate();

        listener = (VuforiaTrackableDefaultListener) targets.get(0).getListener();
    }


    public boolean canSeeTarget() {
        return listener.getPose() != null;
    }


    public CryptoColumn decodeTarget() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(targets.get(0));
        switch(vuMark) {
            case UNKNOWN:
                return CryptoColumn.UNKNOWN;
            case LEFT:
                return CryptoColumn.LEFT;
            case CENTER:
                return CryptoColumn.CENTER;
            case RIGHT:
                return CryptoColumn.RIGHT;
            default:
                return CryptoColumn.UNKNOWN;
        }
    }



    public CameraSide getCameraSide() {
        return cameraSide;
    }

    public PhoneOrientation getOrientation() {
        return orientation;
    }
}
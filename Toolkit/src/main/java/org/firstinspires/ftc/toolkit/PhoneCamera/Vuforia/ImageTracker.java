package org.firstinspires.ftc.toolkit.PhoneCamera.Vuforia;


import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.toolkit.R;
import org.firstinspires.ftc.toolkit.Utilities.Constants;
import org.firstinspires.ftc.toolkit.Utilities.VectorClasses.Vector3D;
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

    /**
     * Creates the ImageTracker with necessary parameters
     *
     * @param cameraSide                  The camera that will be used to locate images
     * @param orientation                 The orientation of the phone on the robot
     * @param maxSimultaneousImageTargets The maximum number of image targets that can be tracked at one time
     * @param showCameraFeedbackOnPhone   Whether or not what the camera is seeing should be displayed on the
     *                                    robot controller phone. This is good for debugging, but eats through
     *                                    the phone battery
     */
    public ImageTracker(CameraSide cameraSide, PhoneOrientation orientation, int maxSimultaneousImageTargets, boolean showCameraFeedbackOnPhone) {
        this.cameraSide = cameraSide;
        this.orientation = orientation;

        if(showCameraFeedbackOnPhone) {
            vuforiaSettings = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
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

    /**
     * Initializes the ImageTracker. This should be called during either
     * is necessary for the ImageTracker class to function properly
     */
    public void init() {
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(vuforiaSettings);

        targets = vuforia.loadTrackablesFromAsset("RelicVuMark");

        targets.get(0).setName("Crypto Key");

        targets.activate();

        listener = (VuforiaTrackableDefaultListener) targets.get(0).getListener();
    }

    /**
     * Checks if the target is visible
     *
     * @return Whether or not it is visible
     */
    public boolean canSeeTarget() {
        return listener.getPose() != null;
    }

    /**
     * Only valid for FTC Relic Recovery (2017-2018) challenge, will be removed when the season ends
     *
     * @return Returns either LEFT, CENTER, or RIGHT, which correspond to the column which will award
     * bonus points if a cube is placed inside it during autonomous
     */
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

    /**
     * Returns the position, using right-hand rule, of the target in 3D space
     *
     * @return The translation of the target in 3D space, relative to the phone, where x
     * is the distance from the origin on the x axis, y is the distance from the
     * origin on the y axis, and z is the distance from the origin on the z axis.
     * These values are all given in millimeters, and use the right-hand rule to
     * determine the meaning of positive and negative values
     */
    public Vector3D getRelativeTargetTranslation() {
        OpenGLMatrix pose = listener.getPose();

        Vector3D result = new Vector3D();

        if(pose != null) {
            VectorF translation = pose.getTranslation();

            switch(orientation) {
                case UPRIGHT:
                    result.x = translation.get(0);
                    result.y = translation.get(1);
                    break;
                case UPSIDE_DOWN:
                    result.x = -translation.get(0);
                    result.y = -translation.get(1);
                    break;
                case VOLUME_SIDE_DOWN:
                    result.x = translation.get(1);
                    result.y = -translation.get(0);
                    break;
                case VOLUME_SIDE_UP:
                    result.x = -translation.get(1);
                    result.y = translation.get(0);
                    break;
            }

            result.z = translation.get(2);

            return result;
        } else return Vector3D.ZERO;
    }

    /**
     * Returns the rotation, of the target in 3D space
     *
     * @return The rotation of the target in 3D space, relative to the phone, using
     * Euler angles, where x is pitch, y is yaw, and z is roll, and all are
     * in degrees, where a counter-clockwise rotation is positive. For more
     * help, look into the right-hand rule, because this can get confusing
     * very quickly
     */
    public Vector3D getRelativeTargetRotation() { //TODO: Implement this function for ALL phone orientations
        OpenGLMatrix rawPose = listener.getRawPose();
        AxesOrder axesOrder = AxesOrder.XYZ;
        double xMod = 1, yMod = 1, zMod = 1;

        if(rawPose != null) {
            switch(cameraSide) {
                case SCREEN:
                    switch(orientation) {
                        case UPRIGHT:
                            axesOrder = AxesOrder.XZX;
                            break;
                        case UPSIDE_DOWN:

                            break;
                        case VOLUME_SIDE_DOWN:

                            break;
                        case VOLUME_SIDE_UP:
                            break;
                    }
                    break;
                case BACK:
                    switch(orientation) {
                        case UPRIGHT:
                            axesOrder = AxesOrder.XYZ;
                            break;
                        case UPSIDE_DOWN:

                            break;
                        case VOLUME_SIDE_DOWN:

                            break;
                        case VOLUME_SIDE_UP:
                            break;
                    }
                    break;
            }

            Orientation rotation = Orientation.getOrientation(rawPose, AxesReference.EXTRINSIC, axesOrder, AngleUnit.DEGREES);

            return new Vector3D(rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        } else return Vector3D.ZERO;
    }

    /**
     * Getter for cameraSide
     *
     * @return The camera being used to locate images
     */
    public CameraSide getCameraSide() {
        return cameraSide;
    }

    /**
     * Getter for orientation
     *
     * @return The orientation of the phone on the robot
     */
    public PhoneOrientation getOrientation() {
        return orientation;
    }
}
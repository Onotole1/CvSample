package components;

import org.opencv.core.Core;

import java.util.LinkedHashMap;
import java.util.Map;

public class Constants {
    public static final int NO_ROTATION_VALUE = -754;
    public static final String NO_ROTATION_KEY = "No rotation";

    public static final int ROTATION_VALUE_180 = -1;
    public static  final String ROTATION_KEY_180 = "180";
    public static final String ROTATION_KEY_90_CLOCKWISE = "90 clockwise";
    public static final String ROTATION_KEY_90_COUNTERCLOCKWISE = "90 counterclockwise";

    public static final Map<String, Integer> ROTATIONS = new LinkedHashMap<>(4);
    static {
        ROTATIONS.put(Constants.NO_ROTATION_KEY, Constants.NO_ROTATION_VALUE);
        ROTATIONS.put(Constants.ROTATION_KEY_90_CLOCKWISE, Core.ROTATE_90_CLOCKWISE);
        ROTATIONS.put(Constants.ROTATION_KEY_90_COUNTERCLOCKWISE, Core.ROTATE_90_COUNTERCLOCKWISE);
        ROTATIONS.put(Constants.ROTATION_KEY_180, Constants.ROTATION_VALUE_180);
    }
}

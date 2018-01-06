package components;

import java.util.prefs.Preferences;

public final class PreferencesProvider {

    private static final String NUM_DISPARITIES_KEY = PreferencesProvider.class.getCanonicalName() + ".NUM_DISPARITIES_KEY";
    private static final String BLOCK_SIZE_KEY = PreferencesProvider.class.getCanonicalName() + ".BLOCK_SIZE_KEY";
    private static final String WINDOW_SIZE_KEY = PreferencesProvider.class.getCanonicalName() + ".WINDOW_SIZE_KEY";
    private static final String MIN_DISPARITIES_KEY = PreferencesProvider.class.getCanonicalName() + ".MIN_DISPARITIES_KEY";
    private static final String DISP_12_MAX_DISP_KEY = PreferencesProvider.class.getCanonicalName() + ".DISP_12_MAX_DISP_KEY";
    private static final String UNIQUENESS_RATIO_KEY = PreferencesProvider.class.getCanonicalName() + ".UNIQUENESS_RATIO_KEY";
    private static final String SPECKLE_WINDOW_SIZE_KEY = PreferencesProvider.class.getCanonicalName() + ".SPECKLE_WINDOW_SIZE_KEY";
    private static final String SPECKLE_RANGE_KEY = PreferencesProvider.class.getCanonicalName() + ".SPECKLE_RANGE_KEY";
    private static final String PRE_FILTERED_KEY = PreferencesProvider.class.getCanonicalName() + ".PRE_FILTERED_KEY";


    private static final int BLOCK_SIZE_DEFAULT = 15;
    private static final int WINDOW_SIZE_DEFAULT = 3;
    private static final int MIN_DISPARITIES_DEFAULT = 16;
    private static final int NUM_DISPARITIES_DEFAULT = 112 - MIN_DISPARITIES_DEFAULT;
    private static final int DISP_12_MAX_DISP_DEFAULT = 1;
    private static final int UNIQUENESS_RATIO_DEFAULT = 10;
    private static final int SPECKLE_WINDOW_SIZE_DEFAULT = 100;
    private static final int SPECKLE_RANGE_DEFAULT = 32;
    private static final int PRE_FILTERED_DEFAULT = 0;

    private final Preferences preferences = Preferences.userRoot().node(PreferencesProvider.class.getCanonicalName());

    public void savePreFiltered(final int preFiltered) {
        preferences.putInt(PRE_FILTERED_KEY, preFiltered);
    }

    public void saveNumDisparities(final int numDisparities) {
        preferences.putInt(NUM_DISPARITIES_KEY, numDisparities);
    }

    public void saveBlockSize(final int blockSize) {
        preferences.putInt(BLOCK_SIZE_KEY, blockSize);
    }

    public void saveWindowSize(final int windowSize) {
        preferences.putInt(WINDOW_SIZE_KEY, windowSize);
    }

    public void saveMinDisparities(final int minDisparities) {
        preferences.putInt(MIN_DISPARITIES_KEY, minDisparities);
    }

    public void saveDisp12MaxDisp(final int disp12MaxDisp) {
        preferences.putInt(DISP_12_MAX_DISP_KEY, disp12MaxDisp);
    }

    public void saveUniquenessRatio(final int uniquenessRatio) {
        preferences.putInt(UNIQUENESS_RATIO_KEY, uniquenessRatio);
    }

    public void saveSpeckleWindowSize(final int speckleWindowSize) {
        preferences.putInt(SPECKLE_WINDOW_SIZE_KEY, speckleWindowSize);
    }

    public void saveSpeckleRange(final int speckleRange) {
        preferences.putInt(SPECKLE_RANGE_KEY, speckleRange);
    }

    public int getPreFiltered() {
        return preferences.getInt(NUM_DISPARITIES_KEY, PRE_FILTERED_DEFAULT);
    }

    public int getNumDisparities() {
        return preferences.getInt(NUM_DISPARITIES_KEY, NUM_DISPARITIES_DEFAULT);
    }

    public int getBlockSize() {
        return preferences.getInt(BLOCK_SIZE_KEY, BLOCK_SIZE_DEFAULT);
    }

    public int getWindowSize() {
        return preferences.getInt(WINDOW_SIZE_KEY, WINDOW_SIZE_DEFAULT);
    }

    public int getMinDisparities() {
        return preferences.getInt(MIN_DISPARITIES_KEY, MIN_DISPARITIES_DEFAULT);
    }

    public int getDisp12MaxDisp() {
        return preferences.getInt(DISP_12_MAX_DISP_KEY, DISP_12_MAX_DISP_DEFAULT);
    }

    public int getUniquenessRatio() {
        return preferences.getInt(UNIQUENESS_RATIO_KEY, UNIQUENESS_RATIO_DEFAULT);
    }

    public int getSpeckleWindowSize() {
        return preferences.getInt(SPECKLE_WINDOW_SIZE_KEY, SPECKLE_WINDOW_SIZE_DEFAULT);
    }

    public int getSpeckleRange() {
        return preferences.getInt(SPECKLE_RANGE_KEY, SPECKLE_RANGE_DEFAULT);
    }

    public void setDefaults() {
        saveBlockSize(BLOCK_SIZE_DEFAULT);
        saveDisp12MaxDisp(DISP_12_MAX_DISP_DEFAULT);
        saveMinDisparities(MIN_DISPARITIES_DEFAULT);
        saveNumDisparities(NUM_DISPARITIES_DEFAULT);
        saveSpeckleRange(SPECKLE_RANGE_DEFAULT);
        saveSpeckleWindowSize(SPECKLE_WINDOW_SIZE_DEFAULT);
        saveUniquenessRatio(UNIQUENESS_RATIO_DEFAULT);
        saveWindowSize(WINDOW_SIZE_DEFAULT);
        savePreFiltered(PRE_FILTERED_DEFAULT);
    }
}

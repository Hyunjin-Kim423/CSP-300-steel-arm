using System;
using UnityEngine;

// All user settings of the app, saved on the headset (PlayerPrefs) so they survive restarts.
//
//   AppSettings.Current         the values
//   AppSettings.NotifyChanged() call after changing a value: saves it and tells everyone (Changed event)
//   AppSettings.Changed         components that use a setting listen to this and apply the new value right away
[Serializable]
public class AppSettings
{
    public enum Language { Korean, English }
    public enum ColorScheme { GreenRed, BlueOrange }
    public enum ScanMode { Button, Live }
    public enum OutlineWidth { Thin, Normal, Thick }

    // ----- Scan -----
    public bool silhouette = true;              // false = bounding boxes
    public bool replaceOnlyInView = true;       // a new scan replaces only what is in view
    public bool scanWave = true;
    public float waveIntensity = 0.8f;          // 0..1

    // ----- Menu -----
    public float menuYawAngle = 30f;            // degrees the object menu follows the gaze sideways
    public float menuPitchAngle = 15f;          // degrees up / down
    public float menuDistance = 0.55f;          // meters from the player (the thumbstick changes it too)
    public bool menuMirror = true;              // action buttons on the side away from the object

    // ----- Display -----
    public ColorScheme colorScheme = ColorScheme.GreenRed;
    public float fillAlpha = 0.35f;             // 0..1
    public OutlineWidth outlineWidth = OutlineWidth.Normal;
    public bool showLabel = false;              // "cup 0.87" label when a menu opens

    // ----- Language -----
    public Language language = Language.Korean;

    // ----- Advanced -----
    public ScanMode scanMode = ScanMode.Button;
    public bool useTracking = true;             // live mode only
    public bool showLog = false;

    // ---------- derived values ----------

    public Color RegisteredColor => colorScheme == ColorScheme.GreenRed ? Hex(0x4ADE80) : Hex(0x60A5FA);
    public Color UnregisteredColor => colorScheme == ColorScheme.GreenRed ? Hex(0xF87171) : Hex(0xFB923C);

    public float OutlineWidthMeters => outlineWidth == OutlineWidth.Thin ? 0.0025f
        : outlineWidth == OutlineWidth.Thick ? 0.006f : 0.004f;

    static Color Hex(int rgb) => new Color(((rgb >> 16) & 0xFF) / 255f, ((rgb >> 8) & 0xFF) / 255f, (rgb & 0xFF) / 255f, 1f);

    // ---------- storage ----------

    const string PrefsKey = "SteelArm.Settings";
    static AppSettings s_Current;

    public static event Action Changed;

    public static AppSettings Current
    {
        get
        {
            if (s_Current == null)
                Load();
            return s_Current;
        }
    }

    static void Load()
    {
        string json = PlayerPrefs.GetString(PrefsKey, "");
        s_Current = null;
        if (!string.IsNullOrEmpty(json))
        {
            try { s_Current = JsonUtility.FromJson<AppSettings>(json); }
            catch (Exception e) { Debug.LogWarning("[steelarm] Saved settings could not be read; using defaults. " + e.Message); }
        }
        if (s_Current == null)
            s_Current = new AppSettings();
    }

    public static void Save()
    {
        PlayerPrefs.SetString(PrefsKey, JsonUtility.ToJson(Current));
        PlayerPrefs.Save();
    }

    // Call after changing values. save = false while a slider is being dragged (Save() runs when the menu closes).
    public static void NotifyChanged(bool save = true)
    {
        if (save)
            Save();
        Changed?.Invoke();
    }

    public static void ResetToDefaults()
    {
        s_Current = new AppSettings();
        NotifyChanged();
    }
}

using TMPro;
using UnityEngine;
using UnityEngine.UI;

// Parts of one settings row (built by SteelArm > Build Settings Menu Prefab). A row type uses only some of them:
//   segmented: label, description, segmentRoot      toggle: label, description, switch*
//   slider:    label, value, slider                 info:   label (caption), value
public class SettingsRow : MonoBehaviour
{
    public TMP_Text label;
    public TMP_Text description;
    public TMP_Text value;
    public RectTransform segmentRoot;
    public Button switchButton;
    public Image switchTrack;
    public RectTransform switchKnob;
    public Image switchKnobImage;
    public Slider slider;
    public CanvasGroup group;
}

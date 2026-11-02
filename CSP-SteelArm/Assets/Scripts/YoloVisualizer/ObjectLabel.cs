using TMPro;
using UnityEngine;

// The floating text ("cup 0.87") shown at the top-left of an object after it is selected.
public class ObjectLabel
{
    readonly TextMeshPro text;
    readonly float heightAtOneMeter;
    readonly float duration;
    float hideTime;

    // heightAtOneMeter: text height in meters for an object 1 m away (scales with distance)
    // duration: how long the label stays visible (seconds)
    public ObjectLabel(float heightAtOneMeter, float duration, Color color)
    {
        this.heightAtOneMeter = heightAtOneMeter;
        this.duration = duration;

        text = new GameObject("YoloLabel", typeof(RectTransform)).AddComponent<TextMeshPro>();
        text.rectTransform.pivot = Vector2.zero;
        text.rectTransform.sizeDelta = Vector2.zero;
        text.alignment = TextAlignmentOptions.BottomLeft;
        text.textWrappingMode = TextWrappingModes.NoWrap;
        text.color = color;
        text.outlineWidth = 0.2f;
        text.outlineColor = new Color32(0, 0, 0, 255);
        text.gameObject.SetActive(false);
    }

    public void Show(DetectedObject obj)
    {
        Vector3 topLeft = obj.corners[1];
        text.text = obj.text;
        text.color = obj.color;
        text.fontSize = heightAtOneMeter * obj.depth * 10f;
        text.transform.SetPositionAndRotation(topLeft, Quaternion.LookRotation(topLeft - obj.cameraPosition));
        text.gameObject.SetActive(true);
        hideTime = Time.time + duration;
    }

    public void Hide() => text.gameObject.SetActive(false);

    // Call every frame: hides the label when its time is up.
    public void Tick()
    {
        if (Time.time > hideTime)
            Hide();
    }
}

using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

// Shows the latest "[steelarm]" log lines on a small panel at the lower left of the view (Settings > Advanced > Show log).
// Creates itself; nothing needs to be added to the scene.
public class ScreenLog : MonoBehaviour
{
    const int MaxLines = 8;
    const float Scale = 0.0006f;
    static readonly Vector3 ViewOffset = new Vector3(-0.22f, -0.2f, 0.75f); // in head space (meters)

    readonly Queue<string> lines = new Queue<string>();
    GameObject panel;
    TMP_Text text;
    Transform head;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void CreateIfMissing()
    {
        if (FindAnyObjectByType<ScreenLog>() == null)
            new GameObject(nameof(ScreenLog)).AddComponent<ScreenLog>();
    }

    void Awake()
    {
        panel = new GameObject("ScreenLogPanel", typeof(RectTransform));
        panel.transform.SetParent(transform, false);
        var rect = (RectTransform)panel.transform;
        rect.sizeDelta = new Vector2(640f, 250f);
        rect.localScale = Vector3.one * Scale;
        var canvas = panel.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.WorldSpace;
        panel.AddComponent<Image>().color = new Color(0.07f, 0.078f, 0.094f, 0.8f);

        var textObject = new GameObject("Text", typeof(RectTransform));
        textObject.transform.SetParent(panel.transform, false);
        var textRect = (RectTransform)textObject.transform;
        textRect.anchorMin = Vector2.zero;
        textRect.anchorMax = Vector2.one;
        textRect.offsetMin = new Vector2(16f, 12f);
        textRect.offsetMax = new Vector2(-16f, -12f);
        text = textObject.AddComponent<TextMeshProUGUI>();
        if (TMP_Settings.defaultFontAsset != null)
            text.font = TMP_Settings.defaultFontAsset;
        text.fontSize = 15f;
        text.alignment = TextAlignmentOptions.BottomLeft;
        text.textWrappingMode = TextWrappingModes.Normal;
        text.raycastTarget = false;

        Application.logMessageReceived += OnLog;
        AppSettings.Changed += Apply;
        Apply();
    }

    void OnDestroy()
    {
        Application.logMessageReceived -= OnLog;
        AppSettings.Changed -= Apply;
    }

    void Apply() => panel.SetActive(AppSettings.Current.showLog);

    void OnLog(string message, string stackTrace, LogType type)
    {
        if (!message.StartsWith("[steelarm]"))
            return;
        string color = type == LogType.Log ? "#E5E7EB" : type == LogType.Warning ? "#FCD34D" : "#FCA5A5";
        lines.Enqueue($"<color={color}>{System.DateTime.Now:HH:mm:ss}  {message.Substring(10).Trim()}</color>");
        while (lines.Count > MaxLines)
            lines.Dequeue();
        text.text = string.Join("\n", lines);
    }

    void LateUpdate()
    {
        if (!panel.activeSelf)
            return;
        if (head == null)
        {
            if (Camera.main == null)
                return;
            head = Camera.main.transform;
        }
        // Lags slightly behind the head so it doesn't feel glued to the face.
        Quaternion yawPitch = head.rotation;
        Vector3 target = head.position + yawPitch * ViewOffset;
        panel.transform.position = Vector3.Lerp(panel.transform.position, target, 1f - Mathf.Exp(-8f * Time.deltaTime));
        panel.transform.rotation = Quaternion.LookRotation(panel.transform.position - head.position, Vector3.up);
    }
}

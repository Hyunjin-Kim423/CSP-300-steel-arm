using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.UI;

// The settings panel (left controller ≡ button, see SettingsMenuOpener).
// Tabs on the left; the selected tab's rows on the right. Rows are made at runtime from the row templates in the prefab,
// so the look of each row type is edited in one place. Every change is applied right away (AppSettings.Changed).
//
// Like the Quest OS windows, the panel opens in front of the player's gaze and then stays where it is; the bar under
// it (SettingsGrabBar) carries it around. While it is open the surroundings are dimmed, and the scan results and
// the object menu are hidden (YoloBoxVisualizer listens to OpenStateChanged).
public class SettingsMenu : MonoBehaviour
{
    [Header("Parts (filled in by the prefab builder)")]
    public CanvasGroup canvasGroup;
    public TMP_Text navTitle;
    public TMP_Text navHint;
    public TMP_Text pageTitle;
    public TMP_Text resetLabel;
    public RectTransform tabRoot;
    public RectTransform rowRoot;
    public MenuButtonVisual closeButton;
    public MenuButtonVisual resetButton;
    [Header("Templates")]
    public MenuButtonVisual tabTemplate;
    public MenuButtonVisual segmentTemplate;
    public SettingsRow segmentedRowTemplate;
    public SettingsRow toggleRowTemplate;
    public SettingsRow sliderRowTemplate;
    public SettingsRow infoRowTemplate;

    [Header("Placement")]
    [Tooltip("Distance from the player's eyes (meters).")]
    public float distance = 0.8f;
    [Tooltip("Height relative to eye level (meters).")]
    public float heightOffset = -0.05f;
    public float animationTime = 0.15f;

    [Header("Dim the surroundings")]
    [Tooltip("How dark everything behind the panel gets while it is open (0 = off).")]
    [Range(0f, 1f)] public float dimAmount = 0.6f;

    static readonly Color Accent = new Color(0.29f, 0.87f, 0.5f, 1f);
    static readonly Color AccentText = new Color(0.04f, 0.14f, 0.08f, 1f);
    static readonly Color Muted = new Color(0.76f, 0.79f, 0.82f, 1f);
    static readonly Color Danger = new Color(0.94f, 0.27f, 0.27f, 1f);
    static readonly Color Clear = new Color(1f, 1f, 1f, 0f);
    static readonly Color HoverTint = new Color(1f, 1f, 1f, 0.12f);
    static readonly Color TabSelected = new Color(0.29f, 0.87f, 0.5f, 0.16f);
    static readonly Color SwitchOff = new Color(1f, 1f, 1f, 0.18f);
    static readonly Color KnobOff = new Color(0.85f, 0.87f, 0.9f, 1f);
    static readonly Color DotOff = new Color(1f, 1f, 1f, 0.25f);

    enum Tab { Scan, Menu, Display, Advanced, Language }
    static readonly Tab[] Tabs = { Tab.Scan, Tab.Menu, Tab.Display, Tab.Advanced, Tab.Language };
    static readonly string[] TabKeys = { "tab.scan", "tab.menu", "tab.display", "tab.advanced", "tab.language" };

    static AppSettings S => AppSettings.Current;

    Tab tab = Tab.Scan;
    Transform head;
    Vector3 baseScale;
    bool open;
    bool applying;      // our own change is being applied: don't rebuild in response to it
    bool unsaved;       // slider changes are saved when the panel closes
    Coroutine fade;
    readonly List<Action> refreshers = new List<Action>();
    TMP_Text rosValue;
    TMP_Text topicValue;
    TMP_Text lastValue;
    float nextStatusTime;

    public bool IsOpen => open;

    // true while a settings panel is open; YoloBoxVisualizer hides the scan results and blocks scanning meanwhile.
    public static bool AnyOpen { get; private set; }
    public static event Action<bool> OpenStateChanged;

    GameObject dimmer;
    Material dimMaterial;
    bool warnedNoDimmer;

    // ---------- open / close ----------

    void Awake()
    {
        baseScale = transform.localScale;
        foreach (Component template in new Component[] { tabTemplate, segmentTemplate, segmentedRowTemplate, toggleRowTemplate, sliderRowTemplate, infoRowTemplate })
        {
            if (template != null)
                template.gameObject.SetActive(false);
        }

        closeButton.GetComponent<Button>().onClick.AddListener(Close);
        resetButton.GetComponent<Button>().onClick.AddListener(AppSettings.ResetToDefaults);

        var canvas = GetComponent<Canvas>();
        if (canvas != null && canvas.worldCamera == null)
            canvas.worldCamera = Camera.main;
    }

    void OnEnable() => AppSettings.Changed += OnSettingsChanged;

    void OnDisable()
    {
        AppSettings.Changed -= OnSettingsChanged;
        SaveIfNeeded();
    }

    public void Toggle()
    {
        if (open)
            Close();
        else
            Open();
    }

    public void Open()
    {
        head = Camera.main != null ? Camera.main.transform : null;
        if (!gameObject.activeSelf)
        {
            canvasGroup.alpha = 0f;
            gameObject.SetActive(true);
        }
        open = true;
        // Every time it opens: in front of where the player is looking now.
        if (head != null)
        {
            transform.position = TargetPosition();
            FaceHead();
        }
        SetOpenState(true);
        closeButton.SetHoverColors(Danger, Color.white);
        resetButton.SetHoverColors(Danger, Color.white);
        Rebuild();
        Animate(true);
    }

    public void Close()
    {
        if (!open)
            return;
        open = false;
        SaveIfNeeded();
        Animate(false);
        SetOpenState(false);
    }

    static void SetOpenState(bool value)
    {
        if (AnyOpen == value)
            return;
        AnyOpen = value;
        OpenStateChanged?.Invoke(value);
    }

    void OnDestroy()
    {
        if (open)
            SetOpenState(false);
        if (dimmer != null)
            Destroy(dimmer);
        if (dimMaterial != null)
            Destroy(dimMaterial);
    }

    void SaveIfNeeded()
    {
        if (!unsaved)
            return;
        unsaved = false;
        AppSettings.Save();
    }

    void Animate(bool show)
    {
        if (fade != null)
            StopCoroutine(fade);
        fade = StartCoroutine(AnimateRoutine(show));
    }

    IEnumerator AnimateRoutine(bool show)
    {
        canvasGroup.interactable = show;
        canvasGroup.blocksRaycasts = show;
        float from = canvasGroup.alpha;
        float to = show ? 1f : 0f;
        float elapsed = 0f;
        while (elapsed < animationTime)
        {
            elapsed += Time.deltaTime;
            float alpha = Mathf.Lerp(from, to, Mathf.Clamp01(elapsed / animationTime));
            canvasGroup.alpha = alpha;
            SetDim(alpha);
            transform.localScale = baseScale * Mathf.Lerp(0.92f, 1f, alpha);
            yield return null;
        }
        canvasGroup.alpha = to;
        SetDim(to);
        transform.localScale = baseScale;
        fade = null;
        if (!show)
            gameObject.SetActive(false);
    }

    // ---------- dim the surroundings ----------

    // A dark sphere around the head, drawn before the UI (SteelArm/Dimmer), faded in and out with the panel.
    void SetDim(float amount)
    {
        if (amount <= 0f || dimAmount <= 0f)
        {
            if (dimmer != null)
                dimmer.SetActive(false);
            return;
        }
        if (dimmer == null && !CreateDimmer())
            return;
        dimmer.SetActive(true);
        dimMaterial.color = new Color(0f, 0f, 0f, dimAmount * amount);
    }

    bool CreateDimmer()
    {
        Camera cam = Camera.main;
        Shader shader = Shader.Find("SteelArm/Dimmer");
        if (cam == null || shader == null)
        {
            if (shader == null && !warnedNoDimmer)
            {
                warnedNoDimmer = true;
                Debug.LogWarning("[steelarm] Shader SteelArm/Dimmer not found; the surroundings are not dimmed.");
            }
            return false;
        }
        dimmer = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        dimmer.name = "SettingsDimmer";
        Destroy(dimmer.GetComponent<Collider>());
        dimmer.transform.SetParent(cam.transform, false);
        // Just outside the near clip plane; ZTest Always, so its size doesn't matter otherwise.
        float radius = Mathf.Clamp(cam.nearClipPlane * 2f, 0.2f, 0.6f);
        dimmer.transform.localScale = Vector3.one * (radius * 2f);
        dimMaterial = new Material(shader);
        var dimRenderer = dimmer.GetComponent<MeshRenderer>();
        dimRenderer.sharedMaterial = dimMaterial;
        dimRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        dimRenderer.receiveShadows = false;
        return true;
    }

    // ---------- status ----------

    void Update()
    {
        if (open && tab == Tab.Advanced && Time.unscaledTime >= nextStatusTime)
        {
            nextStatusTime = Time.unscaledTime + 0.5f;
            UpdateStatus();
        }
    }

    // In front of the eyes; the pitch is limited so the panel never ends up on the floor or the ceiling.
    Vector3 TargetPosition()
    {
        Vector3 flat = new Vector3(head.forward.x, 0f, head.forward.z);
        if (flat.sqrMagnitude < 1e-4f)
            flat = new Vector3(transform.position.x - head.position.x, 0f, transform.position.z - head.position.z);
        if (flat.sqrMagnitude < 1e-4f)
            flat = Vector3.forward;
        flat.Normalize();

        float pitch = Mathf.Clamp(Mathf.Asin(Mathf.Clamp(head.forward.y, -1f, 1f)) * Mathf.Rad2Deg, -30f, 20f);
        Vector3 right = Vector3.Cross(Vector3.up, flat);
        Vector3 direction = Quaternion.AngleAxis(-pitch, right) * flat;
        return head.position + direction * distance + Vector3.up * heightOffset;
    }

    void FaceHead()
    {
        if (head != null)
            transform.rotation = Quaternion.LookRotation(transform.position - head.position, Vector3.up);
    }

    // ---------- content ----------

    void OnSettingsChanged()
    {
        // Changes made elsewhere (reset, the thumbstick, ...) rebuild the page; our own changes refresh it in place.
        if (!applying && open)
            Rebuild();
    }

    // Applies a change made on the panel. save = false while a slider is dragged (saved when the panel closes).
    void Commit(bool save = true, bool rebuild = false)
    {
        applying = true;
        try { AppSettings.NotifyChanged(save); }
        finally { applying = false; }
        if (!save)
            unsaved = true;
        if (rebuild)
            Rebuild();
        else
            foreach (var refresh in refreshers)
                refresh();
    }

    void Rebuild()
    {
        Loc.Set(navTitle, "settings.title");
        Loc.Set(navHint, "settings.hint");
        Loc.Set(resetLabel, "settings.reset");
        BuildTabs();

        Loc.Set(pageTitle, tab == Tab.Advanced ? "page.advanced" : TabKeys[(int)tab]);
        ClearChildren(rowRoot);
        refreshers.Clear();
        rosValue = topicValue = lastValue = null;

        switch (tab)
        {
            case Tab.Scan: BuildScanPage(); break;
            case Tab.Menu: BuildMenuPage(); break;
            case Tab.Display: BuildDisplayPage(); break;
            case Tab.Advanced: BuildAdvancedPage(); break;
            case Tab.Language: BuildLanguagePage(); break;
        }
    }

    void BuildTabs()
    {
        ClearChildren(tabRoot);
        for (int i = 0; i < Tabs.Length; i++)
        {
            Tab t = Tabs[i];
            bool selected = t == tab;
            MenuButtonVisual button = Instantiate(tabTemplate, tabRoot);
            button.gameObject.SetActive(true);
            button.SetLabel(Loc.T(TabKeys[i]));
            Loc.ApplyFont(button.label);
            button.SetNormalColors(selected ? TabSelected : Clear, selected ? Color.white : Muted);
            button.SetHoverColors(selected ? TabSelected : HoverTint, Color.white);
            Transform dot = button.transform.Find("Dot");
            if (dot != null && dot.TryGetComponent(out Image dotImage))
                dotImage.color = selected ? Accent : DotOff;
            button.GetComponent<Button>().onClick.AddListener(() =>
            {
                if (tab == t)
                    return;
                tab = t;
                Rebuild();
            });
        }
    }

    void BuildScanPage()
    {
        AddSegmented("scan.style", "scan.style.desc", Keys("opt.silhouette", "opt.box"),
            () => S.silhouette ? 0 : 1, v => S.silhouette = v == 0);
        AddSegmented("scan.replace", "scan.replace.desc", Keys("opt.inView", "opt.all"),
            () => S.replaceOnlyInView ? 0 : 1, v => S.replaceOnlyInView = v == 0);
        AddToggle("scan.wave", "scan.wave.desc", () => S.scanWave, v => S.scanWave = v);
        AddSlider("scan.waveIntensity", 0f, 1f, 0.05f, () => S.waveIntensity, v => S.waveIntensity = v, v => $"{v * 100f:0}%");
    }

    void BuildMenuPage()
    {
        AddSlider("menu.yaw", 0f, 60f, 1f, () => S.menuYawAngle, v => S.menuYawAngle = v, v => $"{v:0}°");
        AddSlider("menu.pitch", 0f, 45f, 1f, () => S.menuPitchAngle, v => S.menuPitchAngle = v, v => $"{v:0}°");
        AddSlider("menu.distance", 0.3f, 1.2f, 0.05f, () => S.menuDistance, v => S.menuDistance = v, v => $"{v:0.00} m");
        AddToggle("menu.mirror", "menu.mirror.desc", () => S.menuMirror, v => S.menuMirror = v);
    }

    void BuildDisplayPage()
    {
        AddSegmented("display.colors", "display.colors.desc", Keys("opt.greenRed", "opt.blueOrange"),
            () => (int)S.colorScheme, v => S.colorScheme = (AppSettings.ColorScheme)v);
        AddSlider("display.fill", 0f, 1f, 0.05f, () => S.fillAlpha, v => S.fillAlpha = v, v => $"{v * 100f:0}%");
        AddSegmented("display.width", "display.width.desc", Keys("opt.thin", "opt.normal", "opt.thick"),
            () => (int)S.outlineWidth, v => S.outlineWidth = (AppSettings.OutlineWidth)v);
        AddToggle("display.label", "display.label.desc", () => S.showLabel, v => S.showLabel = v);
    }

    void BuildAdvancedPage()
    {
        AddSegmented("adv.scanMode", "adv.scanMode.desc", Keys("opt.button", "opt.live"),
            () => (int)S.scanMode, v => S.scanMode = (AppSettings.ScanMode)v);
        AddToggle("adv.tracking", "adv.tracking.desc", () => S.useTracking, v => S.useTracking = v,
            () => S.scanMode == AppSettings.ScanMode.Live);
        AddToggle("adv.log", "adv.log.desc", () => S.showLog, v => S.showLog = v);
        rosValue = AddInfo("status.ros");
        topicValue = AddInfo("status.topic");
        lastValue = AddInfo("status.last");
        UpdateStatus();
    }

    void BuildLanguagePage()
    {
        // Each language is written in itself, so "한국어" always uses the Korean font.
        AddSegmented("lang.title", "lang.desc", new[] { "한국어", "English" },
            () => (int)S.language, v => S.language = (AppSettings.Language)v,
            rebuildAfterChange: true, koreanFont: new[] { true, false });
    }

    static string[] Keys(params string[] keys)
    {
        var texts = new string[keys.Length];
        for (int i = 0; i < keys.Length; i++)
            texts[i] = Loc.T(keys[i]);
        return texts;
    }

    // ---------- rows ----------

    SettingsRow AddRow(SettingsRow template, string labelKey, string descriptionKey)
    {
        SettingsRow row = Instantiate(template, rowRoot);
        row.gameObject.SetActive(true);
        Loc.Set(row.label, labelKey);
        if (row.description != null)
        {
            if (descriptionKey != null)
                Loc.Set(row.description, descriptionKey);
            else
                row.description.gameObject.SetActive(false);
        }
        return row;
    }

    void AddSegmented(string labelKey, string descriptionKey, string[] options, Func<int> get, Action<int> set,
        bool rebuildAfterChange = false, bool[] koreanFont = null)
    {
        SettingsRow row = AddRow(segmentedRowTemplate, labelKey, descriptionKey);
        var buttons = new List<MenuButtonVisual>();
        for (int i = 0; i < options.Length; i++)
        {
            MenuButtonVisual button = Instantiate(segmentTemplate, row.segmentRoot);
            button.gameObject.SetActive(true);
            button.SetLabel(options[i]);
            Loc.ApplyFont(button.label, koreanFont != null && koreanFont[i]);
            var rect = (RectTransform)button.transform;
            rect.sizeDelta = new Vector2(Mathf.Max(72f, button.label.GetPreferredValues(options[i]).x + 32f), rect.sizeDelta.y);
            int index = i;
            button.GetComponent<Button>().onClick.AddListener(() =>
            {
                if (get() == index)
                    return;
                set(index);
                Commit(true, rebuildAfterChange);
            });
            buttons.Add(button);
        }

        // The label and description use the room the segment buttons leave.
        LayoutRebuilder.ForceRebuildLayoutImmediate(row.segmentRoot);
        float room = row.segmentRoot.rect.width + 30f;
        foreach (TMP_Text text in new[] { row.label, row.description })
        {
            if (text != null)
                text.rectTransform.offsetMax = new Vector2(-room, text.rectTransform.offsetMax.y);
        }

        void Refresh()
        {
            int selected = get();
            for (int i = 0; i < buttons.Count; i++)
            {
                bool on = i == selected;
                buttons[i].SetNormalColors(on ? Accent : Clear, on ? AccentText : Color.white);
                buttons[i].SetHoverColors(on ? Accent : HoverTint, on ? AccentText : Color.white);
            }
        }
        Refresh();
        refreshers.Add(Refresh);
    }

    void AddToggle(string labelKey, string descriptionKey, Func<bool> get, Action<bool> set, Func<bool> available = null)
    {
        SettingsRow row = AddRow(toggleRowTemplate, labelKey, descriptionKey);
        row.switchButton.onClick.AddListener(() =>
        {
            if (available != null && !available())
                return;
            set(!get());
            Commit();
        });

        void Refresh()
        {
            bool on = get();
            bool usable = available == null || available();
            row.switchTrack.color = on ? Accent : SwitchOff;
            row.switchKnobImage.color = on ? AccentText : KnobOff;
            row.switchKnob.anchoredPosition = new Vector2(on ? 12f : -12f, 0f);
            row.switchButton.interactable = usable;
            if (row.group != null)
                row.group.alpha = usable ? 1f : 0.45f;
        }
        Refresh();
        refreshers.Add(Refresh);
    }

    void AddSlider(string labelKey, float min, float max, float step, Func<float> get, Action<float> set, Func<float, string> format)
    {
        SettingsRow row = AddRow(sliderRowTemplate, labelKey, null);
        Slider slider = row.slider;
        slider.minValue = min;
        slider.maxValue = max;
        slider.wholeNumbers = false;
        slider.SetValueWithoutNotify(get());
        row.value.text = format(get());
        Loc.ApplyFont(row.value);
        slider.onValueChanged.AddListener(raw =>
        {
            float value = Mathf.Clamp(Mathf.Round(raw / step) * step, min, max);
            if (Mathf.Approximately(value, get()))
                return;
            set(value);
            Commit(save: false);
        });
        refreshers.Add(() =>
        {
            slider.SetValueWithoutNotify(get());
            row.value.text = format(get());
        });
    }

    TMP_Text AddInfo(string captionKey)
    {
        SettingsRow row = AddRow(infoRowTemplate, captionKey, null);
        Loc.ApplyFont(row.value);
        return row.value;
    }

    void UpdateStatus()
    {
        if (rosValue == null)
            return;

        ROSConnection ros = ROSConnection.GetOrCreateInstance();
        string state = !ros.HasConnectionThread ? Loc.T("status.notStarted")
            : ros.HasConnectionError ? Loc.T("status.error") : Loc.T("status.connected");
        rosValue.text = $"{state} · {ros.RosIPAddress}:{ros.RosPort}";

        YoloBoxVisualizer visualizer = YoloBoxVisualizer.Instance;
        topicValue.text = visualizer != null ? visualizer.ResultSourceDescription : Loc.T("status.none");
        lastValue.text = visualizer != null && visualizer.LastResultTime >= 0f
            ? Loc.F("status.lastValue", Time.time - visualizer.LastResultTime, visualizer.LastResultCount)
            : Loc.T("status.none");
    }

    static void ClearChildren(Transform parent)
    {
        for (int i = parent.childCount - 1; i >= 0; i--)
        {
            GameObject child = parent.GetChild(i).gameObject;
            child.SetActive(false); // leaves the layout right away; destroyed at the end of the frame
            Destroy(child);
        }
    }
}

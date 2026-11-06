using TMPro;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit.UI;

// Builds the settings panel prefab (layout of the "설정창" mock) and the asset that lets SettingsMenuOpener find it.
//   Menu: SteelArm > Build Settings Menu Prefab
//   Output: Assets/Prefabs/SettingsMenu.prefab, Assets/Resources/SteelArmUIAssets.asset (overwritten / updated on every run)
//
// Only the frame and one template per row type are built here; SettingsMenu fills in the tabs and rows at runtime.
public static class SettingsMenuPrefabBuilder
{
    const string PrefabFolder = "Assets/Prefabs";
    const string MenuPath = PrefabFolder + "/SettingsMenu.prefab";
    const string AssetsPath = "Assets/Resources/SteelArmUIAssets.asset";
    const string SpriteFolder = "Assets/MRTemplateAssets/Sprites/UI/";

    const float CanvasScale = 0.0006f; // 1 canvas unit = 0.6 mm -> the panel is about 52 x 36 cm
    const float Width = 860f;
    const float Height = 608f;
    const float NavWidth = 210f;
    const float RowWidth = Width - NavWidth - 64f;

    static readonly Color Panel = new Color(0.07f, 0.078f, 0.094f, 0.93f);
    static readonly Color BackPlate = new Color(1f, 1f, 1f, 0.07f);
    static readonly Color Border = new Color(1f, 1f, 1f, 0.16f);
    static readonly Color Divider = new Color(1f, 1f, 1f, 0.1f);
    static readonly Color RowBack = new Color(1f, 1f, 1f, 0.04f);
    static readonly Color SegmentBack = new Color(1f, 1f, 1f, 0.06f);
    static readonly Color ButtonBack = new Color(1f, 1f, 1f, 0.06f);
    static readonly Color ButtonOutline = new Color(1f, 1f, 1f, 0.25f);
    static readonly Color Muted = new Color(0.67f, 0.69f, 0.73f, 1f);
    static readonly Color ValueText = new Color(0.76f, 0.79f, 0.82f, 1f);
    static readonly Color Accent = new Color(0.29f, 0.87f, 0.5f, 1f);
    static readonly Color AccentText = new Color(0.04f, 0.14f, 0.08f, 1f);
    static readonly Color TrackBack = new Color(1f, 1f, 1f, 0.15f);
    static readonly Color Clear = new Color(1f, 1f, 1f, 0f);

    static Sprite s_Round;          // rounded rectangle, 40 px corners
    static Sprite s_RoundOutline;
    static Sprite s_Pill;           // horizontal pill, 120 px ends
    static Sprite s_Circle;
    static Sprite s_Ring;

    [MenuItem("SteelArm/Build Settings Menu Prefab")]
    public static void Build()
    {
        s_Round = LoadSprite("Round_Radius_10.png");
        s_RoundOutline = LoadSprite("Round_Radius_10_Outline.png");
        s_Pill = LoadSprite("Circle_60x60_Horizontal.png");
        s_Circle = LoadSprite("CircleMask.png");
        s_Ring = LoadSprite("Circle_60x60_Outline.png");

        if (!AssetDatabase.IsValidFolder(PrefabFolder))
            AssetDatabase.CreateFolder("Assets", "Prefabs");
        if (!AssetDatabase.IsValidFolder("Assets/Resources"))
            AssetDatabase.CreateFolder("Assets", "Resources");

        GameObject root = BuildMenu();
        GameObject prefab = PrefabUtility.SaveAsPrefabAsset(root, MenuPath);
        Object.DestroyImmediate(root);

        var assets = AssetDatabase.LoadAssetAtPath<SteelArmUIAssets>(AssetsPath);
        if (assets == null)
        {
            assets = ScriptableObject.CreateInstance<SteelArmUIAssets>();
            AssetDatabase.CreateAsset(assets, AssetsPath);
        }
        assets.settingsMenuPrefab = prefab;
        EditorUtility.SetDirty(assets);
        AssetDatabase.SaveAssets();
        Debug.Log($"[steelarm] Built {MenuPath} and {AssetsPath}. The left controller menu button opens it in play mode (M in the editor).");
    }

    // ---------- frame ----------

    static GameObject BuildMenu()
    {
        var root = new GameObject("SettingsMenu", typeof(RectTransform));
        var rootRect = (RectTransform)root.transform;
        rootRect.sizeDelta = new Vector2(Width, Height);
        rootRect.localScale = Vector3.one * CanvasScale;
        var canvas = root.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.WorldSpace;
        canvas.sortingOrder = 10;
        root.AddComponent<TrackedDeviceGraphicRaycaster>();
        var group = root.AddComponent<CanvasGroup>();
        var menu = root.AddComponent<SettingsMenu>();
        menu.canvasGroup = group;

        // Two layers like the other menus: a faint plate behind, the panel 16 units toward the viewer.
        Image back = NewImage("Back", rootRect, s_Round, BackPlate, 40f / 28f);
        Stretch(back.rectTransform);
        back.rectTransform.anchoredPosition = new Vector2(0f, -12f);

        RectTransform panel = NewRect("Panel", rootRect);
        Stretch(panel);
        SetZ(panel, -16f);
        Image plate = NewImage("Plate", panel, s_Round, Panel, 40f / 28f);
        Stretch(plate.rectTransform);
        plate.raycastTarget = true; // pointing at the panel counts as pointing at the UI (no object behind it is picked)
        Stretch(NewImage("Outline", panel, s_RoundOutline, Border, 42f / 28f).rectTransform);

        // ----- left: title, tabs, hint -----
        RectTransform nav = NewRect("Nav", panel);
        SetRect(nav, new Vector2(0f, 0f), new Vector2(0f, 1f), new Vector2(0f, 0f), new Vector2(NavWidth, 0f));
        Image divider = NewImage("Divider", nav, null, Divider);
        SetRect(divider.rectTransform, new Vector2(1f, 0f), new Vector2(1f, 1f), new Vector2(-1f, 0f), new Vector2(0f, 0f));

        menu.navTitle = NewText("Title", nav, "설정", 22f, Color.white, FontStyles.Bold, TextAlignmentOptions.Left);
        TopStrip(menu.navTitle.rectTransform, 28f, 28f, 28f, 34f);

        RectTransform tabs = NewRect("Tabs", nav);
        TopStrip(tabs, 80f, 16f, 16f, 5 * 48f + 4 * 6f);
        var tabLayout = tabs.gameObject.AddComponent<VerticalLayoutGroup>();
        tabLayout.spacing = 6f;
        tabLayout.childControlWidth = true;
        tabLayout.childControlHeight = false;
        tabLayout.childForceExpandWidth = true;
        tabLayout.childForceExpandHeight = false;
        menu.tabRoot = tabs;

        menu.navHint = NewText("Hint", nav, "", 12f, Muted, FontStyles.Normal, TextAlignmentOptions.BottomLeft);
        menu.navHint.textWrappingMode = TextWrappingModes.Normal;
        SetRect(menu.navHint.rectTransform, Vector2.zero, new Vector2(1f, 0f), new Vector2(28f, 28f), new Vector2(-16f, 72f));

        // ----- right: page title, close, rows, reset -----
        RectTransform main = NewRect("Main", panel);
        SetRect(main, Vector2.zero, Vector2.one, new Vector2(NavWidth, 0f), Vector2.zero);

        menu.pageTitle = NewText("PageTitle", main, "", 20f, Color.white, FontStyles.Bold, TextAlignmentOptions.Left);
        TopStrip(menu.pageTitle.rectTransform, 28f, 32f, 88f, 44f);

        menu.closeButton = BuildCloseButton(main);

        RectTransform rows = NewRect("Rows", main);
        SetRect(rows, Vector2.zero, Vector2.one, new Vector2(32f, 82f), new Vector2(-32f, -90f));
        var rowLayout = rows.gameObject.AddComponent<VerticalLayoutGroup>();
        rowLayout.spacing = 10f;
        rowLayout.childControlWidth = true;
        rowLayout.childControlHeight = false;
        rowLayout.childForceExpandWidth = true;
        rowLayout.childForceExpandHeight = false;
        menu.rowRoot = rows;

        menu.resetButton = BuildResetButton(main, out menu.resetLabel);

        BuildGrabBar(rootRect);

        // ----- templates (inactive; SettingsMenu copies them) -----
        RectTransform templates = NewRect("Templates", main);
        Stretch(templates);
        menu.tabTemplate = BuildTab(templates);
        menu.segmentTemplate = BuildSegment(templates);
        menu.segmentedRowTemplate = BuildSegmentedRow(templates);
        menu.toggleRowTemplate = BuildToggleRow(templates);
        menu.sliderRowTemplate = BuildSliderRow(templates);
        menu.infoRowTemplate = BuildInfoRow(templates);
        foreach (Transform template in templates)
            template.gameObject.SetActive(false);

        return root;
    }

    static MenuButtonVisual BuildCloseButton(RectTransform parent)
    {
        MenuButtonVisual button = NewButton("CloseButton", parent, s_Circle, 0f, ButtonBack, s_Ring, 0f, ButtonOutline);
        var rect = (RectTransform)button.transform;
        rect.anchorMin = rect.anchorMax = new Vector2(1f, 1f);
        rect.pivot = new Vector2(1f, 1f);
        rect.anchoredPosition = new Vector2(-32f, -28f);
        rect.sizeDelta = new Vector2(44f, 44f);

        var content = new Graphic[2];
        for (int i = 0; i < 2; i++)
        {
            Image bar = NewImage("X" + (i + 1), rect, null, Color.white);
            Center(bar.rectTransform, Vector2.zero, new Vector2(18f, 2.5f));
            bar.rectTransform.localRotation = Quaternion.Euler(0f, 0f, i == 0 ? 45f : -45f);
            content[i] = bar;
        }
        button.content = content;
        return button;
    }

    // Bar under the panel for carrying it (SettingsGrabBar). The hit area is much larger than the visible bar.
    static void BuildGrabBar(RectTransform root)
    {
        RectTransform area = NewRect("GrabBar", root);
        area.anchorMin = area.anchorMax = new Vector2(0.5f, 0f);
        area.anchoredPosition = new Vector2(0f, -34f);
        area.sizeDelta = new Vector2(280f, 44f);
        SetZ(area, -16f);
        Image hit = area.gameObject.AddComponent<Image>();
        hit.color = Clear;
        hit.raycastTarget = true;

        Image bar = NewImage("Bar", area, s_Pill, new Color(1f, 1f, 1f, 0.35f));
        bar.type = Image.Type.Sliced;
        bar.pixelsPerUnitMultiplier = 120f / 5f;
        Center(bar.rectTransform, Vector2.zero, new Vector2(160f, 10f));

        var grab = area.gameObject.AddComponent<SettingsGrabBar>();
        grab.panel = root;
        grab.bar = bar;
    }

    static MenuButtonVisual BuildResetButton(RectTransform parent, out TMP_Text label)
    {
        MenuButtonVisual button = NewButton("ResetButton", parent, s_Round, 40f / 14f, ButtonBack, s_RoundOutline, 42f / 14f, ButtonOutline);
        var rect = (RectTransform)button.transform;
        rect.anchorMin = rect.anchorMax = Vector2.zero;
        rect.pivot = Vector2.zero;
        rect.anchoredPosition = new Vector2(32f, 24f);
        rect.sizeDelta = new Vector2(150f, 44f);
        label = NewText("Label", rect, "설정 초기화", 14f, Color.white, FontStyles.Normal, TextAlignmentOptions.Center);
        Stretch(label.rectTransform);
        button.label = label;
        button.content = new Graphic[] { label };
        return button;
    }

    // ---------- templates ----------

    // Tab: dot + label; the selected tab gets a tinted plate (colors set by SettingsMenu).
    static MenuButtonVisual BuildTab(RectTransform parent)
    {
        MenuButtonVisual button = NewButton("TabTemplate", parent, s_Round, 40f / 14f, Clear, null, 0f, Clear);
        ((RectTransform)button.transform).sizeDelta = new Vector2(NavWidth - 32f, 48f);
        Image dot = NewImage("Dot", (RectTransform)button.transform, s_Circle, Color.white);
        SetRect(dot.rectTransform, new Vector2(0f, 0.5f), new Vector2(0f, 0.5f), new Vector2(14f, -4f), new Vector2(22f, 4f));
        TMP_Text label = NewText("Label", (RectTransform)button.transform, "Tab", 15f, Color.white, FontStyles.Normal, TextAlignmentOptions.Left);
        SetRect(label.rectTransform, Vector2.zero, Vector2.one, new Vector2(34f, 0f), new Vector2(-8f, 0f));
        button.label = label;
        button.content = new Graphic[] { label };
        return button;
    }

    static MenuButtonVisual BuildSegment(RectTransform parent)
    {
        MenuButtonVisual button = NewButton("SegmentTemplate", parent, s_Round, 40f / 9f, Clear, null, 0f, Clear);
        ((RectTransform)button.transform).sizeDelta = new Vector2(96f, 40f);
        TMP_Text label = NewText("Label", (RectTransform)button.transform, "Option", 14f, Color.white, FontStyles.Normal, TextAlignmentOptions.Center);
        Stretch(label.rectTransform);
        button.label = label;
        button.content = new Graphic[] { label };
        return button;
    }

    // Label + description on the left, segment buttons on the right.
    static SettingsRow BuildSegmentedRow(RectTransform parent)
    {
        SettingsRow row = NewRow("SegmentedRowTemplate", parent, 64f);
        AddLabelAndDescription(row);

        RectTransform segments = NewRect("Segments", (RectTransform)row.transform);
        segments.anchorMin = segments.anchorMax = new Vector2(1f, 0.5f);
        segments.pivot = new Vector2(1f, 0.5f);
        segments.anchoredPosition = new Vector2(-14f, 0f);
        segments.sizeDelta = new Vector2(200f, 48f);
        Image back = segments.gameObject.AddComponent<Image>();
        back.sprite = s_Round;
        back.type = Image.Type.Sliced;
        back.pixelsPerUnitMultiplier = 40f / 12f;
        back.color = SegmentBack;
        back.raycastTarget = false;
        var layout = segments.gameObject.AddComponent<HorizontalLayoutGroup>();
        layout.padding = new RectOffset(4, 4, 4, 4);
        layout.spacing = 4f;
        layout.childControlWidth = false;
        layout.childControlHeight = false;
        layout.childForceExpandWidth = false;
        layout.childForceExpandHeight = false;
        layout.childAlignment = TextAnchor.MiddleCenter;
        var fitter = segments.gameObject.AddComponent<ContentSizeFitter>();
        fitter.horizontalFit = ContentSizeFitter.FitMode.PreferredSize;
        row.segmentRoot = segments;
        return row;
    }

    // Label + description on the left, an on/off switch on the right.
    static SettingsRow BuildToggleRow(RectTransform parent)
    {
        SettingsRow row = NewRow("ToggleRowTemplate", parent, 64f);
        AddLabelAndDescription(row);
        row.group = row.gameObject.AddComponent<CanvasGroup>();

        RectTransform switchRect = NewRect("Switch", (RectTransform)row.transform);
        switchRect.anchorMin = switchRect.anchorMax = new Vector2(1f, 0.5f);
        switchRect.pivot = new Vector2(1f, 0.5f);
        switchRect.anchoredPosition = new Vector2(-18f, 0f);
        switchRect.sizeDelta = new Vector2(56f, 32f);
        Image track = switchRect.gameObject.AddComponent<Image>();
        track.sprite = s_Pill;
        track.type = Image.Type.Sliced;
        track.pixelsPerUnitMultiplier = 120f / 16f;
        track.color = Accent;
        var button = switchRect.gameObject.AddComponent<Button>();
        PlainButton(button, track);

        Image knob = NewImage("Knob", switchRect, s_Circle, AccentText);
        Center(knob.rectTransform, new Vector2(12f, 0f), new Vector2(24f, 24f));

        row.switchButton = button;
        row.switchTrack = track;
        row.switchKnob = knob.rectTransform;
        row.switchKnobImage = knob;
        return row;
    }

    // Label and value on top, the slider below.
    static SettingsRow BuildSliderRow(RectTransform parent)
    {
        SettingsRow row = NewRow("SliderRowTemplate", parent, 80f);
        var rect = (RectTransform)row.transform;
        row.label = NewText("Label", rect, "Label", 15f, Color.white, FontStyles.Normal, TextAlignmentOptions.Left);
        TopStrip(row.label.rectTransform, 14f, 18f, 140f, 24f);
        row.value = NewText("Value", rect, "0", 14f, ValueText, FontStyles.Normal, TextAlignmentOptions.Right);
        row.value.rectTransform.anchorMin = row.value.rectTransform.anchorMax = new Vector2(1f, 1f);
        row.value.rectTransform.pivot = new Vector2(1f, 1f);
        row.value.rectTransform.anchoredPosition = new Vector2(-18f, -14f);
        row.value.rectTransform.sizeDelta = new Vector2(130f, 24f);

        RectTransform sliderRect = NewRect("Slider", rect);
        SetRect(sliderRect, Vector2.zero, new Vector2(1f, 0f), new Vector2(18f, 10f), new Vector2(-18f, 40f));
        var slider = sliderRect.gameObject.AddComponent<Slider>();

        Image background = NewImage("Background", sliderRect, s_Pill, TrackBack);
        background.type = Image.Type.Sliced;
        background.pixelsPerUnitMultiplier = 120f / 3f;
        SetRect(background.rectTransform, new Vector2(0f, 0.5f), new Vector2(1f, 0.5f), new Vector2(0f, -3f), new Vector2(0f, 3f));
        background.raycastTarget = true;

        // The track itself is thin; a wider transparent strip makes it easy to hit with the ray.
        Image hitArea = NewImage("HitArea", sliderRect, null, Clear);
        Stretch(hitArea.rectTransform);
        hitArea.raycastTarget = true;

        RectTransform fillArea = NewRect("Fill Area", sliderRect);
        SetRect(fillArea, new Vector2(0f, 0.5f), new Vector2(1f, 0.5f), new Vector2(0f, -3f), new Vector2(0f, 3f));
        Image fill = NewImage("Fill", fillArea, s_Pill, Accent);
        fill.type = Image.Type.Sliced;
        fill.pixelsPerUnitMultiplier = 120f / 3f;
        Stretch(fill.rectTransform);

        // The slider stretches the handle over the slide area's height, so the area is exactly as tall as the
        // handle is wide (22): the handle stays round.
        RectTransform handleArea = NewRect("Handle Slide Area", sliderRect);
        SetRect(handleArea, new Vector2(0f, 0.5f), new Vector2(1f, 0.5f), new Vector2(11f, -11f), new Vector2(-11f, 11f));
        Image handle = NewImage("Handle", handleArea, s_Circle, Accent);
        handle.raycastTarget = true;
        handle.rectTransform.anchorMin = new Vector2(0f, 0f);
        handle.rectTransform.anchorMax = new Vector2(0f, 1f);
        handle.rectTransform.sizeDelta = new Vector2(22f, 0f);
        Image handleDot = NewImage("Center", handle.rectTransform, s_Circle, Panel);
        Center(handleDot.rectTransform, Vector2.zero, new Vector2(10f, 10f));

        slider.fillRect = fill.rectTransform;
        slider.handleRect = handle.rectTransform;
        slider.targetGraphic = handle;
        slider.direction = Slider.Direction.LeftToRight;
        slider.transition = Selectable.Transition.None;
        var navigation = slider.navigation;
        navigation.mode = Navigation.Mode.None;
        slider.navigation = navigation;
        row.slider = slider;
        return row;
    }

    // Status line: caption on the left, value on the right.
    static SettingsRow BuildInfoRow(RectTransform parent)
    {
        SettingsRow row = NewRow("InfoRowTemplate", parent, 40f);
        var rect = (RectTransform)row.transform;
        row.label = NewText("Caption", rect, "Caption", 13f, Muted, FontStyles.Normal, TextAlignmentOptions.Left);
        SetRect(row.label.rectTransform, Vector2.zero, new Vector2(0f, 1f), new Vector2(18f, 0f), new Vector2(180f, 0f));
        row.value = NewText("Value", rect, "-", 15f, Color.white, FontStyles.Normal, TextAlignmentOptions.Right);
        SetRect(row.value.rectTransform, Vector2.zero, Vector2.one, new Vector2(180f, 0f), new Vector2(-18f, 0f));
        return row;
    }

    static SettingsRow NewRow(string name, RectTransform parent, float height)
    {
        RectTransform rect = NewRect(name, parent);
        rect.sizeDelta = new Vector2(RowWidth, height);
        Image back = rect.gameObject.AddComponent<Image>();
        back.sprite = s_Round;
        back.type = Image.Type.Sliced;
        back.pixelsPerUnitMultiplier = 40f / 16f;
        back.color = RowBack;
        back.raycastTarget = false;
        return rect.gameObject.AddComponent<SettingsRow>();
    }

    static void AddLabelAndDescription(SettingsRow row)
    {
        var rect = (RectTransform)row.transform;
        row.label = NewText("Label", rect, "Label", 15f, Color.white, FontStyles.Normal, TextAlignmentOptions.BottomLeft);
        SetRect(row.label.rectTransform, new Vector2(0f, 0.5f), new Vector2(1f, 1f), new Vector2(18f, 1f), new Vector2(-260f, -8f));
        row.description = NewText("Description", rect, "Description", 12f, Muted, FontStyles.Normal, TextAlignmentOptions.TopLeft);
        SetRect(row.description.rectTransform, Vector2.zero, new Vector2(1f, 0.5f), new Vector2(18f, 8f), new Vector2(-260f, -3f));
    }

    // ---------- helpers ----------

    // Transparent hit area + Button + MenuButtonVisual with a flat front plate (and an optional outline).
    static MenuButtonVisual NewButton(string name, RectTransform parent, Sprite plateSprite, float plateMultiplier, Color plateColor,
        Sprite outlineSprite, float outlineMultiplier, Color outlineColor)
    {
        RectTransform rect = NewRect(name, parent);
        Image hit = rect.gameObject.AddComponent<Image>();
        hit.sprite = plateSprite;
        hit.color = Clear;
        if (plateMultiplier > 0f)
        {
            hit.type = Image.Type.Sliced;
            hit.pixelsPerUnitMultiplier = plateMultiplier;
        }
        var button = rect.gameObject.AddComponent<Button>();
        PlainButton(button, hit);

        Image plate = NewImage("Plate", rect, plateSprite, plateColor, plateMultiplier);
        Stretch(plate.rectTransform);
        var visual = rect.gameObject.AddComponent<MenuButtonVisual>();
        visual.front = plate;
        visual.normalColor = plateColor;
        visual.hoverColor = new Color(1f, 1f, 1f, 0.12f);
        visual.contentHoverColor = Color.white;
        visual.raisedZ = 0f;
        visual.pressedZ = 0f;
        if (outlineSprite != null)
        {
            Image outline = NewImage("Outline", rect, outlineSprite, outlineColor, outlineMultiplier);
            Stretch(outline.rectTransform);
            visual.outline = outline;
            visual.outlineColor = outlineColor;
        }
        return visual;
    }

    static void PlainButton(Button button, Graphic target)
    {
        button.transition = Selectable.Transition.None;
        button.targetGraphic = target;
        var navigation = button.navigation;
        navigation.mode = Navigation.Mode.None;
        button.navigation = navigation;
    }

    static RectTransform NewRect(string name, Transform parent)
    {
        var go = new GameObject(name, typeof(RectTransform));
        var rect = (RectTransform)go.transform;
        rect.SetParent(parent, false);
        return rect;
    }

    static Image NewImage(string name, RectTransform parent, Sprite sprite, Color color, float slicedMultiplier = 0f)
    {
        RectTransform rect = NewRect(name, parent);
        var image = rect.gameObject.AddComponent<Image>();
        image.sprite = sprite;
        image.color = color;
        image.raycastTarget = false;
        if (sprite != null && slicedMultiplier > 0f)
        {
            image.type = Image.Type.Sliced;
            image.pixelsPerUnitMultiplier = slicedMultiplier;
        }
        return image;
    }

    static TMP_Text NewText(string name, RectTransform parent, string text, float size, Color color, FontStyles style, TextAlignmentOptions alignment)
    {
        RectTransform rect = NewRect(name, parent);
        var tmp = rect.gameObject.AddComponent<TextMeshProUGUI>();
        if (TMP_Settings.defaultFontAsset != null)
            tmp.font = TMP_Settings.defaultFontAsset;
        tmp.text = text;
        tmp.fontSize = size;
        tmp.color = color;
        tmp.fontStyle = style;
        tmp.alignment = alignment;
        tmp.textWrappingMode = TextWrappingModes.NoWrap;
        tmp.overflowMode = TextOverflowModes.Ellipsis;
        tmp.raycastTarget = false;
        return tmp;
    }

    static void Stretch(RectTransform rect)
    {
        rect.anchorMin = Vector2.zero;
        rect.anchorMax = Vector2.one;
        rect.offsetMin = Vector2.zero;
        rect.offsetMax = Vector2.zero;
    }

    static void SetRect(RectTransform rect, Vector2 anchorMin, Vector2 anchorMax, Vector2 offsetMin, Vector2 offsetMax)
    {
        rect.anchorMin = anchorMin;
        rect.anchorMax = anchorMax;
        rect.offsetMin = offsetMin;
        rect.offsetMax = offsetMax;
    }

    // A strip along the top edge: `top` below it, inset `left` / `right`, `height` tall.
    static void TopStrip(RectTransform rect, float top, float left, float right, float height)
    {
        SetRect(rect, new Vector2(0f, 1f), Vector2.one, new Vector2(left, -top - height), new Vector2(-right, -top));
    }

    static void Center(RectTransform rect, Vector2 position, Vector2 size)
    {
        rect.anchorMin = rect.anchorMax = new Vector2(0.5f, 0.5f);
        rect.anchoredPosition = position;
        rect.sizeDelta = size;
    }

    static void SetZ(RectTransform rect, float z)
    {
        Vector3 p = rect.localPosition;
        p.z = z;
        rect.localPosition = p;
    }

    static Sprite LoadSprite(string file)
    {
        var sprite = AssetDatabase.LoadAssetAtPath<Sprite>(SpriteFolder + file);
        if (sprite == null)
            Debug.LogWarning($"[steelarm] Sprite not found: {SpriteFolder + file} (the settings menu will use plain squares).");
        return sprite;
    }
}

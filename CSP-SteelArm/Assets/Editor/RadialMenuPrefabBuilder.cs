using TMPro;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit.UI;

// Builds the radial object menu prefabs and assigns the menu to the scene's ObjectMenuGenerator.
//   Menu: SteelArm > Build Radial Menu Prefabs
//   Output: Assets/Prefabs/RadialMenu.prefab, Assets/Prefabs/RadialMenuButton.prefab (overwritten on every run)
//
// Buttons use the two-layer look of the MR template's CoachingUI buttons: a faint back plate and a raised front plate
// (front 20 canvas units toward the viewer) with an outline ring.
public static class RadialMenuPrefabBuilder
{
    const string PrefabFolder = "Assets/Prefabs";
    const string ButtonPath = PrefabFolder + "/RadialMenuButton.prefab";
    const string MenuPath = PrefabFolder + "/RadialMenu.prefab";
    const string SpriteFolder = "Assets/MRTemplateAssets/Sprites/UI/";

    const float CanvasScale = 0.0008f; // 1 canvas unit = 0.8 mm
    const float CanvasSize = 420f;
    const float ButtonSize = 84f;
    const float HubSize = 128f;
    const float CloseSize = 46f;

    static readonly Color Panel = new Color(0.07f, 0.078f, 0.094f, 0.9f);
    static readonly Color BackPlate = new Color(1f, 1f, 1f, 0.12f);
    static readonly Color Outline = new Color(1f, 1f, 1f, 0.3f);
    static readonly Color Secondary = new Color(0.76f, 0.79f, 0.82f, 1f);
    static readonly Color DarkText = new Color(0.04f, 0.1f, 0.06f, 1f);

    static Sprite s_Circle;
    static Sprite s_Ring;
    static Sprite s_Pill;

    [MenuItem("SteelArm/Build Radial Menu Prefabs")]
    public static void Build()
    {
        s_Circle = LoadSprite("CircleMask.png");
        s_Ring = LoadSprite("Circle_60x60_Outline.png");
        s_Pill = LoadSprite("Circle_60x60_Horizontal.png");

        if (!AssetDatabase.IsValidFolder(PrefabFolder))
            AssetDatabase.CreateFolder("Assets", "Prefabs");

        GameObject buttonAsset = SaveAndDestroy(BuildButton("RadialMenuButton", ButtonSize, true), ButtonPath);
        GameObject menuAsset = SaveAndDestroy(BuildMenu(buttonAsset.GetComponent<MenuButtonVisual>()), MenuPath);
        AssignToScene(menuAsset);
        Debug.Log($"[steelarm] Built {MenuPath} and {ButtonPath}.");
    }

    // ---------- menu ----------

    static GameObject BuildMenu(MenuButtonVisual buttonPrefab)
    {
        var root = new GameObject("RadialMenu", typeof(RectTransform));
        var rootRect = (RectTransform)root.transform;
        rootRect.sizeDelta = new Vector2(CanvasSize, CanvasSize);
        rootRect.localScale = Vector3.one * CanvasScale;
        var canvas = root.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.WorldSpace;
        root.AddComponent<TrackedDeviceGraphicRaycaster>();
        var menu = root.AddComponent<RadialMenu>();

        // Center: two layers like the buttons, with name / confidence / status.
        RectTransform hub = NewRect("Hub", root.transform, HubSize);
        Image hubBack = NewImage("Back", hub, s_Circle, BackPlate);
        Stretch(hubBack.rectTransform, -4f);
        RectTransform hubFront = NewRect("Front", hub, 0f);
        Stretch(hubFront);
        SetZ(hubFront, -12f);
        Stretch(NewImage("Plate", hubFront, s_Circle, Panel).rectTransform);
        Stretch(NewImage("Outline", hubFront, s_Ring, Outline).rectTransform);

        TMP_Text nameText = NewText("Name", hubFront, "object", 26f, Color.white, FontStyles.Bold);
        Place(nameText.rectTransform, new Vector2(0f, 18f), new Vector2(118f, 34f));
        TMP_Text infoText = NewText("Info", hubFront, "Confidence 0.00", 13f, Secondary, FontStyles.Normal);
        Place(infoText.rectTransform, new Vector2(0f, -10f), new Vector2(118f, 20f));

        Image chip = NewImage("Status", hubFront, s_Pill, Color.green);
        chip.type = Image.Type.Sliced;
        chip.pixelsPerUnitMultiplier = 4f;
        Place(chip.rectTransform, new Vector2(0f, -36f), new Vector2(92f, 22f));
        TMP_Text statusText = NewText("Text", chip.rectTransform, "0 actions", 12f, DarkText, FontStyles.Bold);
        Stretch(statusText.rectTransform);

        RectTransform buttonRoot = NewRect("Buttons", root.transform, 0f);

        GameObject close = BuildButton("CloseButton", CloseSize, false);
        close.transform.SetParent(root.transform, false);

        LineRenderer connector = NewLine("Connector", root.transform);
        LineRenderer anchorDot = NewLine("AnchorDot", root.transform);

        menu.hub = hub;
        menu.nameText = nameText;
        menu.infoText = infoText;
        menu.statusText = statusText;
        menu.buttonRoot = buttonRoot;
        menu.actionButtonPrefab = buttonPrefab;
        menu.closeButton = close.GetComponent<MenuButtonVisual>();
        menu.connector = connector;
        menu.anchorDot = anchorDot;
        menu.hubRadius = HubSize * 0.5f;
        return root;
    }

    // ---------- button ----------

    // Root: transparent circle that receives the ray + Button.
    // Children: Back plate, and the raised Front (plate, outline, label or X icon).
    static GameObject BuildButton(string name, float size, bool withLabel)
    {
        var root = new GameObject(name, typeof(RectTransform));
        var rootRect = (RectTransform)root.transform;
        rootRect.sizeDelta = new Vector2(size, size);

        var hitArea = root.AddComponent<Image>();
        hitArea.sprite = s_Circle;
        hitArea.color = new Color(1f, 1f, 1f, 0f);
        var button = root.AddComponent<Button>();
        button.transition = Selectable.Transition.None;
        button.targetGraphic = hitArea;
        var navigation = button.navigation;
        navigation.mode = Navigation.Mode.None;
        button.navigation = navigation;

        Stretch(NewImage("Back", rootRect, s_Circle, BackPlate).rectTransform);

        RectTransform frontPlate = NewRect("Front", rootRect, 0f);
        Stretch(frontPlate);
        SetZ(frontPlate, -20f);
        Image front = NewImage("Plate", frontPlate, s_Circle, Panel);
        Stretch(front.rectTransform);
        Image outline = NewImage("Outline", frontPlate, s_Ring, Outline);
        Stretch(outline.rectTransform);

        var visual = root.AddComponent<MenuButtonVisual>();
        visual.front = front;
        visual.frontPlate = frontPlate;
        visual.outline = outline;

        if (withLabel)
        {
            TMP_Text label = NewText("Label", frontPlate, "Action", 15f, Color.white, FontStyles.Bold);
            Stretch(label.rectTransform);
            visual.label = label;
            visual.content = new Graphic[] { label };
        }
        else
        {
            // X icon: two thin bars.
            Image barA = NewImage("X1", frontPlate, null, Color.white);
            Place(barA.rectTransform, Vector2.zero, new Vector2(size * 0.42f, 3f));
            barA.rectTransform.localRotation = Quaternion.Euler(0f, 0f, 45f);
            Image barB = NewImage("X2", frontPlate, null, Color.white);
            Place(barB.rectTransform, Vector2.zero, new Vector2(size * 0.42f, 3f));
            barB.rectTransform.localRotation = Quaternion.Euler(0f, 0f, -45f);
            visual.content = new Graphic[] { barA, barB };
        }
        return root;
    }

    // ---------- helpers ----------

    static RectTransform NewRect(string name, Transform parent, float size)
    {
        var go = new GameObject(name, typeof(RectTransform));
        var rect = (RectTransform)go.transform;
        rect.SetParent(parent, false);
        rect.sizeDelta = new Vector2(size, size);
        return rect;
    }

    static Image NewImage(string name, RectTransform parent, Sprite sprite, Color color)
    {
        RectTransform rect = NewRect(name, parent, 0f);
        var image = rect.gameObject.AddComponent<Image>();
        image.sprite = sprite;
        image.color = color;
        image.raycastTarget = false;
        return image;
    }

    static TMP_Text NewText(string name, RectTransform parent, string text, float size, Color color, FontStyles style)
    {
        RectTransform rect = NewRect(name, parent, 0f);
        var tmp = rect.gameObject.AddComponent<TextMeshProUGUI>();
        if (TMP_Settings.defaultFontAsset != null)
            tmp.font = TMP_Settings.defaultFontAsset;
        tmp.text = text;
        tmp.fontSize = size;
        tmp.color = color;
        tmp.fontStyle = style;
        tmp.alignment = TextAlignmentOptions.Center;
        tmp.textWrappingMode = TextWrappingModes.NoWrap;
        tmp.overflowMode = TextOverflowModes.Ellipsis;
        tmp.raycastTarget = false;
        return tmp;
    }

    static LineRenderer NewLine(string name, Transform parent)
    {
        var go = new GameObject(name);
        go.transform.SetParent(parent, false);
        var line = go.AddComponent<LineRenderer>();
        line.useWorldSpace = true;
        line.positionCount = 0;
        line.widthMultiplier = 0.003f;
        line.numCapVertices = 4;
        line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        line.receiveShadows = false;
        return line;
    }

    static void Stretch(RectTransform rect, float inset = 0f)
    {
        rect.anchorMin = Vector2.zero;
        rect.anchorMax = Vector2.one;
        rect.offsetMin = new Vector2(inset, inset);
        rect.offsetMax = new Vector2(-inset, -inset);
    }

    static void Place(RectTransform rect, Vector2 position, Vector2 size)
    {
        rect.anchorMin = new Vector2(0.5f, 0.5f);
        rect.anchorMax = new Vector2(0.5f, 0.5f);
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
            Debug.LogWarning($"[steelarm] Sprite not found: {SpriteFolder + file} (the menu will use plain squares).");
        return sprite;
    }

    static GameObject SaveAndDestroy(GameObject go, string path)
    {
        GameObject asset = PrefabUtility.SaveAsPrefabAsset(go, path);
        Object.DestroyImmediate(go);
        return asset;
    }

    static void AssignToScene(GameObject menuPrefab)
    {
        var generator = Object.FindFirstObjectByType<ObjectMenuGenerator>(FindObjectsInactive.Include);
        if (generator == null)
        {
            Debug.LogWarning("[steelarm] No ObjectMenuGenerator in the open scene. Assign Assets/Prefabs/RadialMenu.prefab to its UI Prefab manually.");
            return;
        }
        Undo.RecordObject(generator, "Assign radial menu prefab");
        generator._uiPrefab = menuPrefab;
        EditorUtility.SetDirty(generator);
        EditorSceneManager.MarkSceneDirty(generator.gameObject.scene);
        Debug.Log($"[steelarm] Assigned {MenuPath} to {generator.name}. Save the scene to keep it.");
    }
}

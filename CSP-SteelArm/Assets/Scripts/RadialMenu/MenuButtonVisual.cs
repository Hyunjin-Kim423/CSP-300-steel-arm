using TMPro;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

// Look of one two-layer menu button (like the CoachingUI buttons): a faint back plate and a raised front plate.
//   hover -> the front plate and its outline take the hover color
//   press -> the front plate sinks toward the back plate
[RequireComponent(typeof(Button))]
public class MenuButtonVisual : MonoBehaviour, IPointerEnterHandler, IPointerExitHandler, IPointerDownHandler, IPointerUpHandler
{
    [Header("Parts")]
    public Graphic front;
    public RectTransform frontPlate;
    public Graphic outline;
    public TMP_Text label;
    [Tooltip("Label / icon graphics that change color with the front plate.")]
    public Graphic[] content;

    [Header("Colors")]
    public Color normalColor = new Color(0.07f, 0.078f, 0.094f, 0.9f);
    public Color hoverColor = Color.white;
    public Color outlineColor = new Color(1f, 1f, 1f, 0.3f);
    public Color contentColor = Color.white;
    public Color contentHoverColor = new Color(0.04f, 0.1f, 0.06f, 1f);

    [Header("Depth (canvas units, negative = toward the viewer)")]
    public float raisedZ = -20f;
    public float pressedZ = -6f;

    bool hovered;
    bool pressed;

    public Button Button { get; private set; }

    void Awake()
    {
        Button = GetComponent<Button>();
        Apply();
    }

    void OnDisable()
    {
        hovered = false;
        pressed = false;
        Apply();
    }

    public void SetLabel(string text)
    {
        if (label != null)
            label.text = text;
    }

    public void SetNormalColors(Color normal, Color contentNormal)
    {
        normalColor = normal;
        contentColor = contentNormal;
        Apply();
    }

    public void SetHoverColors(Color hover, Color contentOnHover)
    {
        hoverColor = hover;
        contentHoverColor = contentOnHover;
        Apply();
    }

    public void OnPointerEnter(PointerEventData eventData) { hovered = true; Apply(); }
    public void OnPointerExit(PointerEventData eventData) { hovered = false; pressed = false; Apply(); }
    public void OnPointerDown(PointerEventData eventData) { pressed = true; Apply(); }
    public void OnPointerUp(PointerEventData eventData) { pressed = false; Apply(); }

    void Apply()
    {
        bool hot = hovered && (Button == null || Button.interactable);
        if (front != null)
            front.color = hot ? hoverColor : normalColor;
        if (outline != null)
            outline.color = hot ? hoverColor : outlineColor;
        if (content != null)
        {
            foreach (var graphic in content)
            {
                if (graphic != null)
                    graphic.color = hot ? contentHoverColor : contentColor;
            }
        }
        if (frontPlate != null)
        {
            Vector3 p = frontPlate.localPosition;
            p.z = hot && pressed ? pressedZ : raisedZ;
            frontPlate.localPosition = p;
        }
    }
}

using System;
using System.Collections.Generic;
using UnityEngine;

// Keeps the drawn objects, keyed by DetectedObject.key, and reuses their GameObjects (line + fill) instead of
// creating and destroying them for every YOLO result.
public class DetectedObjectPool
{
    readonly Material lineMaterial;
    readonly Material fillMaterial;
    Color outlineColor;
    float lineWidth;

    readonly Dictionary<string, DetectedObject> active = new Dictionary<string, DetectedObject>();
    readonly Stack<DetectedObject> free = new Stack<DetectedObject>();
    readonly List<DetectedObject> scratch = new List<DetectedObject>();

    public DetectedObjectPool(Material lineMaterial, Material fillMaterial, Color outlineColor, float lineWidth)
    {
        this.lineMaterial = lineMaterial;
        this.fillMaterial = fillMaterial;
        this.outlineColor = outlineColor;
        this.lineWidth = lineWidth;
    }

    // Objects currently shown. Don't add or remove while iterating; use RemoveWhere for that.
    public IEnumerable<DetectedObject> Active => active.Values;

    public DetectedObject GetOrCreate(string key, out bool isNew)
    {
        if (active.TryGetValue(key, out DetectedObject obj))
        {
            isNew = false;
            return obj;
        }
        isNew = true;
        obj = free.Count > 0 ? free.Pop() : new DetectedObject();
        obj.key = key;
        obj.active = true;
        obj.hasMask = false;
        obj.hasDepth = false;
        obj.partCount = 0;
        obj.flashStart = -1f;
        active.Add(key, obj);
        return obj;
    }

    // Outline part `index` of the object, created on first use.
    public DetectedObject.Part GetPart(DetectedObject obj, int index)
    {
        while (obj.parts.Count <= index)
            obj.parts.Add(new DetectedObject.Part { line = CreateLine(), fill = CreateFill() });
        return obj.parts[index];
    }

    public void Remove(DetectedObject obj)
    {
        if (!obj.active)
            return;
        obj.active = false;
        if (obj.flashStart >= 0f)
        {
            obj.flashStart = -1f;
            foreach (var part in obj.parts)
            {
                part.line.startColor = outlineColor;
                part.line.endColor = outlineColor;
                part.line.widthMultiplier = lineWidth;
            }
        }
        obj.partCount = 0;
        obj.HideAll();
        active.Remove(obj.key);
        free.Push(obj);
    }

    // New outline color / width (settings menu) for every pooled line. Flashing outlines settle to it by themselves.
    public void SetLook(Color color, float width)
    {
        outlineColor = color;
        lineWidth = width;
        foreach (var obj in active.Values)
        {
            if (obj.flashStart < 0f)
                SetLook(obj);
        }
        foreach (var obj in free)
            SetLook(obj);
    }

    void SetLook(DetectedObject obj)
    {
        foreach (var part in obj.parts)
        {
            part.line.startColor = outlineColor;
            part.line.endColor = outlineColor;
            part.line.widthMultiplier = lineWidth;
        }
    }

    public void RemoveWhere(Predicate<DetectedObject> match)
    {
        scratch.Clear();
        foreach (var obj in active.Values)
        {
            if (match(obj))
                scratch.Add(obj);
        }
        foreach (var obj in scratch)
            Remove(obj);
    }

    public void RemoveAll() => RemoveWhere(_ => true);

    LineRenderer CreateLine()
    {
        var line = new GameObject("YoloOutline").AddComponent<LineRenderer>();
        line.useWorldSpace = true;
        line.loop = true;
        line.positionCount = 4;
        line.widthMultiplier = lineWidth;
        line.material = lineMaterial;
        line.startColor = outlineColor;
        line.endColor = outlineColor;
        return line;
    }

    MeshFilter CreateFill()
    {
        // Vertices are written in world space, so the object stays at the origin.
        var fillObject = new GameObject("YoloFill");
        var fill = fillObject.AddComponent<MeshFilter>();
        fill.sharedMesh = new Mesh { name = "YoloFill" };
        fill.sharedMesh.MarkDynamic();
        var renderer = fillObject.AddComponent<MeshRenderer>();
        renderer.sharedMaterial = fillMaterial;
        renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        renderer.receiveShadows = false;
        fillObject.SetActive(false);
        return fill;
    }
}

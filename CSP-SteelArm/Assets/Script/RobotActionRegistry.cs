using System.Collections.Generic;
using System.Reflection;
using UnityEngine;

public static class RobotActionRegistry
{
    [System.Serializable]
    public class ActionData
    {
        public string name;
        public string[] actions;
    }

    [System.Serializable]
    public class ActionDatabase
    {
        public ActionData[] objects;
    }

    private static Dictionary<string, string[]> actionMap = new Dictionary<string, string[]>();

    public static string[] GetActions(string name) => actionMap.ContainsKey(name) ? actionMap[name] : null;

    public static void Initialize(string jsonText)
    {
        // JsonUtility를 이용해 파싱 (__comment__ 필드는 선언하지 않아도 에러 없이 자동 무시됨)
        var database = JsonUtility.FromJson<ActionDatabase>(jsonText);
        
        actionMap.Clear();
        foreach (var obj in database.objects)
        {
            if (!actionMap.ContainsKey(obj.name))
            {
                actionMap.Add(obj.name, obj.actions);
            }
        }
    }

    public static void ExecuteAction(string methodName)
    {
        MethodInfo method = typeof(RobotActionRegistry).GetMethod(methodName);
        if (method == null)
        {
            Debug.LogError($"Method '{methodName}' not found in RobotActionRegistry.");
            return;
        }

        method?.Invoke(null, null); // 정적 메서드 실행
    }

    // ROS methods below

    public static void Point()
    {
        // send somewhat signal to ROS here
    }

    public static void Handshake()
    {
        
    }

    public static void Push()
    {
        
    }

    public static void Wave()
    {
        
    }

    public static void Grab()
    {
        
    }
     
    public static void Click()
    {
        
    }

    public static void Pull() {}
}

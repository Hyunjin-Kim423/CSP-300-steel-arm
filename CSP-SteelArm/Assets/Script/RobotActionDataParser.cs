
using UnityEngine;

public class RobotActionDataParser : MonoBehaviour
{
    public void Start()
    {
        // Assets/Resources 폴더에 yolo_dectection_list.json 파일이 있다고 가정 (.json 확장자 제외)
        TextAsset jsonFile = Resources.Load<TextAsset>("yolo_dectection_list");
        
        if (jsonFile != null)
        {
            RobotActionRegistry.Initialize(jsonFile.text);
        }
        else
        {
            Debug.LogError("yolo_dectection_list JSON 파일을 찾을 수 없습니다!");
        }
    }
}
using System.Collections;   
using System.Collections.Generic;
using UnityEngine;

public class ObjectMenu : MonoBehaviour
{
    // struct or SO _currentData;

    Transform mainCam;
    const float radius = .1f;
    const float orbitSpeed = 10f;
    public Vector3 fixedPosition;

    public GameObject _menuPrefab;
    [SerializeField] TMPro.TextMeshProUGUI _objectNameText;
    public int _menuCount = 4;
    [Tooltip("Angle offset for the first menu item (0 is right, 90 is up)")]
    public float startAngleOffset = -90f;
    public float _menuRadius = 45; // pixels

    [Space]
    static public float duration = 0.6f;
    public float staggerDelay = 0.05f;

    List<RectTransform> spawnedUIList = new List<RectTransform>();
    bool isClosing;

    public bool IsClosing => isClosing;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        mainCam = Camera.main.transform;
    }

    // Update is called once per frame
    void Update()
    {
        

        if (mainCam != null)
        {
            Vector3 newPos = fixedPosition + (mainCam.position - fixedPosition).normalized * radius;
            transform.position = Vector3.Lerp(transform.position, newPos, Time.deltaTime * orbitSpeed);

            Vector3 lookDirection = mainCam.position - transform.position;
            transform.rotation = Quaternion.LookRotation(-lookDirection, Vector3.up);
        }
    }

    public void Open(RosMessageTypes.Yolo.DetectionMsg data) // send data through parameter
    {
        // bind here
        var jsonData = RobotActionRegistry.GetActions(data.class_name);
        _menuCount = jsonData != null ? jsonData.Length + 1 : 3;
        _objectNameText.text = data.class_name;
        // ===

        float angleStep = 360f / _menuCount;
        for (int i = 0; i < _menuCount; i++)
        {
            // 각도 계산 (도 -> 라디안)
            float currentAngle = startAngleOffset + (i * angleStep);
            float radian = currentAngle * Mathf.Deg2Rad;

            // UI X, Y 피벗 좌표계 상의 원형 목표 위치 (anchoredPosition 기준)
            float targetX = Mathf.Cos(radian) * _menuRadius;
            float targetY = Mathf.Sin(radian) * _menuRadius;
            Vector2 targetPosition = new Vector2(targetX, targetY);

            // UI 생성 및 부모 설정
            GameObject spawnedObj = Instantiate(_menuPrefab, transform);

            if (i > 0 && jsonData != null)
            {
                string methodName = jsonData[i - 1];
                spawnedObj.GetComponentInChildren<TMPro.TextMeshProUGUI>().text = methodName;
                spawnedObj.GetComponent<UnityEngine.UI.Button>().onClick.AddListener(() => 
                    RobotActionRegistry.ExecuteAction(methodName));
            }

            RectTransform rectTransform = spawnedObj.GetComponent<RectTransform>();

            // UI 앵커 및 피벗을 중앙(0.5, 0.5)으로 정렬
            rectTransform.anchoredPosition = Vector2.zero;
            rectTransform.localScale = Vector3.one;

            spawnedUIList.Add(rectTransform);

            // 초기 상태: 크기 0, 중앙 위치
            rectTransform.anchoredPosition = Vector2.zero;
            rectTransform.localScale = Vector3.zero;

            StartCoroutine(AnimateUISpawn(rectTransform, targetPosition, i * staggerDelay));
        }

        Transform first = spawnedUIList[0];
        first.GetComponentInChildren<TMPro.TextMeshProUGUI>().text = "Close"; 
        first.GetComponent<UnityEngine.UI.Button>().onClick.AddListener(Close);
    }

    // 메뉴 전체 닫기: 버튼들이 차례로 들어간 뒤 패널(배경 + 이름)까지 줄어들고, 끝나면 메뉴 오브젝트 자체를 Destroy
    // 이미 닫히는 중이면 무시하므로 여러 번 호출해도 안전
    public void Close()
    {
        if (isClosing) return;
        isClosing = true;

        // 아직 진행 중인 펼침 애니메이션이 닫힘 애니메이션과 같은 값을 덮어쓰지 않도록 중단
        StopAllCoroutines();
        StartCoroutine(CloseRoutine());
    }

    private IEnumerator CloseRoutine()
    {
        // 닫히는 동안 버튼이 다시 눌리지 않도록 리스너 제거
        foreach (RectTransform rect in spawnedUIList)
        {
            if (rect != null && rect.TryGetComponent(out UnityEngine.UI.Button button))
                button.onClick.RemoveAllListeners();
        }

        // 버튼: 마지막 버튼부터 staggerDelay 간격으로 안쪽으로 접힘
        int count = spawnedUIList.Count;
        for (int i = count - 1; i >= 0; i--)
        {
            if (spawnedUIList[i] != null)
                StartCoroutine(AnimateCloseSingle(spawnedUIList[i], (count - 1 - i) * staggerDelay));
        }

        // 패널: 마지막 버튼이 접히기 시작할 때 함께 줄어들기 시작
        float panelDelay = Mathf.Max(0, count - 1) * staggerDelay;
        if (panelDelay > 0)
            yield return new WaitForSeconds(panelDelay);

        Vector3 startScale = transform.localScale;
        float elapsed = 0f;
        while (elapsed < duration)
        {
            elapsed += Time.deltaTime;
            float t = Mathf.Clamp01(elapsed / duration);
            transform.localScale = Vector3.LerpUnclamped(startScale, Vector3.zero, EaseInBack(t));
            yield return null;
        }

        // 버튼은 자식이므로 메뉴와 함께 제거됨
        Destroy(gameObject);
    }

    private IEnumerator AnimateUISpawn(RectTransform targetRect, Vector2 targetPos, float delay)
    {
        if (delay > 0)
            yield return new WaitForSeconds(delay);

        float elapsed = 0f;
        float originScale = _menuPrefab.transform.localScale.x;

        while (elapsed < duration)
        {
            elapsed += Time.deltaTime;
            float t = Mathf.Clamp01(elapsed / duration);

            // Ease Out Back 수식 적용 (목표치보다 살짝 넘어갔다가 부드럽게 안착)
            float easeT = EaseOutBack(t);

            // 위치 및 크기 보간 (Vector2.LerpUnclamped 사용)
            targetRect.anchoredPosition = Vector2.LerpUnclamped(Vector2.zero, targetPos, easeT);
            targetRect.localScale = Vector3.LerpUnclamped(Vector3.zero, Vector3.one * originScale, easeT);

            yield return null;
        }

        // 정확한 최종값 고정
        targetRect.anchoredPosition = targetPos;
    }

    // 버튼 하나를 중앙으로 접어 넣는 애니메이션 (제거는 CloseRoutine에서 메뉴 전체와 함께 처리)
    private IEnumerator AnimateCloseSingle(RectTransform targetRect, float delay)
    {
        if (delay > 0) 
            yield return new WaitForSeconds(delay);

        Vector2 startPos = targetRect.anchoredPosition;
        Vector3 startScale = targetRect.localScale;

        float elapsed = 0f;
        while (elapsed < duration)
        {
            elapsed += Time.deltaTime;
            float t = Mathf.Clamp01(elapsed / duration);

            // 닫힐 때는 안쪽으로 살짝 당겨졌다가 빠르게 쏙 들어가는 EaseInBack 사용
            float easeT = EaseInBack(t);

            targetRect.anchoredPosition = Vector2.LerpUnclamped(startPos, Vector2.zero, easeT);
            targetRect.localScale = Vector3.LerpUnclamped(startScale, Vector3.zero, easeT);

            yield return null;
        }
    }

    // Ease Out Back 수학 함수
    private float EaseOutBack(float x)
    {
        const float c1 = 1.70158f;
        const float c3 = c1 + 1f;
        return 1f + c3 * Mathf.Pow(x - 1f, 3f) + c1 * Mathf.Pow(x - 1f, 2f);
    }

    private float EaseInBack(float x)
    {
        const float c1 = 1.70158f;
        const float c3 = c1 + 1f;
        return c3 * x * x * x - c1 * x * x;
    }
}

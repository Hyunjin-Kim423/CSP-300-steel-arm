using System.Collections.Generic;
using TMPro;
using UnityEngine;

// Texts in Korean and English, and the matching font (AppSettings.language).
//   Loc.T("tab.scan")              -> "스캔" / "Scan"
//   Loc.F("menu.actions", 3)       -> "동작 3개" / "3 actions"
//   Loc.ApplyFont(text)            -> Korean font (NanumGothicBold SDF) in Korean, the default TMP font in English
public static class Loc
{
    // key -> { Korean, English }
    static readonly Dictionary<string, string[]> s_Texts = new Dictionary<string, string[]>
    {
        // settings menu
        { "settings.title", new[] { "설정", "Settings" } },
        { "settings.hint", new[] { "왼쪽 컨트롤러 메뉴 버튼으로\n열기 · 닫기", "Left controller menu button\nopens and closes" } },
        { "settings.reset", new[] { "설정 초기화", "Reset settings" } },
        { "tab.scan", new[] { "스캔", "Scan" } },
        { "tab.menu", new[] { "메뉴", "Menu" } },
        { "tab.display", new[] { "표시", "Display" } },
        { "tab.advanced", new[] { "고급 · 연결", "Advanced" } },
        { "tab.language", new[] { "언어", "Language" } },
        { "page.advanced", new[] { "고급 · 연결 상태", "Advanced · Connection" } },

        // scan
        { "scan.style", new[] { "표시 방식", "Highlight" } },
        { "scan.style.desc", new[] { "사물을 어떤 모양으로 표시할지", "How detected objects are drawn" } },
        { "opt.silhouette", new[] { "실루엣", "Silhouette" } },
        { "opt.box", new[] { "바운딩 박스", "Bounding box" } },
        { "scan.replace", new[] { "다시 스캔할 때", "When scanning again" } },
        { "scan.replace.desc", new[] { "이전 스캔 결과를 어디까지 지울지", "Which earlier results a new scan replaces" } },
        { "opt.inView", new[] { "보이는 곳만", "In view only" } },
        { "opt.all", new[] { "전체", "Everything" } },
        { "scan.wave", new[] { "스캔 파동 효과", "Scan wave" } },
        { "scan.wave.desc", new[] { "스캔할 때 주변 표면을 따라 퍼지는 빛", "Light spreading over surfaces while scanning" } },
        { "scan.waveIntensity", new[] { "파동 밝기", "Wave brightness" } },

        // menu
        { "menu.yaw", new[] { "좌우로 따라오는 최대 각도", "Follows the gaze sideways up to" } },
        { "menu.pitch", new[] { "위아래로 따라오는 최대 각도", "Follows the gaze up / down up to" } },
        { "menu.distance", new[] { "기본 거리 · 조이스틱으로도 조절", "Distance · also with the thumbstick" } },
        { "menu.mirror", new[] { "버튼 좌우 자동 전환", "Buttons switch sides" } },
        { "menu.mirror.desc", new[] { "버튼을 항상 사물 반대편에 두기", "Keep the buttons on the side away from the object" } },

        // display
        { "display.colors", new[] { "색상 조합", "Colors" } },
        { "display.colors.desc", new[] { "동작 있음 / 없음", "With actions / without" } },
        { "opt.greenRed", new[] { "초록 / 빨강", "Green / Red" } },
        { "opt.blueOrange", new[] { "파랑 / 주황", "Blue / Orange" } },
        { "display.fill", new[] { "채우기 투명도", "Fill opacity" } },
        { "display.width", new[] { "외곽선 두께", "Outline width" } },
        { "display.width.desc", new[] { "인식된 사물의 흰 외곽선", "White outline of detected objects" } },
        { "opt.thin", new[] { "얇게", "Thin" } },
        { "opt.normal", new[] { "보통", "Normal" } },
        { "opt.thick", new[] { "두껍게", "Thick" } },
        { "display.label", new[] { "사물 이름 라벨", "Object name label" } },
        { "display.label.desc", new[] { "메뉴를 열 때 사물 위에 이름 표시", "Show the name above the object when its menu opens" } },

        // advanced
        { "adv.scanMode", new[] { "스캔 방식", "Scan mode" } },
        { "adv.scanMode.desc", new[] { "버튼: A / X로 스캔 · 실시간: 계속 스캔", "Button: scan with A / X · Live: scan continuously" } },
        { "opt.button", new[] { "버튼", "Button" } },
        { "opt.live", new[] { "실시간", "Live" } },
        { "adv.tracking", new[] { "트래킹 사용", "Use tracking" } },
        { "adv.tracking.desc", new[] { "실시간 모드에서만 · 끊기면 일반 감지로 전환", "Live mode only · falls back to plain detections" } },
        { "adv.log", new[] { "로그 표시", "Show log" } },
        { "adv.log.desc", new[] { "[steelarm] 로그를 화면에 띄우기", "Show [steelarm] logs in view" } },
        { "status.ros", new[] { "ROS 연결", "ROS connection" } },
        { "status.connected", new[] { "연결됨", "Connected" } },
        { "status.error", new[] { "연결 오류", "Connection error" } },
        { "status.notStarted", new[] { "시작 전", "Not started" } },
        { "status.topic", new[] { "결과 토픽", "Result topic" } },
        { "status.scanTopic", new[] { "{0} · 버튼 스캔", "{0} · button scan" } },
        { "status.trackingTopic", new[] { "{0} · 트래킹", "{0} · tracking" } },
        { "status.detectionTopic", new[] { "{0} · 일반 감지", "{0} · detections" } },
        { "status.waiting", new[] { "결과 대기 중", "Waiting for results" } },
        { "status.last", new[] { "마지막 결과", "Last result" } },
        { "status.lastValue", new[] { "{0:0.0}초 전 · 사물 {1}개", "{0:0.0} s ago · {1} object(s)" } },
        { "status.none", new[] { "없음", "None" } },

        // language
        { "lang.title", new[] { "표시 언어", "Display language" } },
        { "lang.desc", new[] { "설정, 메뉴, 안내 문구의 언어", "Language of settings, menus and hints" } },

        // object menu
        { "menu.confidence", new[] { "신뢰도 {0:0.00}", "Confidence {0:0.00}" } },
        { "menu.actions", new[] { "동작 {0}개", "{0} actions" } },
        { "menu.action1", new[] { "동작 1개", "1 action" } },
        { "menu.noActions", new[] { "동작 없음", "No actions" } },

        // start
        { "start.hint", new[] { "B 또는 Y를 누르세요", "Press B or Y" } },
    };

    public static bool IsKorean => AppSettings.Current.language == AppSettings.Language.Korean;

    public static string T(string key)
    {
        if (s_Texts.TryGetValue(key, out string[] texts))
            return texts[IsKorean ? 0 : 1];
        Debug.LogWarning("[steelarm] Missing text: " + key);
        return key;
    }

    public static string F(string key, params object[] args) => string.Format(T(key), args);

    // ---------- font ----------

    const string KoreanFontPath = "Fonts & Materials/NanumGothicBold SDF"; // under Assets/TextMesh Pro/Resources
    static TMP_FontAsset s_KoreanFont;
    static bool s_Searched;

    public static TMP_FontAsset KoreanFont
    {
        get
        {
            if (!s_Searched)
            {
                s_Searched = true;
                s_KoreanFont = Resources.Load<TMP_FontAsset>(KoreanFontPath);
                if (s_KoreanFont == null)
                    Debug.LogWarning($"[steelarm] Korean font not found at Resources/{KoreanFontPath}; Korean text may show as boxes.");
            }
            return s_KoreanFont;
        }
    }

    // Korean font in Korean (it also has Latin letters), the default TMP font in English.
    // forceKorean: for text that is always Korean (e.g. the "한국어" option).
    public static void ApplyFont(TMP_Text text, bool forceKorean = false)
    {
        if (text == null)
            return;
        TMP_FontAsset font = (IsKorean || forceKorean) && KoreanFont != null ? KoreanFont : TMP_Settings.defaultFontAsset;
        if (font != null && text.font != font)
            text.font = font;
    }

    public static void Set(TMP_Text text, string key)
    {
        if (text == null)
            return;
        text.text = T(key);
        ApplyFont(text);
    }
}

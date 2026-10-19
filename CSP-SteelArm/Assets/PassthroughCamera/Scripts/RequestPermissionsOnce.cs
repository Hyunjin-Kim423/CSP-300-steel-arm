// Copyright (c) Meta Platforms, Inc. and affiliates.

using UnityEngine;
using UnityEngine.SceneManagement;

namespace PassthroughCameraSamples
{
    internal static class RequestPermissionsOnce
    {
        private static bool s_permissionsRequested;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void AfterSceneLoad()
        {
            // RuntimeInitializeLoadType.AfterSceneLoad runs after the first scene has
            // already loaded. Requesting only from the sceneLoaded callback therefore
            // skipped permission entirely in a single-scene player build.
            RequestForScene(SceneManager.GetActiveScene());
            SceneManager.sceneLoaded += (scene, _) => RequestForScene(scene);
        }

        private static void RequestForScene(Scene scene)
        {
            if (s_permissionsRequested || scene.name == "StartScene")
                return;

            s_permissionsRequested = true;
            OVRPermissionsRequester.Request(new[]
            {
                OVRPermissionsRequester.Permission.Scene,
                OVRPermissionsRequester.Permission.PassthroughCameraAccess
            });
        }
    }
}

using Cinemachine;
using UnityEngine;

public class GameInitializer : MonoBehaviour
{
    [Header("Punto de Aparición del Dron")]
    public Transform droneSpawnPoint;

    [Header("Prefabs de Drones")]
    public GameObject racerDronePrefab;
    public GameObject cinematicDronePrefab;

    [Header("Prefabs de Escenarios")]
    public GameObject refinerySceneryPrefab;
    public GameObject solarPSceneryPrefab;

    void Awake()
    {
        if (GameSettingsManager.Instance == null)
        {
            Debug.LogWarning("GameSettingsManager no encontrado. Cargando escena y dron por defecto para pruebas.");
            LoadDefaultScene();
        }
        else
        {
            LoadSelectedScene();
        }
    }

    private void LoadSelectedScene()
    {
        LoadScenery(GameSettingsManager.Instance.SelectedScenery);
        LoadDrone(GameSettingsManager.Instance.SelectedDrone);
    }
    private void LoadDefaultScene()
    {
        LoadScenery(SceneryType.REFINERY);
        LoadDrone(DroneType.RACER);
    }
    private void LoadScenery(SceneryType sceneryType)
    {
        GameObject sceneryToInstantiate = null;

        switch (sceneryType)
        {
            case SceneryType.REFINERY:
                sceneryToInstantiate = refinerySceneryPrefab;
                break;
            case SceneryType.SOLARP:
                sceneryToInstantiate = solarPSceneryPrefab;
                break;
        }

        if (sceneryToInstantiate == null)
        {
            Debug.LogError($"El prefab para el escenario '{sceneryType}' no está asignado. Cargando 'Refinery' por defecto.");
            sceneryToInstantiate = refinerySceneryPrefab;
        }

        if (sceneryToInstantiate != null)
        {
            Instantiate(sceneryToInstantiate, Vector3.zero, Quaternion.identity);
        }
        else
        {
            Debug.LogError("¡ERROR FATAL! No hay ningún prefab de escenario asignado. No se puede cargar el nivel.");
        }
    }

    private void LoadDrone(DroneType droneType)
    {
        GameObject droneToInstantiate = null;

        switch (droneType)
        {
            case DroneType.RACER:
                droneToInstantiate = racerDronePrefab;
                break;
            case DroneType.CINEMATIC:
                droneToInstantiate = cinematicDronePrefab;
                break;
        }

        if (droneToInstantiate == null)
        {
            Debug.LogError($"El prefab para el dron '{droneType}' no está asignado. Cargando 'Racer' por defecto.");
            droneToInstantiate = racerDronePrefab;
        }

        if (droneToInstantiate != null && droneSpawnPoint != null)
        {
            GameObject spawnedDrone = Instantiate(droneToInstantiate, droneSpawnPoint.position, droneSpawnPoint.rotation);

            AssignTargetToCamera(spawnedDrone.transform);
        }
        else
        {
            Debug.LogError("¡ERROR FATAL! No se puede crear el dron. Falta el prefab de dron o el 'DroneSpawnPoint'.");
        }
    }

    private void AssignTargetToCamera(Transform droneTransform)
    {
        CinemachineVirtualCamera vcam = FindObjectOfType<CinemachineVirtualCamera>();

        if (vcam != null)
        {
            vcam.Follow = droneTransform;
            vcam.LookAt = droneTransform;
            Debug.Log($"Asignado {droneTransform.name} a la Cinemachine Virtual Camera.");
        }
        else
        {
            OrbitalCamera orbitalCamera = FindObjectOfType<OrbitalCamera>();
            if (orbitalCamera != null)
            {
                orbitalCamera.AssignTarget(droneTransform);
            }
            else
            {
                Debug.LogWarning("No se encontró ninguna cámara (ni OrbitalCamera ni VCam) para asignarle el dron.");
            }
        }
    }
}
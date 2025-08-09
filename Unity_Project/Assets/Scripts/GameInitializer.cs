using Cinemachine;
using UnityEngine;

// Es una buena práctica declarar aquí que vas a usar el namespace de Cinemachine
// si tu proyecto lo utiliza, para evitar errores de compilación.
// Si no usas Cinemachine, puedes borrar la línea siguiente y la sección de Cinemachine más abajo.
// using Cinemachine; 

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
        // 1. Verificamos si venimos del menú con ajustes guardados.
        if (GameSettingsManager.Instance == null)
        {
            Debug.LogWarning("GameSettingsManager no encontrado. Cargando escena y dron por defecto para pruebas.");
            LoadDefaultScene();
        }
        else
        {
            // Si hay ajustes, los usamos para construir la escena.
            LoadSelectedScene();
        }
    }

    private void LoadSelectedScene()
    {
        // --- Cargar el Escenario ---
        LoadScenery(GameSettingsManager.Instance.SelectedScenery);

        // --- Cargar el Dron ---
        LoadDrone(GameSettingsManager.Instance.SelectedDrone);
    }

    // Carga la escena por defecto para poder probar directamente
    private void LoadDefaultScene()
    {
        LoadScenery(SceneryType.REFINERY); // Por defecto carga la refinería
        LoadDrone(DroneType.RACER);      // Por defecto carga el dron de carreras
    }

    // --- MÉTODOS AYUDANTES CENTRALIZADOS ---

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
            sceneryToInstantiate = refinerySceneryPrefab; // Fallback
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
            droneToInstantiate = racerDronePrefab; // Fallback
        }

        if (droneToInstantiate != null && droneSpawnPoint != null)
        {
            // Creamos el dron y guardamos una referencia a su GameObject
            GameObject spawnedDrone = Instantiate(droneToInstantiate, droneSpawnPoint.position, droneSpawnPoint.rotation);

            // --- ¡AQUÍ ESTÁ LA MAGIA PARA LA CÁMARA! ---
            // Le pasamos el 'Transform' del dron recién creado a la función que busca y asigna la cámara.
            AssignTargetToCamera(spawnedDrone.transform);
        }
        else
        {
            Debug.LogError("¡ERROR FATAL! No se puede crear el dron. Falta el prefab de dron o el 'DroneSpawnPoint'.");
        }
    }

    // Esta función busca la cámara y le asigna el dron como objetivo.
    private void AssignTargetToCamera(Transform droneTransform)
    {
        // Buscamos una cámara virtual de Cinemachine en la escena.
        CinemachineVirtualCamera vcam = FindObjectOfType<CinemachineVirtualCamera>();

        if (vcam != null)
        {
            // Si la encontramos, le asignamos el Follow y el Look At.
            vcam.Follow = droneTransform;
            vcam.LookAt = droneTransform;
            Debug.Log($"Asignado {droneTransform.name} a la Cinemachine Virtual Camera.");
        }
        else
        {
            // Mantenemos tu código antiguo como fallback si no encuentras la VCam.
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
using UnityEngine;

// Enums para que las opciones sean claras y a prueba de errores
public enum DroneType { RACER, CINEMATIC }
public enum SceneryType { REFINERY, SOLARP }
public enum FlightModeType { FREESTYLE, PID }

public class GameSettingsManager : MonoBehaviour
{
    // Instancia Singleton que persistirá entre escenas
    public static GameSettingsManager Instance { get; private set; }

    // Propiedades para guardar las elecciones del jugador
    public DroneType SelectedDrone { get; set; }
    public SceneryType SelectedScenery { get; set; }
    public FlightModeType SelectedFlightMode { get; set; }

    private void Awake()
    {
        // Lógica para asegurar que solo haya una instancia de este objeto
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(gameObject); // ¡La magia para que no se destruya!
        }
        else
        {
            Destroy(gameObject);
        }
    }
}
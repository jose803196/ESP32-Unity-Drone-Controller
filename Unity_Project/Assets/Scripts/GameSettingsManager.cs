using UnityEngine;

public enum DroneType { RACER, CINEMATIC }
public enum SceneryType { REFINERY, SOLARP }
public enum FlightModeType { FREESTYLE, PID }

public class GameSettingsManager : MonoBehaviour
{
    public static GameSettingsManager Instance { get; private set; }
    public DroneType SelectedDrone { get; set; }
    public SceneryType SelectedScenery { get; set; }
    public FlightModeType SelectedFlightMode { get; set; }

    private void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(gameObject);
        }
        else
        {
            Destroy(gameObject);
        }
    }
}
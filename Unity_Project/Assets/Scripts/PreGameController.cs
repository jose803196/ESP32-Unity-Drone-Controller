using System;
using System.IO.Ports;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using UnityEngine.SceneManagement;
using TMPro;

public class PreGameController : MonoBehaviour{
    [Header("Configuración del Puerto Serie")]
    [SerializeField] private string portName = "COM3";
    [SerializeField] private int baudRate = 115200;

    // --- CORRECCIÓN DE TIPO: De 'Text' a 'TextMeshProUGUI' ---
    [Header("Referencias a la UI")]
    public TextMeshProUGUI choiceTextDrone;      // <-- CORREGIDO
    public TextMeshProUGUI choiceTextScenery;    // <-- CORREGIDO
    public TextMeshProUGUI choiceTextFlightMode; // <-- CORREGIDO
    public Button startFlightButton;

    [Header("Referencias de Navegación")]
    public Button droneLeft, droneRight;
    public Button sceneryLeft, sceneryRight;
    public Button flightModeLeft, flightModeRight;

    private SerialPort serial;
    private int currentRow = 0;
    private int[] selectedIndices = new int[3] { 0, 0, 0 };
    private string[][] allOptions;

    private bool verticalInputConsumed = false;
    private bool horizontalInputConsumed = false;
    private bool selectionInputConsumed = false;

    void Start()
    {
        allOptions = new string[][] {
        new string[] { "RACER", "CINEMATIC" },
        new string[] { "REFINERY", "SOLARP" },
        new string[] { "FREESTYLE", "PID" }
    };

        ConnectToSerial();
        UpdateVisuals();
    }

    private void ConnectToSerial()
    {
        try
        {
            serial = new SerialPort(portName, baudRate);
            serial.ReadTimeout = 100;
            serial.Open();
            Debug.Log($"<color=green>Puerto serial {portName} abierto correctamente.</color>");
        }
        catch (Exception e)
        {
            Debug.LogError($"Error al abrir el puerto serial: {e.Message}");
        }
    }

    void Update()
    {
        if (serial == null || !serial.IsOpen || serial.BytesToRead <= 0) return;

        try
        {
            string line = serial.ReadLine();
            if (string.IsNullOrWhiteSpace(line)) return;
            string[] values = line.Trim().Split(',');

            if (values.Length == 13)
            {
                int btnDown = int.Parse(values[1]);
                int btnUp = int.Parse(values[0]);
                int btnLeft = int.Parse(values[2]);
                int btnRight = int.Parse(values[3]);
                int btnSelect = int.Parse(values[4]);

                HandleVerticalNavigation(btnUp, btnDown);
                HandleHorizontalNavigation(btnLeft, btnRight);
                HandleSelection(btnSelect);
            }
        }
        catch (Exception) { /* Ignorar errores */ }
    }

    private void HandleVerticalNavigation(int up, int down)
    {
        if (down == 1 && !verticalInputConsumed)
        {
            verticalInputConsumed = true;
            currentRow = (currentRow + 1) % 4;
            UpdateVisuals();
        }
        else if (up == 1 && !verticalInputConsumed)
        {
            verticalInputConsumed = true;
            currentRow--;
            if (currentRow < 0) currentRow = 3;
            UpdateVisuals();
        }
        if (up == 0 && down == 0) verticalInputConsumed = false;
    }

    private void HandleHorizontalNavigation(int left, int right)
    {
        if (currentRow >= 3) return;
        int direction = 0;
        if (right == 1 && !horizontalInputConsumed)
        {
            horizontalInputConsumed = true;
            direction = 1;
        }
        else if (left == 1 && !horizontalInputConsumed)
        {
            horizontalInputConsumed = true;
            direction = -1;
        }
        if (direction != 0)
        {
            int optionCount = allOptions[currentRow].Length;
            selectedIndices[currentRow] = (selectedIndices[currentRow] + direction + optionCount) % optionCount;
            UpdateVisuals();
        }
        if (left == 0 && right == 0) horizontalInputConsumed = false;
    }

    private void HandleSelection(int select)
    {
        if (select == 1 && !selectionInputConsumed)
        {
            selectionInputConsumed = true;
            if (currentRow == 3)
            {
                startFlightButton.onClick.Invoke();
            }
        }
        if (select == 0) selectionInputConsumed = false;
    }

    private void UpdateVisuals()
    {
        // Actualizar el texto de las opciones (esto se queda igual)
        choiceTextDrone.text = allOptions[0][selectedIndices[0]];
        choiceTextScenery.text = allOptions[1][selectedIndices[1]];
        choiceTextFlightMode.text = allOptions[2][selectedIndices[2]];

        // --- NUEVA LÓGICA DE RESALTADO ---
        // En lugar de mover un indicador, le decimos al EventSystem qué objeto seleccionar.
        GameObject objectToSelect = null;
        switch (currentRow)
        {
            case 0: // Fila Drone
                    // Por defecto, seleccionamos el botón izquierdo de la fila
                objectToSelect = droneLeft.gameObject;
                break;
            case 1: // Fila Scenery
                objectToSelect = sceneryLeft.gameObject;
                break;
            case 2: // Fila Flight Mode
                objectToSelect = flightModeLeft.gameObject;
                break;
            case 3: // Fila del Botón Start
                objectToSelect = startFlightButton.gameObject;
                break;
        }

        // ¡La magia está aquí! Esto le dice a Unity que ponga el foco en el objeto.
        if (objectToSelect != null)
        {
            EventSystem.current.SetSelectedGameObject(objectToSelect);
        }
    }

    public void StartGame()
    {
        GameSettingsManager.Instance.SelectedDrone = (DroneType)selectedIndices[0];
        GameSettingsManager.Instance.SelectedScenery = (SceneryType)selectedIndices[1];
        GameSettingsManager.Instance.SelectedFlightMode = (FlightModeType)selectedIndices[2];
        Debug.Log("Opciones guardadas: " + GameSettingsManager.Instance.SelectedDrone);
        Debug.Log("Opciones guardadas: " + GameSettingsManager.Instance.SelectedScenery);
        Debug.Log("Opciones guardadas: " + GameSettingsManager.Instance.SelectedFlightMode);
        SceneManager.LoadScene("GameScene");
    }

    void OnDestroy()
    {
        if (serial != null && serial.IsOpen)
        {
            serial.Close();
        }
    }
}
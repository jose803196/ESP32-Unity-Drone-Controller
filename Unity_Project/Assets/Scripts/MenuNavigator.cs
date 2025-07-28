using UnityEngine;
using System.IO.Ports;
using System;
using UnityEngine.UI;              // Para interactuar con los Botones.
using UnityEngine.EventSystems;    // Para controlar la selección de la UI.
using UnityEngine.SceneManagement; // Para cambiar de escena.

// Este script único maneja tanto la comunicación serie como la navegación del MainMenu.
public class MenuNavegator : MonoBehaviour{
    // --- Variables de Comunicación Serial ---
    private SerialPort serial;
    [SerializeField] private string portName = "COM3";
    [SerializeField] private int baudRate = 115200;

    // --- Variables para el Control del Menú ---
    [Header("UI del Menú Principal")]
    [Tooltip("Arrastra aquí los botones del menú en orden: PLAY primero, QUIT después.")]
    [SerializeField] private Button[] menuButtons; // Un array para los botones PLAY y QUIT

    private int selectedIndex = 0;
    private bool upButtonPressed = false;
    private bool downButtonPressed = false;
    private bool selectButtonPressed = false;

    void Start(){
        // Parte 1: Abrir el puerto serie (tu código original)
        try{
            serial = new SerialPort(portName, baudRate);
            serial.ReadTimeout = 1000;
            serial.Open();
            Debug.Log("Puerto serial abierto: " + portName);
        }
        catch (Exception e){
            Debug.LogError("Error al abrir el puerto serial: " + e.Message);
        }

        // Parte 2: Resaltar el primer botón del menú
        if (menuButtons != null && menuButtons.Length > 0){
            EventSystem.current.SetSelectedGameObject(menuButtons[selectedIndex].gameObject);
        }
    }

    void Update(){
        if (serial != null && serial.IsOpen){
            try{
                if (serial.BytesToRead > 0){
                    string line = serial.ReadLine();
                    string[] values = line.Split(',');

                    if (values.Length == 13){
                        // 1. Leemos los datos de los botones que nos interesan
                        int btnDown = int.Parse(values[0]);   // "Avanzar" = Botón de ABAJO
                        int btnUp = int.Parse(values[1]);     // "Retroceder" = Botón de ARRIBA
                        int btnSelect = int.Parse(values[4]);   // Botón del Joystick = Botón de SELECCIÓN

                        // 2. Lógica de navegación vertical (Subir y Bajar)
                        HandleNavigation(btnUp, btnDown);

                        // 3. Lógica de selección (Hacer clic)
                        HandleSelection(btnSelect);
                    }
                }
            }
            catch (Exception){
                // Dejamos tu catch vacío original
            }
        }
    }

    // --- Lógica de Navegación del Menú (Añadida a tu código) ---
    private void HandleNavigation(int up, int down){
        if (up == 1 && !upButtonPressed){
            upButtonPressed = true;
            selectedIndex--;
            if (selectedIndex < 0){
                selectedIndex = menuButtons.Length - 1;
            }
            EventSystem.current.SetSelectedGameObject(menuButtons[selectedIndex].gameObject);
        }
        else if (up == 0){
            upButtonPressed = false;
        }

        if (down == 1 && !downButtonPressed){
            downButtonPressed = true;
            selectedIndex++;
            if (selectedIndex >= menuButtons.Length){
                selectedIndex = 0;
            }
            EventSystem.current.SetSelectedGameObject(menuButtons[selectedIndex].gameObject);
        }
        else if (down == 0){
            downButtonPressed = false;
        }
    }

    private void HandleSelection(int select){
        if (select == 1 && !selectButtonPressed){
            selectButtonPressed = true;
            menuButtons[selectedIndex].onClick.Invoke();
        }
        else if (select == 0){
            selectButtonPressed = false;
        }
    }

    // --- Funciones que serán llamadas por los botones en la UI ---
    public void PlayGame(){
        SceneManager.LoadScene("PreGameMenu");
    }

    public void QuitGame(){
        Application.Quit();
        Debug.Log("Saliendo del juego...");
    }
    void OnDestroy(){
        if (serial != null && serial.IsOpen)
        {
            serial.Close();
            Debug.Log("Puerto serial cerrado");
        }
    }
}
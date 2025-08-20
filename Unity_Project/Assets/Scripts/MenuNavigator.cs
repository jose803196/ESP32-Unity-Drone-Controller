using UnityEngine;
using System.IO.Ports;
using System;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using UnityEngine.SceneManagement;

public class MenuNavegator : MonoBehaviour{
    private SerialPort serial;
    [SerializeField] private string portName = "COM3";
    [SerializeField] private int baudRate = 115200;

    [Header("UI del Menú Principal")]
    [Tooltip("Arrastra aquí los botones del menú en orden: PLAY primero, QUIT después.")]
    [SerializeField] private Button[] menuButtons;

    private int selectedIndex = 0;
    private bool upButtonPressed = false;
    private bool downButtonPressed = false;
    private bool selectButtonPressed = false;

    void Start(){
        try{
            serial = new SerialPort(portName, baudRate);
            serial.ReadTimeout = 1000;
            serial.Open();
            Debug.Log("Puerto serial abierto: " + portName);
        }
        catch (Exception e){
            Debug.LogError("Error al abrir el puerto serial: " + e.Message);
        }
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
                        int btnDown = int.Parse(values[0]);
                        int btnUp = int.Parse(values[1]);
                        int btnSelect = int.Parse(values[4]);

                        HandleNavigation(btnUp, btnDown);

                        HandleSelection(btnSelect);
                    }
                }
            }
            catch (Exception){
            }
        }
    }

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
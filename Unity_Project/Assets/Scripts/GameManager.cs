// GameManager.cs
using UnityEngine;
using TMPro; // Asegúrate de tener esta línea para usar TextMeshPro

public class GameManager : MonoBehaviour
{
    [Header("Conexiones UI (desde el Inspector)")]
    // Estas son las variables que ya tienes en tu Inspector
    public TextMeshProUGUI CollectiblesNumberText; // El texto para el contador actual (ej. 00)
    public TextMeshProUGUI TotalCollectibleNumberText; // El texto para el total (ej. / 10)

    // --- Variables internas para gestionar la puntuación ---
    private int currentCollectibles = 0;
    private int totalCollectibles = 0;

    // --- Singleton para que el Dron pueda acceder fácilmente a este script ---
    public static GameManager Instance { get; private set; }

    void Awake()
    {
        // Lógica del Singleton para asegurar que solo haya una instancia
        if (Instance != null && Instance != this)
        {
            Destroy(gameObject);
        }
        else
        {
            Instance = this;
        }
    }

    void Start()
    {
        // --- LÓGICA AUTOMÁTICA DE CONTEO ---
        // Este método se ejecuta DESPUÉS de que el GameInitializer haya creado el escenario.

        // 1. Encuentra todos los GameObjects en la escena que tengan la etiqueta "Collectible".
        GameObject[] allCollectiblesInScene = GameObject.FindGameObjectsWithTag("Collectible");

        // 2. El número total es simplemente la cantidad de objetos que encontró.
        totalCollectibles = allCollectiblesInScene.Length;

        // 3. Reinicia el contador actual a cero.
        currentCollectibles = 0;

        // 4. Actualiza la UI para mostrar el estado inicial (ej. "00 / 10").
        UpdateScoreUI();
    }

    // --- MÉTODO PÚBLICO QUE SERÁ LLAMADO POR EL DRON ---
    public void AddCollectible()
    {
        currentCollectibles++;
        UpdateScoreUI();

        // (Opcional) Comprueba si se ha ganado el nivel
        if (currentCollectibles >= totalCollectibles)
        {
            Debug.Log("¡Nivel completado! Todos los coleccionables recogidos.");
            // Aquí podrías activar un panel de victoria, etc.
        }
    }

    private void UpdateScoreUI()
    {
        // Actualiza los dos textos por separado, como en tu configuración.
        // El formato "00" asegura que siempre tenga al menos dos dígitos (ej. 01, 02, ... 10).
        if (CollectiblesNumberText != null)
        {
            CollectiblesNumberText.text = currentCollectibles.ToString("00");
        }

        if (TotalCollectibleNumberText != null)
        {
            TotalCollectibleNumberText.text = totalCollectibles.ToString("00");
        }
    }
}
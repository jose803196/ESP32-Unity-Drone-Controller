using UnityEngine;
using TMPro;

public class GameManager : MonoBehaviour
{
    [Header("Conexiones UI (desde el Inspector)")]
    public TextMeshProUGUI CollectiblesNumberText;
    public TextMeshProUGUI TotalCollectibleNumberText;

    private int currentCollectibles = 0;
    private int totalCollectibles = 0;
    public static GameManager Instance { get; private set; }

    void Awake()
    {
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
        GameObject[] allCollectiblesInScene = GameObject.FindGameObjectsWithTag("Collectible");
        totalCollectibles = allCollectiblesInScene.Length;
        currentCollectibles = 0;
        UpdateScoreUI();
    }
    public void AddCollectible()
    {
        currentCollectibles++;
        UpdateScoreUI();
        if (currentCollectibles >= totalCollectibles)
        {
            Debug.Log("¡Nivel completado! Todos los coleccionables recogidos.");
            // Colocar Panel de Victoria
        }
    }

    private void UpdateScoreUI()
    {
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
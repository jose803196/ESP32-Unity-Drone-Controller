using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class GameManager : MonoBehaviour
{
    public TMP_Text collectiblesNumbersText;
    private int collectiblesNumber;

    public TMP_Text totalCollectibleNumbersText;
    private int totalCollectibleNumbers;
    void Start()
    {
        totalCollectibleNumbers = transform.childCount;
        totalCollectibleNumbersText.text =totalCollectibleNumbers.ToString();
    }

    
    void Update()
    {
        if (transform.childCount <= 0)
        {
            Debug.Log("Win");
        }
    }

    public void AddCollectible()
    {
        collectiblesNumber++;
        collectiblesNumbersText.text = collectiblesNumber.ToString();
    }
}

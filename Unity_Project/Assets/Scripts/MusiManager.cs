using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MusiManager : MonoBehaviour
{
    private static MusiManager instance;
    void Start()
    {
        if (instance == null)
        {
            instance = this;
            DontDestroyOnLoad(this.gameObject);
        }
        else 
        { 
            Destroy(this.gameObject);
        }
    }
}

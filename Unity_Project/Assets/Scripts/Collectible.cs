using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Collectible : MonoBehaviour
{
    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Dron"))
        {
            FindAnyObjectByType<GameManager>().AddCollectible();
            Destroy(gameObject);
        }
    }


}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Collectible : MonoBehaviour
{
    [Header("Efectos")]
    [Tooltip("El sonido que se reproducirá al recoger este objeto.")]
    public AudioClip collectionSound;

    private bool isCollected = false;

    private void OnTriggerEnter(Collider other)
    {
        if (isCollected)
        {
            return;
        }
        if (other.CompareTag("Dron"))
        {
            isCollected = true;

            AudioSource playerAudioSource = other.GetComponent<AudioSource>();
            if (playerAudioSource != null && collectionSound != null)
            {
                playerAudioSource.PlayOneShot(collectionSound);
            }
            if (GameManager.Instance != null)
            {
                GameManager.Instance.AddCollectible();
            }
            else
            {
                Debug.LogError("¡Se recogió un coleccionable, pero no se encontró la instancia del GameManager!");
            }
            Destroy(gameObject);
        }
    }
}
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Collectible : MonoBehaviour
{
    // Una bandera para asegurarnos de que solo se pueda recoger una vez.
    // Evita bugs si el dron toca el coleccionable en dos frames seguidos.
    private bool isCollected = false;

    private void OnTriggerEnter(Collider other)
    {
        // Si ya fue recogido, salimos inmediatamente.
        if (isCollected)
        {
            return;
        }

        // Comprobamos si fue el dron quien lo tocó.
        if (other.CompareTag("Dron"))
        {
            // Marcamos como recogido INMEDIATAMENTE para evitar dobles llamadas.
            isCollected = true;

            // --- EL CAMBIO CLAVE ---
            // En lugar de buscar, vamos directamente a la "marcación rápida".
            // Es miles de veces más rápido y más limpio.
            if (GameManager.Instance != null)
            {
                GameManager.Instance.AddCollectible();
            }
            else
            {
                Debug.LogError("¡Se recogió un coleccionable, pero no se encontró la instancia del GameManager!");
            }

            // Destruimos el coleccionable.
            Destroy(gameObject);
        }
    }
}

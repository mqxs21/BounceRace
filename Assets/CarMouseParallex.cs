using System;
using UnityEngine;

public class CarMouseParallex : MonoBehaviour
{
    public float maxX = 192.9f;
    public float minX = 22.6f;
    public float maxY = 398.8f;
    public float minY = 309.8f;


    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        Vector2 mousePos = Input.mousePosition;
        float halfScreenWidth = Screen.width / 2f;
        float halfScreenHeight = Screen.height / 2f;
        float xOffset = mousePos.x - halfScreenWidth;
        float yOffset = mousePos.y - halfScreenHeight;
        
        float targetX = -Mathf.Lerp(minX, maxX, Input.mousePosition.x / Screen.width);
        float targetY = -Mathf.Lerp(minY, maxY, Input.mousePosition.y / Screen.height);
        transform.position = new Vector3(targetX, targetY, transform.position.z);
    }
}

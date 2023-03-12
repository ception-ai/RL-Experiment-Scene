using System.Collections.Generic;
using UnityEngine;

public class GeneratingPiles : MonoBehaviour
{
    private Terrain terrain;
    private float xSize, zSize;
    public int pileResolution = 10;
    public int piles = 5;
    public float depth = 5f;
    public int scale = 2;


    void Start()
    {
        terrain = Terrain.activeTerrain;

        xSize = terrain.terrainData.size.x;
        zSize = terrain.terrainData.size.z;

        generatePiles();
    }

    private void generatePiles()
    {
        int resolution = terrain.terrainData.heightmapResolution;

        // setting initial terrain shape
        terrain.terrainData.SetHeights(0, 0, new float[resolution, resolution]);
        terrain.terrainData.size = new Vector3(xSize, depth, zSize);

        // creating random terrain piles
        for (int i = 0; i < piles; i++)
        {
            terrain.terrainData.SetHeights((int)Random.Range(0f, resolution - pileResolution), (int)Random.Range(0f, resolution - pileResolution), getHeights());
        }

    }


    private float[,] getHeights()
    {

        float[,] heights = new float[pileResolution, pileResolution];

        for (int x = 0; x < pileResolution; x++)
        {
            for (int z = 0; z < pileResolution; z++)
            {
                heights[x, z] = GetHeightWorldCoords(x, z);

            }
        }

        return heights;
    }



    float GetHeightWorldCoords(int x, int z)
    {
        float xCoord = (float)x / xSize * scale;
        float zCoord = (float)z / zSize * scale;

        return Mathf.PerlinNoise(xCoord, zCoord);
    }

}
